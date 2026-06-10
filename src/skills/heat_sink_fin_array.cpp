// @lat: [[engine/skills#heat_sink_fin_array]]

#include "heat_sink_fin_array.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepBndLib.hxx>
#include <Bnd_Box.hxx>
#include <gp.hxx>
#include <gp_Ax2.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <vector>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::skill::heat_sink_fin_array {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Spec library ─────────────────────────────────────────────────────────
//
// Industry-standard extrusion series for finned heat sinks.  Each entry is
// a self-consistent (pitch, fin_thickness, fin_height) tuple corresponding
// to commercially available stock.

namespace {

struct SpecEntry {
    const char* key;
    double      pitch_mm;
    double      fin_thickness_mm;
    double      fin_height_mm;
};

constexpr std::array<SpecEntry, 4> kSpecTable {{
    { "T-30",  3.0,  1.0,  8.0 },   // small electronics, AR=8
    { "T-50",  5.0,  1.5, 15.0 },   // medium, AR=10
    { "T-80",  8.0,  2.0, 20.0 },   // power supplies, AR=10
    { "T-120", 12.0, 3.0, 30.0 },   // large equipment, AR=10
}};

}  // namespace

FinSpec finSpecFor(const std::string& key)
{
    for (const auto& e : kSpecTable) {
        if (key == e.key) return { e.pitch_mm, e.fin_thickness_mm, e.fin_height_mm };
    }
    return {};
}

// ── Resolution (spec → effective params, possibly from overrides) ────────

namespace {

FinSpec resolveSpec(const Input& in)
{
    if (!in.spec_key.empty()) return finSpecFor(in.spec_key);
    // No spec — caller supplied explicit overrides.
    return { in.fin_pitch_mm, in.fin_thickness_mm, in.fin_height_mm };
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (!in.spec_key.empty()) {
        FinSpec s = finSpecFor(in.spec_key);
        if (s.pitch_mm <= 0.0) {
            r.add("DFM-FIN-SP", "error",
                  "heat_sink_fin_array: unknown spec_key '" + in.spec_key +
                  "' (supported: T-30/T-50/T-80/T-120)");
            return r;
        }
    } else {
        // Explicit override path — all three must be positive.
        if (in.fin_thickness_mm <= 0.0 ||
            in.fin_height_mm    <= 0.0 ||
            in.fin_pitch_mm     <= 0.0) {
            r.add("DFM-INPUT", "error",
                  "heat_sink_fin_array: spec_key empty AND override "
                  "fin_thickness/height/pitch not all > 0");
            return r;
        }
    }
    if (in.fin_count < 2) {
        r.add("DFM-FIN-N", "error",
              "heat_sink_fin_array: fin_count must be ≥ 2");
    }
    const FinSpec eff = resolveSpec(in);

    if (eff.fin_thickness_mm > 0.0) {
        const double ar = eff.fin_height_mm / eff.fin_thickness_mm;
        if (ar > 10.0) {
            r.add("DFM-FIN-AR", "error",
                  "heat_sink_fin_array: aspect ratio " + std::to_string(ar) +
                  " > 10 : 1 (not manufacturable)");
        }
    }
    if (eff.fin_thickness_mm > 0.0 && eff.pitch_mm > 0.0 &&
        eff.fin_thickness_mm >= eff.pitch_mm) {
        r.add("DFM-FIN-FIT", "error",
              "heat_sink_fin_array: fin_thickness >= pitch (fins would overlap)");
    }
    // Footprint vs. workpiece base face: total span = N · thickness + (N-1) · gap.
    if (in.fin_count >= 2 && eff.pitch_mm > 0.0 && eff.fin_thickness_mm > 0.0) {
        const double span = in.fin_count * eff.fin_thickness_mm +
                           (in.fin_count - 1) * (eff.pitch_mm - eff.fin_thickness_mm);
        double xMin, yMin, zMin, xMax, yMax, zMax;
        wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
        const double maxDim = std::max(xMax - xMin, yMax - yMin);
        if (span > maxDim + 1e-3) {
            r.add("DFM-FIN-FIT", "error",
                  "heat_sink_fin_array: total span " + std::to_string(span) +
                  " mm > base max dim " + std::to_string(maxDim) + " mm");
        }
    }

    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "heat_sink_fin_array DFM failed:";
        for (const auto& f : dfm.findings) msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const FinSpec eff = resolveSpec(in);

    // Resolve base face — we need its TopZ for fin-bottom placement.
    auto baseId = wp.resolve(in.base_face);
    if (!baseId) throw SkillError("heat_sink_fin_array: base_face datum unresolved");

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    // The fin base sits ON the top of the workpiece (zMax).
    const double zFinBase = zMax;

    const double finLen = (in.fin_length_mm > 0.0) ? in.fin_length_mm : 30.0;

    // Local frame: airflow_dir = X-axis of fin extrusion.
    // pitchDir = perpendicular in XY plane (right-hand rule with +Z).
    const gp_Dir adir = in.airflow_dir;
    const gp_Dir pitchDir(-adir.Y(), adir.X(), 0.0);

    // Build N fin boxes, fused together into a single manifold, then fuse
    // onto the workpiece.
    std::vector<TopoDS_Shape> finBoxes;
    finBoxes.reserve(in.fin_count);
    for (int i = 0; i < in.fin_count; ++i) {
        // Each fin's left-front-bottom corner along pitchDir.
        const double pOffset = i * eff.pitch_mm;
        const gp_Pnt origin(
            in.origin_x_mm + pOffset * pitchDir.X(),
            in.origin_y_mm + pOffset * pitchDir.Y(),
            zFinBase);
        // gp_Ax2(P, V, Vx): V = LOCAL Z (height = fin_height), Vx = LOCAL X
        // = airflow_dir (length = fin_length).  Local Y = pitchDir
        // (thickness extrudes along pitch direction).
        const gp_Ax2 ax(origin, gp_Dir(0.0, 0.0, 1.0), adir);
        // box(ax, dx, dy, dz): dx along Vx (length), dy along Y (thickness),
        // dz along V (height).
        finBoxes.push_back(pr::box(ax, finLen, eff.fin_thickness_mm,
                                       eff.fin_height_mm));
    }

    // Fuse all fins into one manifold, then fuse onto workpiece (additive).
    TopoDS_Shape allFins = finBoxes.front();
    for (size_t i = 1; i < finBoxes.size(); ++i) {
        allFins = pr::fuse(allFins, finBoxes[i]);
    }
    const TopoDS_Shape newShape = pr::fuse(wp.shape(), allFins);

    // Tooling: extrusion (commercial) or end-mill from solid stock.
    const double finVolume = finLen * eff.fin_thickness_mm * eff.fin_height_mm;
    const double totalAddedMm3 = finVolume * in.fin_count;

    json params = {
        { "base_face_id",    *baseId },
        { "fin_count",       in.fin_count },
        { "spec_key",        in.spec_key },
        { "fin_thickness_mm", eff.fin_thickness_mm },
        { "fin_height_mm",    eff.fin_height_mm },
        { "fin_pitch_mm",     eff.pitch_mm },
        { "fin_length_mm",    finLen },
        { "airflow_dir",      { adir.X(), adir.Y(), adir.Z() } },
        { "origin_x_mm",      in.origin_x_mm },
        { "origin_y_mm",      in.origin_y_mm },
    };
    json pattern = {
        { "kind",              kSkillId },
        { "is_compound",       true },
        { "subfeature_count",  in.fin_count },
        { "spec_key",          in.spec_key },
        { "fin_thickness_mm",  eff.fin_thickness_mm },
        { "fin_height_mm",     eff.fin_height_mm },
        { "fin_pitch_mm",      eff.pitch_mm },
        { "fin_length_mm",     finLen },
        { "airflow_dir",       { adir.X(), adir.Y(), adir.Z() } },
        { "fin_aspect_ratio",  eff.fin_height_mm / std::max(1e-9, eff.fin_thickness_mm) },
    };
    ToolingMeta tooling;
    tooling.tool_type        = "extrusion;mill";
    tooling.tool_dia_mm      = eff.fin_thickness_mm;
    tooling.tool_length_mm   = eff.fin_height_mm + 5.0;
    tooling.tool_material    = "carbide";
    tooling.flute_count      = 2;
    tooling.cutting_speed_sfm = 500.0;
    tooling.feed_per_tooth_mm = 0.04;
    tooling.stock_removed_mm3 = -totalAddedMm3;   // additive (negative removal)
    tooling.est_cycle_time_s  = std::max(5.0, in.fin_count * 2.0);
    tooling.extra = {
        { "operation_note",
          "Either purchase as a finned extrusion (preferred when "
          "fin_count*pitch <= 200 mm), or mill from solid plate." },
        { "fin_count",      in.fin_count },
        { "added_volume_mm3", totalAddedMm3 },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);  // keep full chain history
    wpNew->addFeature(sig);

    spdlog::debug("skill::heat_sink_fin_array applied: spec={} count={} ({}×{}×{} mm)",
                  in.spec_key, in.fin_count,
                  eff.fin_thickness_mm, finLen, eff.fin_height_mm);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// Recognition strategy:
//   1. Metadata replay (feature history) — primary, exact (confidence 0.92).
//   2. Geometric back-map (secondary, lower confidence):
//      - Walk all planar faces; collect those whose normal is in the
//        XY plane (vertical walls — fin sides).  These come in parallel
//        pairs, one per fin (each fin contributes 2 wall faces).
//      - Group by their bbox-Z extent so we only count walls that share
//        the same fin height.
//      - From the wall-face bboxes, recover:
//          * fin_count        = (# distinct wall-X centers) / 2
//          * fin_thickness_mm = mean X-extent of a wall
//                               (actually mean distance between paired walls)
//          * fin_height_mm    = mean Z-extent
//          * fin_length_mm    = mean Y-extent
//          * fin_pitch_mm     = mean spacing between wall-pair centers
//      - Snap (pitch, thickness, height) tuple to nearest T-30/T-50/T-80/T-120
//        in the spec table; emit spec_key in recovered_params.

namespace {

struct WallBox {
    double x0, y0, z0, x1, y1, z1;
    double cx() const { return 0.5 * (x0 + x1); }
};

// Snap (pitch, thickness, height) tuple to closest spec.
const char* nearestSpec(double pitch_mm, double thk_mm, double hgt_mm,
                        double* out_err)
{
    constexpr std::array<SpecEntry, 4> table = kSpecTable;
    const char* best = nullptr;
    double bestErr = 4.0;  // generous since geometry is approximate
    for (const auto& s : table) {
        const double e = std::abs(pitch_mm - s.pitch_mm)
                       + std::abs(thk_mm   - s.fin_thickness_mm)
                       + std::abs(hgt_mm   - s.fin_height_mm);
        if (e < bestErr) { bestErr = e; best = s.key; }
    }
    if (out_err) *out_err = bestErr;
    return best;
}

}  // namespace

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;

    // Pass 1: metadata replay.
    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        json rec = {
            { "fin_count",        f.params.value("fin_count", 0) },
            { "spec_key",         f.params.value("spec_key",  std::string()) },
            { "spec_key_table",   f.params.value("spec_key",  std::string()) },
            { "fin_thickness_mm", f.params.value("fin_thickness_mm", 0.0) },
            { "fin_height_mm",    f.params.value("fin_height_mm",    0.0) },
            { "fin_pitch_mm",     f.params.value("fin_pitch_mm",     0.0) },
            { "fin_length_mm",    f.params.value("fin_length_mm",    0.0) },
            { "airflow_dir",      f.params.value("airflow_dir",
                                                  json::array({1.0,0.0,0.0})) },
        };
        out.push_back(RecognizedFeature{
            kSkillId, rec, 0.92, json{ { "source", "feature_history" } }
        });
    }
    if (!out.empty()) return out;

    // Pass 2: geometric back-map.  We assume airflow along +X (the apply
    // path's default), so fin walls are planar faces with normal ±Y.
    double wpXMin, wpYMin, wpZMin, wpXMax, wpYMax, wpZMax;
    wp.boundingBox(wpXMin, wpYMin, wpZMin, wpXMax, wpYMax, wpZMax);
    const double panelZ = wpZMax;  // approximate fin-base Z

    std::vector<WallBox> walls;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFacePlanar(i)) continue;
        gp_Dir n;
        try { n = wp.faceNormal(i); } catch (...) { continue; }
        // Wall normal: ±Y (in-plane, perpendicular to airflow X).
        if (std::abs(n.Y()) < 0.9 || std::abs(n.X()) > 0.1 ||
            std::abs(n.Z()) > 0.1) continue;
        Bnd_Box bb; BRepBndLib::Add(wp.face(i), bb);
        if (bb.IsVoid()) continue;
        WallBox w;
        bb.Get(w.x0, w.y0, w.z0, w.x1, w.y1, w.z1);
        // Skip the outer panel side faces (they touch wp bbox).
        const double m = 0.5;
        if (std::abs(w.y0 - wpYMin) < m && std::abs(w.y1 - wpYMin) < m) continue;
        if (std::abs(w.y0 - wpYMax) < m && std::abs(w.y1 - wpYMax) < m) continue;
        // Walls that rise ABOVE the panel top — fin sides.
        if (w.z1 < panelZ - 0.1) continue;
        if (w.z1 - w.z0 < 0.5) continue;  // ignore tiny slivers
        walls.push_back(w);
    }
    if (walls.size() < 4) return out;

    // Group walls by their Y center → pair them as left/right of each fin.
    std::sort(walls.begin(), walls.end(),
              [](const WallBox& a, const WallBox& b){
                  return 0.5 * (a.y0 + a.y1) < 0.5 * (b.y0 + b.y1);
              });
    // Pair consecutive walls; each pair encloses one fin in Y.
    std::vector<std::pair<double, double>> fins; // (yCenter, thickness)
    for (size_t i = 0; i + 1 < walls.size(); i += 2) {
        const double yA = 0.5 * (walls[i].y0   + walls[i].y1);
        const double yB = 0.5 * (walls[i+1].y0 + walls[i+1].y1);
        fins.push_back({ 0.5 * (yA + yB), std::abs(yB - yA) });
    }
    if (fins.size() < 2) return out;

    // Pitch = mean center-to-center distance.
    double pitchSum = 0.0;
    for (size_t i = 1; i < fins.size(); ++i)
        pitchSum += std::abs(fins[i].first - fins[i-1].first);
    const double pitch = pitchSum / static_cast<double>(fins.size() - 1);
    // Mean thickness.
    double thkSum = 0.0;
    for (const auto& f : fins) thkSum += f.second;
    const double thk = thkSum / static_cast<double>(fins.size());
    // Mean height = walls[].z1 - walls[].z0 of the tallest fin.
    double hgt = 0.0;
    for (const auto& w : walls) hgt = std::max(hgt, w.z1 - w.z0);
    // Mean length (along X) = walls[].x1 - walls[].x0.
    double lenSum = 0.0;
    for (const auto& w : walls) lenSum += (w.x1 - w.x0);
    const double finLen = lenSum / static_cast<double>(walls.size());

    double err = 0.0;
    const char* specKey = nearestSpec(pitch, thk, hgt, &err);
    if (!specKey) return out;

    // Origin = walls bbox min XY of the first fin.
    const double originX = walls.front().x0;
    const double originY = walls.front().y0;

    json rec = {
        { "fin_count",         static_cast<int>(fins.size()) },
        { "spec_key",          std::string(specKey) },
        { "spec_key_table",    std::string(specKey) },
        { "fin_thickness_mm",  thk },
        { "fin_height_mm",     hgt },
        { "fin_pitch_mm",      pitch },
        { "fin_length_mm",     finLen },
        { "airflow_dir",       json::array({ 1.0, 0.0, 0.0 }) },
        { "origin_x_mm",       originX },
        { "origin_y_mm",       originY },
        { "fit_match_err_mm",  err },
    };
    // Confidence model: base 0.50, +0.20 if err < 1.0 mm (tight spec match),
    //                          +0.10 if >= 3 fins detected (clear array).
    double conf = 0.50;
    if (err < 1.0) conf += 0.20;
    if (fins.size() >= 3u) conf += 0.10;
    out.push_back(RecognizedFeature{
        kSkillId, rec, conf,
        json{
            { "source",          "geometric_fin_walls" },
            { "wall_face_count", static_cast<int>(walls.size()) },
            { "fin_count",       static_cast<int>(fins.size()) },
            { "spec_key_table",  std::string(specKey) },
            { "fit_err_mm",      err },
        }
    });
    return out;
}

}  // namespace koocadcam::skill::heat_sink_fin_array
