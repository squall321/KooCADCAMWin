// @lat: [[engine/skills#heat_sink_fin_array]]

#include "heat_sink_fin_array.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

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
    wpNew->addFeature(sig);

    spdlog::debug("skill::heat_sink_fin_array applied: spec={} count={} ({}×{}×{} mm)",
                  in.spec_key, in.fin_count,
                  eff.fin_thickness_mm, finLen, eff.fin_height_mm);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// Compound additive features are hard to identify from geometry alone in
// general: the recognizer falls back to FeatureSignature replay (the
// workpiece feature history).  When metadata is absent (e.g. after a STEP
// round-trip), no candidates are returned.

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;
    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        json rec = {
            { "fin_count",        f.params.value("fin_count", 0) },
            { "spec_key",         f.params.value("spec_key",  std::string()) },
            { "fin_thickness_mm", f.params.value("fin_thickness_mm", 0.0) },
            { "fin_height_mm",    f.params.value("fin_height_mm",    0.0) },
            { "fin_pitch_mm",     f.params.value("fin_pitch_mm",     0.0) },
            { "fin_length_mm",    f.params.value("fin_length_mm",    0.0) },
            { "airflow_dir",      f.params.value("airflow_dir", json::array({1.0,0.0,0.0})) },
        };
        out.push_back(RecognizedFeature{
            kSkillId, rec, 0.92, json{ { "source", "feature_history" } }
        });
    }
    return out;
}

}  // namespace koocadcam::skill::heat_sink_fin_array
