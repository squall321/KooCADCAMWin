// @lat: [[engine/skills#needle_valve_seat]]

#include "needle_valve_seat.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepAdaptor_Surface.hxx>
#include <gp.hxx>
#include <gp_Ax2.hxx>
#include <gp_Cone.hxx>
#include <gp_Cylinder.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::skill::needle_valve_seat {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.pocket_entry_dia_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "needle_valve_seat: pocket_entry_dia must be > 0");
    }
    if (in.pocket_length_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "needle_valve_seat: pocket length must be > 0");
    }
    // ISA 75.11 Annex (small-Cv valves) — needle bore 0.2..1.0 mm.
    if (in.control_bore_dia_mm < 0.2 || in.control_bore_dia_mm > 1.0) {
        r.add("DFM-NEEDLE-BORE-DIA", "error",
              "needle_valve_seat: control_bore_dia " +
              std::to_string(in.control_bore_dia_mm) +
              " mm outside [0.2, 1.0] mm (ISA 75.11 needle-class spec)");
    }
    // Industry-norm taper window 20..45°.
    if (in.pocket_taper_deg < 20.0 || in.pocket_taper_deg > 45.0) {
        r.add("DFM-NEEDLE-TAPER", "error",
              "needle_valve_seat: pocket taper " +
              std::to_string(in.pocket_taper_deg) +
              "° outside [20°, 45°] (industry norm — fine vs coarse throttling)");
    }
    // Stem boss must clear pocket entry by 1.4× (so the stem nut fits).
    if (in.stem_boss_dia_mm < 1.4 * in.pocket_entry_dia_mm) {
        r.add("DFM-NEEDLE-BOSS", "error",
              "needle_valve_seat: stem boss dia " +
              std::to_string(in.stem_boss_dia_mm) +
              " mm < 1.4 × pocket entry — stem nut won't clear");
    }
    // Control bore must be smaller than pocket entry (else the pocket
    // doesn't terminate in a needle seat).
    if (in.control_bore_dia_mm >= in.pocket_entry_dia_mm) {
        r.add("DFM-NEEDLE-BORE-VS-POCKET", "error",
              "needle_valve_seat: control_bore_dia ≥ pocket_entry_dia "
              "(no needle seat exists)");
    }
    if (in.thread_relief_dia_mm <= in.pocket_entry_dia_mm ||
        in.thread_relief_dia_mm >= in.stem_boss_dia_mm) {
        r.add("DFM-NEEDLE-RELIEF", "warning",
              "needle_valve_seat: relief dia should sit strictly between "
              "pocket entry and stem boss for cleanest tap exit");
    }
    if (in.control_bore_depth_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "needle_valve_seat: control_bore_depth must be > 0");
    }
    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "needle_valve_seat DFM failed:";
        for (const auto& f : dfm.findings) msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    if (std::abs(in.axis_dir.X()) > 1e-6 ||
        std::abs(in.axis_dir.Y()) > 1e-6 ||
        in.axis_dir.Z() >= 0.0) {
        throw SkillError("needle_valve_seat: slice-1 supports axis_dir=-Z only");
    }

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double kOverhang = 0.05;

    // Top entry plane at zMax.  The skill cuts down (−Z).
    //
    // Layer stack from top:
    //   0..stem_boss_depth        → stem boss counterbore (wide)
    //   stem_boss_depth..(+relief)→ thread relief
    //   below that                → tapered needle pocket
    //   below pocket tip          → control bore (tiny)

    const double zTop = zMax + kOverhang;

    // ── Sub-feature 1: Tapered needle pocket (cone frustum) ─────────────
    //
    // Cone has wide top = pocket_entry_dia, narrow bottom = control_bore_dia,
    // axial length = pocket_length, axis along -Z.  Built starting from the
    // entry depth (boss + relief) downward.
    //
    // pocket starts at z = zMax - (stem_boss_depth + thread_relief_depth)
    const double pocketTopZ = zMax - (in.stem_boss_depth_mm + in.thread_relief_depth_mm);
    const gp_Ax2 pocketAx(
        gp_Pnt(in.position_x_mm, in.position_y_mm, pocketTopZ),
        gp_Dir(0.0, 0.0, -1.0));
    // Note: cone_frustum r1 (at axis origin/top) → r2 (axial length below).
    const TopoDS_Shape needlePocket = pr::coneFrustum(
        pocketAx,
        in.pocket_entry_dia_mm / 2.0,
        in.control_bore_dia_mm / 2.0,
        in.pocket_length_mm);

    // ── Sub-feature 2: Control bore (precision orifice) ─────────────────
    const double pocketBottomZ = pocketTopZ - in.pocket_length_mm;
    const gp_Ax2 controlAx(
        gp_Pnt(in.position_x_mm, in.position_y_mm, pocketBottomZ + kOverhang),
        gp_Dir(0.0, 0.0, -1.0));
    const TopoDS_Shape controlBore = pr::cylinder(
        controlAx, in.control_bore_dia_mm / 2.0,
        in.control_bore_depth_mm + kOverhang);

    // ── Sub-feature 3: Stem boss counterbore ────────────────────────────
    const gp_Ax2 bossAx(
        gp_Pnt(in.position_x_mm, in.position_y_mm, zTop),
        gp_Dir(0.0, 0.0, -1.0));
    const TopoDS_Shape stemBoss = pr::cylinder(
        bossAx, in.stem_boss_dia_mm / 2.0, in.stem_boss_depth_mm + kOverhang);

    // ── Sub-feature 4: Thread relief (between boss and pocket) ──────────
    const gp_Ax2 reliefAx(
        gp_Pnt(in.position_x_mm, in.position_y_mm,
               zMax - in.stem_boss_depth_mm + kOverhang),
        gp_Dir(0.0, 0.0, -1.0));
    const TopoDS_Shape threadRelief = pr::cylinder(
        reliefAx, in.thread_relief_dia_mm / 2.0,
        in.thread_relief_depth_mm + kOverhang);

    // ── Fuse concentric overlapping tools, then a single cut ────────────
    const TopoDS_Shape coneAndBore   = pr::fuse(needlePocket, controlBore);
    const TopoDS_Shape withRelief    = pr::fuse(coneAndBore, threadRelief);
    const TopoDS_Shape allCutters    = pr::fuse(withRelief, stemBoss);
    const TopoDS_Shape newShape      = pr::cut(wp.shape(), allCutters);

    // ── Signature ───────────────────────────────────────────────────────
    json params = {
        { "position_x_mm",         in.position_x_mm },
        { "position_y_mm",         in.position_y_mm },
        { "axis_dir",              { in.axis_dir.X(), in.axis_dir.Y(), in.axis_dir.Z() } },
        { "pocket_entry_dia_mm",   in.pocket_entry_dia_mm },
        { "pocket_taper_deg",      in.pocket_taper_deg },
        { "pocket_length_mm",      in.pocket_length_mm },
        { "control_bore_dia_mm",   in.control_bore_dia_mm },
        { "control_bore_depth_mm", in.control_bore_depth_mm },
        { "stem_boss_dia_mm",      in.stem_boss_dia_mm },
        { "stem_boss_depth_mm",    in.stem_boss_depth_mm },
        { "thread_relief_dia_mm",  in.thread_relief_dia_mm },
        { "thread_relief_depth_mm",in.thread_relief_depth_mm },
    };
    json pattern = {
        { "kind",                  kSkillId },
        { "is_compound",           true },
        { "subfeature_count",      4 },
        { "subfeatures",           { "needle_pocket", "control_bore",
                                     "stem_boss", "thread_relief" } },
        { "conical_face_count",    1 },
        { "cylindrical_face_count",3 },   // bore + boss + relief
        { "pocket_taper_deg",      in.pocket_taper_deg },
        { "control_bore_dia_mm",   in.control_bore_dia_mm },
        { "is_precision_orifice",  in.control_bore_dia_mm < 1.0 },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "form_cutter;micro_drill;tap";
    tooling.tool_dia_mm       = in.stem_boss_dia_mm;
    tooling.tool_length_mm    = in.stem_boss_depth_mm + in.pocket_length_mm +
                                in.control_bore_depth_mm + 5.0;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 2;
    tooling.cutting_speed_sfm = 250.0;
    tooling.feed_per_tooth_mm = 0.02;
    const double coneV = M_PI * in.pocket_length_mm / 3.0 *
                         (std::pow(in.pocket_entry_dia_mm / 2.0, 2) +
                          (in.pocket_entry_dia_mm / 2.0) * (in.control_bore_dia_mm / 2.0) +
                          std::pow(in.control_bore_dia_mm / 2.0, 2));
    const double boreV = M_PI * std::pow(in.control_bore_dia_mm / 2.0, 2)
                              * in.control_bore_depth_mm;
    const double bossV = M_PI * std::pow(in.stem_boss_dia_mm / 2.0, 2)
                              * in.stem_boss_depth_mm;
    const double reliefV = M_PI * std::pow(in.thread_relief_dia_mm / 2.0, 2)
                                * in.thread_relief_depth_mm;
    tooling.stock_removed_mm3 = coneV + boreV + bossV + reliefV;
    tooling.est_cycle_time_s  = std::max(5.0, tooling.stock_removed_mm3 / 800.0);
    tooling.extra = {
        { "tool_sequence", {
            { { "tool_type", "end_mill" },   { "tool_dia_mm", in.stem_boss_dia_mm },
              { "note", "stem boss counterbore" } },
            { { "tool_type", "end_mill" },   { "tool_dia_mm", in.thread_relief_dia_mm },
              { "note", "thread relief" } },
            { { "tool_type", "form_cutter" },{ "angle_deg", in.pocket_taper_deg },
              { "note", "tapered needle pocket" } },
            { { "tool_type", "micro_drill" },{ "tool_dia_mm", in.control_bore_dia_mm },
              { "note", "precision control bore" } },
        } },
        { "standard", "ISA 75.11 small-Cv (needle class)" },
    };

    FeatureSignature sig { kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);  // keep full chain history
    wpNew->addFeature(sig);

    spdlog::debug(
        "skill::needle_valve_seat applied: taper={}° pocket={} bore={} faces {}→{}",
        in.pocket_taper_deg, in.pocket_entry_dia_mm, in.control_bore_dia_mm,
        wp.faceCount(), wpNew->faceCount());

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;

    // Metadata replay
    for (const auto& f : wp.features()) {
        if (f.skill_id == kSkillId) {
            RecognizedFeature r;
            r.skill_id         = kSkillId;
            r.recovered_params = f.params;
            r.confidence       = 1.0;
            r.matched_geometry = { { "source", "metadata_replay" } };
            out.push_back(r);
        }
    }
    if (!out.empty()) return out;

    // Geometric: cone face (axis along Z) + small Z-axis cylinder (orifice) +
    // larger Z-axis cylinder (stem boss).
    int coneIdx = -1;
    double smallestCylR = 1e9;
    int smallestCylIdx = -1;
    double largestCylR = 0.0;
    int largestCylIdx = -1;
    for (int i = 0; i < wp.faceCount(); ++i) {
        BRepAdaptor_Surface s(wp.face(i));
        if (s.GetType() == GeomAbs_Cone) {
            const gp_Cone c = s.Cone();
            if (std::abs(std::abs(c.Axis().Direction().Z()) - 1.0) < 1e-2) {
                coneIdx = i;
            }
        } else if (s.GetType() == GeomAbs_Cylinder) {
            const gp_Cylinder cyl = s.Cylinder();
            if (std::abs(std::abs(cyl.Axis().Direction().Z()) - 1.0) < 1e-2) {
                const double r = cyl.Radius();
                if (r < smallestCylR) { smallestCylR = r; smallestCylIdx = i; }
                if (r > largestCylR)  { largestCylR  = r; largestCylIdx  = i; }
            }
        }
    }
    if (coneIdx < 0 || smallestCylIdx < 0 || largestCylIdx < 0) return out;
    if (smallestCylIdx == largestCylIdx) return out;

    json recovered = {
        { "control_bore_dia_mm",  2.0 * smallestCylR },
        { "stem_boss_dia_mm",     2.0 * largestCylR  },
        { "pocket_taper_deg",     30.0 },               // best-guess
        { "axis_dir",             { 0.0, 0.0, -1.0 } },
    };
    json matched = {
        { "cone_face_id",        coneIdx },
        { "control_bore_face_id",smallestCylIdx },
        { "stem_boss_face_id",   largestCylIdx },
        { "source",              "geometric" },
    };
    out.push_back(RecognizedFeature{ kSkillId, recovered, 0.7, matched });
    return out;
}

}  // namespace koocadcam::skill::needle_valve_seat
