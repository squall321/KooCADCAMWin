// @lat: [[engine/skills#case_back_screw_down]]

#include "case_back_screw_down.hpp"

#include "_as568_table.hpp"
#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <gp_Ax2.hxx>
#include <gp_Pnt.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::skill::case_back_screw_down {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.back_dia_mm <= 0.0 || in.thread_pitch_mm <= 0.0 ||
        in.thread_depth_mm <= 0.0 || in.thread_band_axial_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "case_back_screw_down: all dimensions must be > 0");
        return r;
    }
    if (in.back_dia_mm < 12.0) {
        r.add("DFM-BACK-SIZE", "error",
              "case_back_screw_down: back_dia " +
              std::to_string(in.back_dia_mm) +
              " mm < 12 mm — sub-watch scale");
    }
    if (in.thread_pitch_mm < 0.4 || in.thread_pitch_mm > 1.5) {
        r.add("DFM-THREAD-PITCH", "error",
              "case_back_screw_down: thread_pitch " +
              std::to_string(in.thread_pitch_mm) +
              " mm outside watch-spec range [0.4, 1.5] mm");
    }
    if (in.thread_depth_mm < 0.3 || in.thread_depth_mm > 1.2) {
        r.add("DFM-THREAD-DEPTH", "error",
              "case_back_screw_down: thread_depth " +
              std::to_string(in.thread_depth_mm) +
              " mm outside [0.3, 1.2] mm");
    }
    // AS568 lookup MUST succeed.
    const auto* spec = as568::findDash(in.o_ring_size_key);
    if (!spec) {
        r.add("DFM-ORING-SIZE", "error",
              "case_back_screw_down: o_ring_size_key '" +
              in.o_ring_size_key +
              "' not in AS568 central table");
        return r;
    }
    // Position string.
    if (in.o_ring_groove_position != "flat" &&
        in.o_ring_groove_position != "external") {
        r.add("DFM-INPUT", "error",
              "case_back_screw_down: o_ring_groove_position must be 'flat' or 'external'");
        return r;
    }
    // Groove fits within caseback footprint.
    const double rBack = in.back_dia_mm / 2.0;
    if (in.o_ring_groove_position == "flat") {
        // Mean groove dia = caseback_dia − 2 × (thread_depth + 1.0)
        const double meanDia = in.back_dia_mm - 2.0 * (in.thread_depth_mm + 1.0);
        if (meanDia < 4.0) {
            r.add("DFM-ORING-FIT", "error",
                  "case_back_screw_down: flat-seal groove mean Ø " +
                  std::to_string(meanDia) +
                  " mm < 4 mm — caseback too small for thread + flat groove");
        }
        if (meanDia + spec->groove_width_mm > 2.0 * rBack - 1.0) {
            r.add("DFM-ORING-CLEAR", "error",
                  "case_back_screw_down: flat groove outer Ø clearance < 0.5 mm to caseback edge");
        }
    } else {
        // External / radial groove sits below thread band.
        if (spec->cross_section_mm + 0.3 > in.thread_band_axial_mm) {
            r.add("DFM-ORING-FIT", "error",
                  "case_back_screw_down: external groove CS + 0.3 mm clearance " +
                  std::to_string(spec->cross_section_mm + 0.3) +
                  " > thread_band_axial");
        }
    }

    if (!wp.shape().IsNull()) {
        double xMin, yMin, zMin, xMax, yMax, zMax;
        wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
        const double caseR = std::min(xMax - xMin, yMax - yMin) / 2.0;
        if (rBack + 0.3 > caseR) {
            r.add("DFM-BACK-FIT", "error",
                  "case_back_screw_down: back_dia clearance to case body < 0.3 mm");
        }
    }

    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────
//
// Sub-feature chain (FUSED then single cut):
//   1. thread band : shallow annular groove on the OUTER cylindrical wall of
//                    the caseback, axial length = thread_band_axial_mm, depth
//                    = thread_depth (cosmetic; the helix is metadata-only)
//   2. O-ring groove: AS568-sized annular groove EITHER flat-face (axial cut
//                    on the bottom) OR external-radial (cut into the OD wall
//                    below the thread band)
//   3. relief shoulder: small annular ring at the top of the thread band to
//                    give the O-ring a clean seat (0.3 mm × 0.2 mm deep)

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "case_back_screw_down DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const auto* spec = as568::findDash(in.o_ring_size_key);
    // validate() guarantees spec != nullptr.

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double bottomZ = (in.bottom_z_mm != 0.0) ? in.bottom_z_mm : zMin;
    constexpr double kOverhang = 0.05;

    const gp_Pnt axisLoc = in.case_axis.Location();
    const gp_Dir axisDir = in.case_axis.Direction();
    const gp_Dir downDir(-axisDir.X(), -axisDir.Y(), -axisDir.Z());
    const gp_Dir upDir   = axisDir;

    const double rBack = in.back_dia_mm / 2.0;

    // ── Sub-feature 1: thread band on OD ─────────────────────────────────
    // Cut as an annular ring with outer = rBack and inner = rBack - thread_depth,
    // sitting just above the caseback bottom (axially).  We cut UP from bottomZ
    // by thread_band_axial.
    const gp_Pnt threadStart(axisLoc.X(), axisLoc.Y(),
                             bottomZ - kOverhang);
    const gp_Ax2 threadAx(threadStart, upDir);
    const TopoDS_Shape threadBand = pr::annularRing(
        threadAx,
        rBack + kOverhang,                 // outer slightly larger to bite
        rBack - in.thread_depth_mm,        // inner
        in.thread_band_axial_mm + kOverhang);

    // ── Sub-feature 2: O-ring groove ─────────────────────────────────────
    TopoDS_Shape oRingGroove;
    if (in.o_ring_groove_position == "flat") {
        // Flat axial groove on the caseback bottom face.
        const double meanRadius = (in.back_dia_mm - 2.0 *
                                   (in.thread_depth_mm + 1.0)) / 2.0;
        const double rGrooveOuter = meanRadius + spec->groove_width_mm / 2.0;
        const double rGrooveInner = meanRadius - spec->groove_width_mm / 2.0;
        const gp_Pnt grooveStart(axisLoc.X(), axisLoc.Y(),
                                 bottomZ - kOverhang);
        const gp_Ax2 grooveAx(grooveStart, upDir);
        oRingGroove = pr::annularRing(
            grooveAx, rGrooveOuter, rGrooveInner,
            spec->groove_depth_mm + kOverhang);
    } else {
        // External radial groove on caseback OD, ABOVE thread band.
        const double zCenter = bottomZ + in.thread_band_axial_mm +
                               spec->cross_section_mm;
        const gp_Pnt grooveStart(axisLoc.X(), axisLoc.Y(),
                                 zCenter - spec->cross_section_mm / 2.0);
        const gp_Ax2 grooveAx(grooveStart, upDir);
        oRingGroove = pr::annularRing(
            grooveAx,
            rBack + kOverhang,                       // outer cuts past OD
            rBack - spec->groove_depth_mm,           // inner = groove floor
            spec->cross_section_mm);
    }

    // ── Sub-feature 3: relief shoulder at TOP of thread band ─────────────
    const double reliefZ = bottomZ + in.thread_band_axial_mm;
    const gp_Pnt reliefStart(axisLoc.X(), axisLoc.Y(), reliefZ);
    const gp_Ax2 reliefAx(reliefStart, upDir);
    const TopoDS_Shape relief = pr::annularRing(
        reliefAx,
        rBack + kOverhang,
        rBack - 0.3,
        0.2);

    // Fuse all 3 cutters into one.
    const TopoDS_Shape fuseA = pr::fuse(threadBand, oRingGroove);
    const TopoDS_Shape fuseB = pr::fuse(fuseA, relief);
    const TopoDS_Shape newShape = pr::cut(wp.shape(), fuseB);

    // ── Signature ────────────────────────────────────────────────────────
    json params = {
        { "case_axis_loc",          { axisLoc.X(), axisLoc.Y(), axisLoc.Z() } },
        { "case_axis_dir",          { axisDir.X(), axisDir.Y(), axisDir.Z() } },
        { "back_dia_mm",            in.back_dia_mm },
        { "thread_pitch_mm",        in.thread_pitch_mm },
        { "thread_depth_mm",        in.thread_depth_mm },
        { "thread_band_axial_mm",   in.thread_band_axial_mm },
        { "o_ring_groove_position", in.o_ring_groove_position },
        { "o_ring_size_key",        in.o_ring_size_key },
        { "bottom_z_mm",            bottomZ },
    };
    json pattern = {
        { "kind",                kSkillId },
        { "is_compound",         true },
        { "is_watch_feature",    true },
        { "watch_feature_type",  "screw_down_caseback" },
        { "subfeature_count",    3 },
        { "subfeatures", json::array({
            { { "op", "internal_thread_band_cosmetic" },
              { "pitch_mm",      in.thread_pitch_mm },
              { "depth_mm",      in.thread_depth_mm },
              { "axial_mm",      in.thread_band_axial_mm },
              { "thread_helix_scheduled_for", "thread_mill_internal" } },
            { { "op", "o_ring_groove_" + in.o_ring_groove_position },
              { "as568_dash",    in.o_ring_size_key },
              { "groove_depth_mm", spec->groove_depth_mm },
              { "groove_width_mm", spec->groove_width_mm },
              { "cs_mm",         spec->cross_section_mm } },
            { { "op", "thread_band_relief_shoulder" },
              { "depth_mm",      0.2 },
              { "width_mm",      0.3 } },
        }) },
        { "back_dia_mm",            in.back_dia_mm },
        { "o_ring_dash",            in.o_ring_size_key },
        { "o_ring_cross_section_mm", spec->cross_section_mm },
        { "thread_pitch_mm",        in.thread_pitch_mm },
        { "groove_position",        in.o_ring_groove_position },
        { "annular_face_count",     6 },   // thread + groove + relief = 3 × 2 walls
        { "axis_dir",               { axisDir.X(), axisDir.Y(), axisDir.Z() } },
    };

    const double rThreadInner = rBack - in.thread_depth_mm;
    const double threadVol = M_PI *
        (rBack * rBack - rThreadInner * rThreadInner) *
        in.thread_band_axial_mm;
    const double grooveVol = M_PI *
        (rBack * rBack -
         (rBack - spec->groove_depth_mm) * (rBack - spec->groove_depth_mm)) *
        spec->cross_section_mm;
    const double totalRemoved = threadVol + grooveVol;

    ToolingMeta tooling;
    tooling.tool_type         = "groove_tool;thread_mill_internal";
    tooling.tool_dia_mm       = in.back_dia_mm;
    tooling.tool_length_mm    = in.thread_band_axial_mm + 2.0;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 2;
    tooling.cutting_speed_sfm = 220.0;
    tooling.feed_per_tooth_mm = 0.03;
    tooling.stock_removed_mm3 = totalRemoved;
    tooling.est_cycle_time_s  = std::max(5.0, totalRemoved / 60.0);
    tooling.extra = {
        { "tool_sequence", {
            { { "tool_type", "groove_tool" },
              { "depth_mm", in.thread_depth_mm } },
            { { "tool_type", "thread_mill_internal" },
              { "pitch_mm", in.thread_pitch_mm } },
            { { "tool_type", "groove_tool" },
              { "depth_mm", spec->groove_depth_mm } },
        } },
        { "feature_standard",
          "ISO 22810 screw-down caseback + AS568 " + in.o_ring_size_key },
    };

    FeatureSignature sig { kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);  // keep full chain history
    wpNew->addFeature(sig);

    spdlog::debug(
        "skill::case_back_screw_down applied: D={} pitch={} dash={} pos={}",
        in.back_dia_mm, in.thread_pitch_mm, in.o_ring_size_key,
        in.o_ring_groove_position);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;
    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        RecognizedFeature r;
        r.skill_id         = kSkillId;
        r.recovered_params = f.params;
        r.confidence       = 1.0;
        r.matched_geometry = {
            { "source",          "metadata_replay" },
            { "is_compound",     true },
            { "groove_position", f.pattern.value("groove_position",
                                                 std::string("flat")) },
        };
        out.push_back(r);
    }
    return out;
}

}  // namespace koocadcam::skill::case_back_screw_down
