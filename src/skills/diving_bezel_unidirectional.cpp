// @lat: [[engine/skills#diving_bezel_unidirectional]]

#include "diving_bezel_unidirectional.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepBuilderAPI_Transform.hxx>
#include <gp_Ax1.hxx>
#include <gp_Ax2.hxx>
#include <gp_Pnt.hxx>
#include <gp_Trsf.hxx>

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

namespace koocadcam::skill::diving_bezel_unidirectional {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    (void)wp;
    DFMReport r;

    if (in.bezel_outer_dia_mm <= 0.0 || in.bezel_inner_dia_mm <= 0.0 ||
        in.bezel_height_mm <= 0.0 || in.click_depth_mm <= 0.0 ||
        in.click_notch_dia_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "diving_bezel_unidirectional: all dimensions must be > 0");
        return r;
    }
    if (in.bezel_inner_dia_mm >= in.bezel_outer_dia_mm) {
        r.add("DFM-BEZEL-GEOM", "error",
              "diving_bezel_unidirectional: bezel_outer_dia (" +
              std::to_string(in.bezel_outer_dia_mm) +
              ") must exceed bezel_inner_dia (" +
              std::to_string(in.bezel_inner_dia_mm) + ")");
        return r;
    }
    const double radialExtent =
        (in.bezel_outer_dia_mm - in.bezel_inner_dia_mm) / 2.0;
    if (radialExtent < 1.5) {
        r.add("DFM-BEZEL-WIDTH", "error",
              "diving_bezel_unidirectional: seat radial extent " +
              std::to_string(radialExtent) +
              " mm < 1.5 mm — unmachinable seat");
    }
    if (in.click_count != 60 && in.click_count != 120) {
        r.add("DFM-CLICK-COUNT", "error",
              "diving_bezel_unidirectional: click_count must be 60 (GMT) or 120 (ISO 6425), got " +
              std::to_string(in.click_count));
    }
    if (in.click_depth_mm >= in.bezel_height_mm) {
        r.add("DFM-CLICK-DEPTH", "error",
              "diving_bezel_unidirectional: click_depth (" +
              std::to_string(in.click_depth_mm) +
              ") >= bezel_height — notches punch through seat floor");
    }
    // Clearance > 0.3 mm between click notches (chord distance at seat OD).
    const double seatOdR = in.bezel_outer_dia_mm / 2.0;
    const double stepRad = 2.0 * M_PI / in.click_count;
    const double notchChord = 2.0 * seatOdR * std::sin(stepRad / 2.0);
    const double notchClearance = notchChord - in.click_notch_dia_mm;
    if (notchClearance < 0.3) {
        r.add("DFM-CLICK-CLEARANCE", "error",
              "diving_bezel_unidirectional: notch clearance " +
              std::to_string(notchClearance) +
              " mm < 0.3 mm — adjacent notches collide");
    }
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "diving_bezel_unidirectional DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const double rOuter = in.bezel_outer_dia_mm / 2.0;
    const double rInner = in.bezel_inner_dia_mm / 2.0;

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double topZ = (in.top_z_mm > 0.0) ? in.top_z_mm : zMax;
    constexpr double kOverhang = 0.05;

    const gp_Pnt axisLoc = in.case_axis.Location();
    const gp_Dir axisDir = in.case_axis.Direction();

    // Bezel seat ring (cut DOWN from topZ).
    const gp_Pnt seatStart(axisLoc.X(), axisLoc.Y(), topZ + kOverhang);
    const gp_Ax2 seatAx(seatStart,
                        gp_Dir(-axisDir.X(), -axisDir.Y(), -axisDir.Z()));
    const TopoDS_Shape seatRing = pr::annularRing(
        seatAx, rOuter, rInner, in.bezel_height_mm + kOverhang);

    // Ratchet notches sit at the OUTER edge of the seat (radius = rOuter).
    // Each is a small cylinder cut going DOWN to click_depth, biased slightly
    // inward by notch_dia/2 so the notch sits on the seat OD ring itself.
    const double rNotchCenter = rOuter - in.click_notch_dia_mm / 2.0;
    const gp_Pnt notchSeedStart(
        axisLoc.X() + rNotchCenter,
        axisLoc.Y(),
        topZ + kOverhang);
    const gp_Ax2 notchSeedAx(notchSeedStart,
                             gp_Dir(-axisDir.X(), -axisDir.Y(), -axisDir.Z()));
    const TopoDS_Shape notchSeed = pr::cylinder(
        notchSeedAx, in.click_notch_dia_mm / 2.0,
        in.click_depth_mm + kOverhang);

    const gp_Ax1 rotAxis(axisLoc, axisDir);
    std::vector<TopoDS_Shape> cutters;
    cutters.reserve(static_cast<size_t>(in.click_count) + 1);
    cutters.push_back(seatRing);
    const double stepDeg = 360.0 / in.click_count;
    for (int i = 0; i < in.click_count; ++i) {
        const double aRad = i * stepDeg * M_PI / 180.0;
        gp_Trsf rot;
        rot.SetRotation(rotAxis, aRad);
        BRepBuilderAPI_Transform xform(notchSeed, rot, true);
        if (!xform.IsDone())
            throw SkillError("diving_bezel_unidirectional: notch transform failed");
        cutters.push_back(xform.Shape());
    }

    const TopoDS_Shape newShape = pr::cutMany(wp.shape(), cutters);

    // Volumes for signature / tooling.
    const double seatVol =
        M_PI * (rOuter * rOuter - rInner * rInner) * in.bezel_height_mm;
    const double notchVol =
        M_PI * (in.click_notch_dia_mm / 2.0) * (in.click_notch_dia_mm / 2.0)
             * in.click_depth_mm * in.click_count;
    const double totalRemoved = seatVol + notchVol;

    // ── Signature ────────────────────────────────────────────────────────
    json params = {
        { "case_axis_loc",       { axisLoc.X(), axisLoc.Y(), axisLoc.Z() } },
        { "case_axis_dir",       { axisDir.X(), axisDir.Y(), axisDir.Z() } },
        { "bezel_outer_dia_mm",  in.bezel_outer_dia_mm },
        { "bezel_inner_dia_mm",  in.bezel_inner_dia_mm },
        { "bezel_height_mm",     in.bezel_height_mm },
        { "click_count",         in.click_count },
        { "click_depth_mm",      in.click_depth_mm },
        { "click_notch_dia_mm",  in.click_notch_dia_mm },
        { "top_z_mm",            topZ },
    };
    json pattern = {
        { "kind",                kSkillId },
        { "is_compound",         true },
        { "is_watch_feature",    true },
        { "watch_feature_type",  "diving_bezel_seat_with_ratchet" },
        { "subfeature_count",    2 },
        { "subfeatures", json::array({
            { { "op", "annular_bezel_seat" },
              { "outer_dia_mm", in.bezel_outer_dia_mm },
              { "inner_dia_mm", in.bezel_inner_dia_mm },
              { "depth_mm",     in.bezel_height_mm } },
            { { "op", "ratchet_click_polar_pattern" },
              { "instance_count", in.click_count },
              { "step_deg",       360.0 / in.click_count },
              { "notch_dia_mm",   in.click_notch_dia_mm },
              { "notch_depth_mm", in.click_depth_mm },
              { "ring_radius_mm", rNotchCenter } },
        }) },
        { "bezel_outer_dia_mm",  in.bezel_outer_dia_mm },
        { "bezel_inner_dia_mm",  in.bezel_inner_dia_mm },
        { "annular_width_mm",    (in.bezel_outer_dia_mm - in.bezel_inner_dia_mm) / 2.0 },
        { "click_count",         in.click_count },
        { "click_step_deg",      360.0 / in.click_count },
        { "derived_volume_removed_mm3", totalRemoved },
        { "axis_dir",            { axisDir.X(), axisDir.Y(), axisDir.Z() } },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "annular_groove;drill";
    tooling.tool_dia_mm       = in.bezel_outer_dia_mm;
    tooling.tool_length_mm    = in.bezel_height_mm + 3.0;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 2;
    tooling.cutting_speed_sfm = 240.0;
    tooling.feed_per_tooth_mm = 0.04;
    tooling.stock_removed_mm3 = totalRemoved;
    tooling.est_cycle_time_s  = std::max(5.0, totalRemoved / 80.0);
    tooling.extra = {
        { "feature_standard",
          (in.click_count == 60 ? std::string("GMT-style 60-click bezel")
                                : std::string("ISO 6425 dive 120-click bezel")) },
    };

    FeatureSignature sig { kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    wpNew->addFeature(sig);

    spdlog::debug(
        "skill::diving_bezel_unidirectional applied: outerD={} innerD={} clicks={}",
        in.bezel_outer_dia_mm, in.bezel_inner_dia_mm, in.click_count);

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
            { "source",       "metadata_replay" },
            { "is_compound",  true },
            { "click_count",  f.pattern.value("click_count", 120) },
        };
        out.push_back(r);
    }
    return out;
}

}  // namespace koocadcam::skill::diving_bezel_unidirectional
