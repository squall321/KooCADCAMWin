// @lat: [[engine/skills#crown_guard_shoulder_compound]]

#include "crown_guard_shoulder_compound.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <gp_Ax1.hxx>
#include <gp_Ax2.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::skill::crown_guard_shoulder_compound {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.case_outer_radius_mm <= 0.0 || in.guard_height_mm <= 0.0 ||
        in.guard_width_mm <= 0.0 || in.guard_thickness_mm <= 0.0 ||
        in.crown_clearance_dia_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "crown_guard_shoulder_compound: all dimensions must be > 0");
        return r;
    }
    if (in.guard_thickness_mm < 0.5) {
        r.add("DFM-GUARD-THICK", "error",
              "crown_guard_shoulder_compound: guard_thickness " +
              std::to_string(in.guard_thickness_mm) +
              " mm < 0.5 mm — would yield under impact");
    }
    if (in.crown_clearance_dia_mm < 1.0) {
        r.add("DFM-CROWN-CLEAR", "error",
              "crown_guard_shoulder_compound: crown_clearance_dia " +
              std::to_string(in.crown_clearance_dia_mm) +
              " mm < 1.0 mm — crown stem won't fit");
    }
    // Boss must have material around hole.
    const double bossWallRadial = in.guard_thickness_mm -
                                  in.crown_clearance_dia_mm / 2.0;
    if (bossWallRadial < 0.3) {
        r.add("DFM-BOSS-WALL", "error",
              "crown_guard_shoulder_compound: boss wall around clearance hole " +
              std::to_string(bossWallRadial) +
              " mm < 0.3 mm");
    }
    const double bossWallTangential =
        (in.guard_width_mm - in.crown_clearance_dia_mm) / 2.0;
    if (bossWallTangential < 0.3) {
        r.add("DFM-BOSS-WALL", "error",
              "crown_guard_shoulder_compound: tangential boss wall " +
              std::to_string(bossWallTangential) +
              " mm < 0.3 mm");
    }
    if (in.guard_width_mm <= in.crown_clearance_dia_mm) {
        r.add("DFM-BOSS-WIDTH", "error",
              "crown_guard_shoulder_compound: guard_width must exceed crown_clearance_dia");
    }

    if (!wp.shape().IsNull()) {
        double xMin, yMin, zMin, xMax, yMax, zMax;
        wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
        const double caseHeight = zMax - zMin;
        if (in.guard_height_mm > 0.6 * caseHeight) {
            r.add("DFM-GUARD-HEIGHT", "warning",
                  "crown_guard_shoulder_compound: guard_height " +
                  std::to_string(in.guard_height_mm) +
                  " mm > 60% of case height");
        }
        const double bossCenterZ = (in.crown_z_mm > 0.0)
            ? in.crown_z_mm : (zMin + zMax) / 2.0;
        const double bossTopZ    = bossCenterZ + in.guard_height_mm / 2.0;
        const double bossBotZ    = bossCenterZ - in.guard_height_mm / 2.0;
        if (bossTopZ > zMax - 0.3 || bossBotZ < zMin + 0.3) {
            r.add("DFM-GUARD-AXIAL", "error",
                  "crown_guard_shoulder_compound: boss extends past case axial limits with < 0.3 mm clearance");
        }
    }

    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────
//
// Sub-feature chain:
//   1. FUSE a tangential boss box onto the case OD at guard_angle_deg
//      (sized guard_width × guard_height × guard_thickness, in tangential
//      × axial × radial directions respectively).
//   2. CUT a clearance hole through the boss along the inward-radial axis.

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "crown_guard_shoulder_compound DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const double a = in.guard_angle_deg * M_PI / 180.0;
    const gp_Dir radialOut(std::cos(a), std::sin(a), 0.0);
    const gp_Dir radialIn (-radialOut.X(), -radialOut.Y(), 0.0);
    const gp_Dir tangentCCW(-std::sin(a), std::cos(a), 0.0);
    const gp_Dir axial = in.case_axis.Direction();

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double bossCenterZ = (in.crown_z_mm != 0.0) ? in.crown_z_mm
                                                      : (zMin + zMax) / 2.0;

    // Anchor on case OD at boss vertical centre.
    const gp_Pnt anchor(
        in.case_axis.Location().X() + in.case_outer_radius_mm * radialOut.X(),
        in.case_axis.Location().Y() + in.case_outer_radius_mm * radialOut.Y(),
        bossCenterZ);

    // Bias boss slightly into the case OD so the fuse leaves no sliver.
    constexpr double kRadialBias = 0.05;

    // ── Sub-feature 1: boss box ──────────────────────────────────────────
    // Box frame: V (main = local Z) = axial,   Vx (local X) = radialOut.
    //   box(ax, dx, dy, dz):
    //     DX along Vx (radial),  DY along Vx × V = -tangentCCW, DZ along V (axial).
    //
    // We want:
    //   DX = guard_thickness + bias (radial outward, biased inward)
    //   DY = guard_width            (tangential)
    //   DZ = guard_height           (axial)
    const gp_Pnt boxOrigin(
        anchor.X() - kRadialBias * radialOut.X()
                  + (in.guard_width_mm / 2.0) * tangentCCW.X()
                  - (in.guard_height_mm / 2.0) * axial.X(),
        anchor.Y() - kRadialBias * radialOut.Y()
                  + (in.guard_width_mm / 2.0) * tangentCCW.Y()
                  - (in.guard_height_mm / 2.0) * axial.Y(),
        anchor.Z() - kRadialBias * radialOut.Z()
                  + (in.guard_width_mm / 2.0) * tangentCCW.Z()
                  - (in.guard_height_mm / 2.0) * axial.Z());
    const gp_Ax2 boxAx(boxOrigin, axial, radialOut);
    const TopoDS_Shape boss = pr::box(
        boxAx,
        in.guard_thickness_mm + kRadialBias,    // DX = radial
        in.guard_width_mm,                       // DY = tangential
        in.guard_height_mm);                     // DZ = axial

    const TopoDS_Shape caseWithBoss = pr::fuse(wp.shape(), boss);

    // ── Sub-feature 2: crown clearance hole ──────────────────────────────
    // Hole axis = radialIn (passes through boss INTO case).  Cylinder length
    // = guard_thickness + a small ingress into case body.
    constexpr double kCaseIngress = 1.0;
    const double cylLen = in.guard_thickness_mm + kCaseIngress + 0.2;

    // Cylinder starts at boss outer face (case_outer_radius + guard_thickness)
    // and goes INWARD by cylLen.
    const gp_Pnt holeStart(
        anchor.X() + (in.guard_thickness_mm + 0.1) * radialOut.X(),
        anchor.Y() + (in.guard_thickness_mm + 0.1) * radialOut.Y(),
        bossCenterZ);
    const gp_Ax2 holeAx(holeStart, radialIn);
    const TopoDS_Shape clearance = pr::cylinder(
        holeAx, in.crown_clearance_dia_mm / 2.0, cylLen);

    const TopoDS_Shape newShape = pr::cut(caseWithBoss, clearance);

    // ── Volumes ──────────────────────────────────────────────────────────
    const double bossVol = in.guard_thickness_mm * in.guard_width_mm *
                           in.guard_height_mm;
    const double holeVol = M_PI *
        (in.crown_clearance_dia_mm / 2.0) * (in.crown_clearance_dia_mm / 2.0) *
        cylLen;

    // ── Signature ────────────────────────────────────────────────────────
    json params = {
        { "case_axis_loc",         { in.case_axis.Location().X(),
                                     in.case_axis.Location().Y(),
                                     in.case_axis.Location().Z() } },
        { "case_axis_dir",         { axial.X(), axial.Y(), axial.Z() } },
        { "case_outer_radius_mm",  in.case_outer_radius_mm },
        { "guard_angle_deg",       in.guard_angle_deg },
        { "guard_height_mm",       in.guard_height_mm },
        { "guard_width_mm",        in.guard_width_mm },
        { "guard_thickness_mm",    in.guard_thickness_mm },
        { "crown_clearance_dia_mm", in.crown_clearance_dia_mm },
        { "crown_z_mm",            bossCenterZ },
    };
    json pattern = {
        { "kind",                kSkillId },
        { "is_compound",         true },
        { "is_watch_feature",    true },
        { "watch_feature_type",  "crown_shoulder_with_clearance" },
        { "subfeature_count",    2 },
        { "subfeatures", json::array({
            { { "op", "boss_fuse_radial" },
              { "thickness_mm", in.guard_thickness_mm },
              { "width_mm",     in.guard_width_mm },
              { "height_mm",    in.guard_height_mm },
              { "angle_deg",    in.guard_angle_deg },
              { "added_vol_mm3", bossVol } },
            { { "op", "crown_clearance_through_boss" },
              { "dia_mm",       in.crown_clearance_dia_mm },
              { "length_mm",    cylLen },
              { "removed_vol_mm3", holeVol } },
        }) },
        { "guard_angle_deg",       in.guard_angle_deg },
        { "guard_thickness_mm",    in.guard_thickness_mm },
        { "crown_clearance_dia_mm", in.crown_clearance_dia_mm },
        { "derived_volume_added_mm3", bossVol },
        { "derived_volume_removed_mm3", holeVol },
        { "boss_with_through_hole", true },
        { "radial_dir",            { radialOut.X(), radialOut.Y(), radialOut.Z() } },
        { "axis_dir",              { axial.X(), axial.Y(), axial.Z() } },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "additive_form;drill";
    tooling.tool_dia_mm       = in.crown_clearance_dia_mm;
    tooling.tool_length_mm    = cylLen + 2.0;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 2;
    tooling.cutting_speed_sfm = 200.0;
    tooling.feed_per_tooth_mm = 0.03;
    tooling.stock_removed_mm3 = holeVol;
    tooling.est_cycle_time_s  = std::max(3.0, (bossVol + holeVol) / 100.0);
    tooling.extra = {
        { "feature_standard",
          "Tudor Pelagos / Seiko-style crown shoulder with clearance" },
        { "added_volume_mm3",   bossVol },
        { "removed_volume_mm3", holeVol },
    };

    FeatureSignature sig { kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    wpNew->addFeature(sig);

    spdlog::debug(
        "skill::crown_guard_shoulder_compound applied: angle={} T={} clearD={}",
        in.guard_angle_deg, in.guard_thickness_mm, in.crown_clearance_dia_mm);

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
            { "source",                 "metadata_replay" },
            { "is_compound",            true },
            { "boss_with_through_hole", true },
        };
        out.push_back(r);
    }
    return out;
}

}  // namespace koocadcam::skill::crown_guard_shoulder_compound
