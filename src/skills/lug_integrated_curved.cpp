// @lat: [[engine/skills#lug_integrated_curved]]

#include "lug_integrated_curved.hpp"

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
#include <array>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::skill::lug_integrated_curved {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

namespace {

constexpr std::array<double, 5> kSpringBarStandardDias { 1.2, 1.5, 1.6, 1.8, 2.0 };

bool isStandardSpringBar(double dia_mm, double tol_mm = 0.05) {
    for (double d : kSpringBarStandardDias)
        if (std::abs(d - dia_mm) < tol_mm) return true;
    return false;
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.case_outer_radius_mm <= 0.0 || in.lug_width_mm <= 0.0 ||
        in.lug_thickness_mm <= 0.0 || in.lug_length_mm <= 0.0 ||
        in.spring_bar_dia_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "lug_integrated_curved: all dimensions must be > 0");
        return r;
    }
    if (in.lug_count != 4 && in.lug_count != 6) {
        r.add("DFM-LUG-COUNT", "error",
              "lug_integrated_curved: lug_count must be 4 (2 pair) or 6 (3 pair), got " +
              std::to_string(in.lug_count));
    }
    if (in.lug_angle_deg < 60.0 || in.lug_angle_deg > 100.0) {
        r.add("DFM-LUG-ANGLE", "error",
              "lug_integrated_curved: lug_angle_deg " +
              std::to_string(in.lug_angle_deg) +
              " outside typical strap subtension [60, 100]");
    }
    if (in.lug_thickness_mm < 1.0) {
        r.add("DFM-LUG-THICK", "error",
              "lug_integrated_curved: lug_thickness " +
              std::to_string(in.lug_thickness_mm) + " mm < 1.0 mm");
    }
    if (!isStandardSpringBar(in.spring_bar_dia_mm)) {
        r.add("DFM-SPRINGBAR-SIZE", "error",
              "lug_integrated_curved: spring_bar_dia " +
              std::to_string(in.spring_bar_dia_mm) +
              " mm not in catalog (1.2/1.5/1.6/1.8/2.0)");
    }
    if (in.lug_length_mm < 4.0 * in.spring_bar_dia_mm) {
        r.add("DFM-LUG-LENGTH", "error",
              "lug_integrated_curved: lug_length " +
              std::to_string(in.lug_length_mm) +
              " mm < 4 × spring_bar_dia");
    }
    if (in.lug_width_mm < 2.0 * in.spring_bar_dia_mm) {
        r.add("DFM-LUG-WIDTH", "error",
              "lug_integrated_curved: lug_width " +
              std::to_string(in.lug_width_mm) +
              " mm < 2 × spring_bar_dia");
    }
    // Spring-bar z offset puts hole inside the lug AXIAL extent.
    if (std::abs(in.spring_bar_z_offset_mm) + in.spring_bar_dia_mm / 2.0 + 0.3
            > in.lug_width_mm / 2.0) {
        r.add("DFM-SPRINGBAR-Z", "error",
              "lug_integrated_curved: spring_bar_z_offset places hole within 0.3 mm of lug axial edge");
    }
    // Clearance > 0.3 mm requirement applied across radial / tangential walls.
    const double radialClear = in.lug_length_mm - in.spring_bar_dia_mm;
    if (radialClear < 0.3) {
        r.add("DFM-LUG-WALL", "error",
              "lug_integrated_curved: radial clearance around spring-bar hole < 0.3 mm");
    }
    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────
//
// For lug_count = 4: two pairs at 12 and 6 o'clock.
//   Pair-1 (top): angles = 90 + angle/2 and 90 - angle/2
//   Pair-2 (bot): angles = 270 + angle/2 and 270 - angle/2
// For lug_count = 6: three pairs at 12 / 4 / 8 o'clock (every 120°).
//
// Each lug is a curved (rect-prism) box fused onto the case, with a transverse
// spring-bar through-hole cut on the tangential axis.

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "lug_integrated_curved DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double caseTopZ = (in.lug_attach_top_z_mm != 0.0) ? in.lug_attach_top_z_mm : zMax;
    const double caseBotZ = (in.lug_attach_bot_z_mm != 0.0) ? in.lug_attach_bot_z_mm : zMin;
    const double caseCenterZ = (caseTopZ + caseBotZ) / 2.0;

    const gp_Dir axial = in.case_axis.Direction();

    // Compute base angles for each pair center, in degrees.
    std::vector<double> pairCenterDeg;
    if (in.lug_count == 4) {
        pairCenterDeg = { 90.0, 270.0 };
    } else {  // lug_count == 6
        pairCenterDeg = { 90.0, 210.0, 330.0 };
    }

    // Build per-lug angle list.
    std::vector<double> lugAnglesDeg;
    lugAnglesDeg.reserve(static_cast<size_t>(in.lug_count));
    for (double cDeg : pairCenterDeg) {
        lugAnglesDeg.push_back(cDeg - in.lug_angle_deg / 2.0);
        lugAnglesDeg.push_back(cDeg + in.lug_angle_deg / 2.0);
    }

    constexpr double kRadialBias = 0.05;
    TopoDS_Shape current = wp.shape();

    auto buildLugBox = [&](double angleDeg) {
        const double a = angleDeg * M_PI / 180.0;
        const gp_Dir radialOut(std::cos(a), std::sin(a), 0.0);
        const gp_Dir tangentCCW(-std::sin(a), std::cos(a), 0.0);

        const gp_Pnt anchor(
            in.case_axis.Location().X() + in.case_outer_radius_mm * radialOut.X(),
            in.case_axis.Location().Y() + in.case_outer_radius_mm * radialOut.Y(),
            caseCenterZ);

        // Box frame: V = radialOut (local Z), Vx = axial (local X).
        //   DX = lug_width  (axial)
        //   DY = lug_thickness (along Vx × V = axial × radialOut = tangentCCW)
        //   DZ = lug_length (radial outward)
        const gp_Pnt boxOrigin(
            anchor.X() - kRadialBias * radialOut.X()
                      - (in.lug_width_mm / 2.0) * axial.X()
                      - (in.lug_thickness_mm / 2.0) * tangentCCW.X(),
            anchor.Y() - kRadialBias * radialOut.Y()
                      - (in.lug_width_mm / 2.0) * axial.Y()
                      - (in.lug_thickness_mm / 2.0) * tangentCCW.Y(),
            anchor.Z() - kRadialBias * radialOut.Z()
                      - (in.lug_width_mm / 2.0) * axial.Z()
                      - (in.lug_thickness_mm / 2.0) * tangentCCW.Z());
        const gp_Ax2 boxAx(boxOrigin, radialOut, axial);
        return pr::box(
            boxAx,
            in.lug_width_mm,                              // DX axial
            in.lug_thickness_mm,                          // DY tangential
            in.lug_length_mm + kRadialBias);              // DZ radial
    };

    auto buildSpringBarHole = [&](double angleDeg) {
        const double a = angleDeg * M_PI / 180.0;
        const gp_Dir radialOut(std::cos(a), std::sin(a), 0.0);
        const gp_Dir tangentCCW(-std::sin(a), std::cos(a), 0.0);
        // Hole centre: midway radially through lug, offset axially by z_offset.
        const gp_Pnt anchor(
            in.case_axis.Location().X() + in.case_outer_radius_mm * radialOut.X(),
            in.case_axis.Location().Y() + in.case_outer_radius_mm * radialOut.Y(),
            caseCenterZ + in.spring_bar_z_offset_mm);
        const gp_Pnt holeCentre(
            anchor.X() + radialOut.X() * (in.lug_length_mm / 2.0),
            anchor.Y() + radialOut.Y() * (in.lug_length_mm / 2.0),
            anchor.Z());
        const double cylLen = in.lug_thickness_mm + 4.0;   // through + extra
        const gp_Pnt cylBase(
            holeCentre.X() - tangentCCW.X() * cylLen / 2.0,
            holeCentre.Y() - tangentCCW.Y() * cylLen / 2.0,
            holeCentre.Z() - tangentCCW.Z() * cylLen / 2.0);
        const gp_Ax2 holeAx(cylBase, tangentCCW);
        return pr::cylinder(holeAx, in.spring_bar_dia_mm / 2.0, cylLen);
    };

    // Pass 1: fuse all lugs.
    std::vector<TopoDS_Shape> lugBoxes;
    lugBoxes.reserve(lugAnglesDeg.size());
    for (double ang : lugAnglesDeg) lugBoxes.push_back(buildLugBox(ang));
    current = pr::fuseMany(current, lugBoxes);

    // Pass 2: cut all spring-bar holes.
    std::vector<TopoDS_Shape> holes;
    holes.reserve(lugAnglesDeg.size());
    for (double ang : lugAnglesDeg) holes.push_back(buildSpringBarHole(ang));
    const TopoDS_Shape newShape = pr::cutMany(current, holes);

    // ── Volumes ──────────────────────────────────────────────────────────
    const double perLugVol = in.lug_width_mm * in.lug_thickness_mm *
                             in.lug_length_mm;
    const double perHoleVol = M_PI *
        (in.spring_bar_dia_mm / 2.0) * (in.spring_bar_dia_mm / 2.0) *
        in.lug_thickness_mm;
    const double addedVol   = perLugVol * in.lug_count;
    const double removedVol = perHoleVol * in.lug_count;

    // ── Signature ────────────────────────────────────────────────────────
    json subList = json::array();
    for (size_t i = 0; i < lugAnglesDeg.size(); ++i) {
        subList.push_back({
            { "op", "lug_fuse_curved" },
            { "angle_deg", lugAnglesDeg[i] },
        });
    }
    for (size_t i = 0; i < lugAnglesDeg.size(); ++i) {
        subList.push_back({
            { "op", "spring_bar_hole" },
            { "angle_deg", lugAnglesDeg[i] },
            { "dia_mm", in.spring_bar_dia_mm },
        });
    }

    json params = {
        { "case_axis_loc",          { in.case_axis.Location().X(),
                                      in.case_axis.Location().Y(),
                                      in.case_axis.Location().Z() } },
        { "case_axis_dir",          { axial.X(), axial.Y(), axial.Z() } },
        { "case_outer_radius_mm",   in.case_outer_radius_mm },
        { "lug_count",              in.lug_count },
        { "lug_angle_deg",          in.lug_angle_deg },
        { "lug_width_mm",           in.lug_width_mm },
        { "lug_thickness_mm",       in.lug_thickness_mm },
        { "lug_length_mm",          in.lug_length_mm },
        { "spring_bar_dia_mm",      in.spring_bar_dia_mm },
        { "spring_bar_z_offset_mm", in.spring_bar_z_offset_mm },
    };
    json pattern = {
        { "kind",                kSkillId },
        { "is_compound",         true },
        { "is_watch_feature",    true },
        { "watch_feature_type",  "integrated_curved_lugs" },
        { "subfeature_count",    static_cast<int>(2 * in.lug_count) },
        { "subfeatures",         subList },
        { "lug_count",           in.lug_count },
        { "spring_bar_dia_mm",   in.spring_bar_dia_mm },
        { "derived_volume_added_mm3",   addedVol },
        { "derived_volume_removed_mm3", removedVol },
        { "lug_angles_deg",      lugAnglesDeg },
        { "axis_dir",            { axial.X(), axial.Y(), axial.Z() } },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "additive_form;drill";
    tooling.tool_dia_mm       = in.spring_bar_dia_mm;
    tooling.tool_length_mm    = in.lug_thickness_mm + 2.0;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 2;
    tooling.cutting_speed_sfm = 200.0;
    tooling.feed_per_tooth_mm = 0.03;
    tooling.stock_removed_mm3 = removedVol;
    tooling.est_cycle_time_s  = std::max(5.0, (addedVol + removedVol) / 80.0);
    tooling.extra = {
        { "feature_standard",
          "integrated welded curved lugs with Bergeon spring-bar holes" },
        { "added_volume_mm3",   addedVol },
        { "removed_volume_mm3", removedVol },
    };

    FeatureSignature sig { kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    wpNew->addFeature(sig);

    spdlog::debug(
        "skill::lug_integrated_curved applied: lugs={} angle={} bar={}",
        in.lug_count, in.lug_angle_deg, in.spring_bar_dia_mm);

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
            { "source",      "metadata_replay" },
            { "is_compound", true },
            { "lug_count",   f.pattern.value("lug_count", 4) },
        };
        out.push_back(r);
    }
    return out;
}

}  // namespace koocadcam::skill::lug_integrated_curved
