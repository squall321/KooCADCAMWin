// @lat: [[engine/skills#crown_guard_compound]]

#include "crown_guard_compound.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <gp_Ax1.hxx>
#include <gp_Ax2.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>

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

namespace koocadcam::skill::crown_guard_compound {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    (void)wp;
    DFMReport r;
    if (in.guard_height_mm <= 0.0 || in.guard_thickness_mm <= 0.0 ||
        in.guard_radial_extent_mm <= 0.0 || in.guard_separation_mm <= 0.0 ||
        in.case_outer_radius_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "crown_guard_compound: all dimensions must be > 0");
        return r;
    }
    if (in.guard_thickness_mm < 0.4) {
        r.add("DFM-GUARD-THICK", "error",
              "crown_guard_compound: guard_thickness " +
              std::to_string(in.guard_thickness_mm) +
              " mm < 0.4 mm (would yield under impact)");
    }
    if (in.guard_separation_mm < 2.0 * in.guard_thickness_mm) {
        r.add("DFM-GUARD-SEP", "error",
              "crown_guard_compound: guard_separation (" +
              std::to_string(in.guard_separation_mm) +
              ") < 2 × guard_thickness — guards would touch");
    }
    if (in.guard_chamfer_mm > std::min(in.guard_height_mm,
                                       in.guard_thickness_mm) / 2.0) {
        r.add("DFM-GUARD-CHAMFER", "warning",
              "crown_guard_compound: chamfer too large for guard section");
    }
    if (in.guard_radial_extent_mm > in.case_outer_radius_mm * 0.5) {
        r.add("DFM-GUARD-EXTENT", "warning",
              "crown_guard_compound: guard protrudes more than case_outer_radius/2");
    }
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────
//
// Sub-feature chain (2 logical fuses + chamfer metadata):
//   1. fuse upper guard box  (offset +guard_separation/2 along case axis)
//   2. fuse lower guard box  (offset -guard_separation/2 along case axis)
//
// Each box is positioned with one face TANGENT to the case OD at the angle
// determined by crown_axis_dir, oriented so DX = radial outward (radial
// extent), DY = tangential (thickness), DZ = axial (height).

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "crown_guard_compound DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    // Build orthonormal frame at the crown side.
    const gp_Dir axial = in.case_axis.Direction();
    // Normalize the radial-outward direction we receive.
    gp_Vec rOutVec(in.crown_axis_dir);
    // Project rOut to the plane perpendicular to axial.
    rOutVec = rOutVec - gp_Vec(axial) * (rOutVec.Dot(gp_Vec(axial)));
    if (rOutVec.Magnitude() < 1e-9)
        throw SkillError("crown_guard_compound: crown_axis_dir parallel to case axis");
    rOutVec.Normalize();
    const gp_Dir radialOut(rOutVec);
    // Tangent (CCW around case axis): axial × radialOut.
    gp_Vec tangentVec = gp_Vec(axial).Crossed(gp_Vec(radialOut));
    tangentVec.Normalize();
    const gp_Dir tangentCCW(tangentVec);

    // Anchor on case OD at crown height.
    const gp_Pnt anchor(
        in.case_axis.Location().X() + in.case_outer_radius_mm * radialOut.X(),
        in.case_axis.Location().Y() + in.case_outer_radius_mm * radialOut.Y(),
        in.crown_axis_origin.Z());

    // Bias guard slightly INSIDE the case OD so the fuse leaves no sliver.
    constexpr double kRadialBias = 0.02;

    auto buildGuardBox = [&](double axialSign) {
        // Box origin: corner of the box.
        //   X-axis = radialOut, length = guard_radial_extent_mm + bias
        //   Y-axis = tangentCCW (auto via gp_Ax2)
        //   Z-axis = axial      (Main direction)
        //
        // gp_Ax2(P, V, Vx): V is local Z, Vx is local X.
        // box(ax, dx, dy, dz) → DX along Vx, DY along V×Vx, DZ along V.
        //
        // We want DX (radial) = guard_radial_extent + bias,
        //          DY (tangential) = guard_thickness,
        //          DZ (axial) = guard_height.
        //
        // Origin = anchor offset by:
        //   −kRadialBias × radialOut   (start slightly inside case OD)
        //   −guard_thickness/2 × tangentCCW (center tangentially)
        //   +axialSign × guard_separation/2 × axial − guard_height/2 × axial
        const double zOffset = axialSign * in.guard_separation_mm / 2.0
                             - in.guard_height_mm / 2.0;
        const gp_Pnt origin(
            anchor.X() - kRadialBias * radialOut.X()
                       - in.guard_thickness_mm / 2.0 * tangentCCW.X()
                       + zOffset * axial.X(),
            anchor.Y() - kRadialBias * radialOut.Y()
                       - in.guard_thickness_mm / 2.0 * tangentCCW.Y()
                       + zOffset * axial.Y(),
            anchor.Z() - kRadialBias * radialOut.Z()
                       - in.guard_thickness_mm / 2.0 * tangentCCW.Z()
                       + zOffset * axial.Z());
        const gp_Ax2 ax(origin, axial, radialOut);
        return pr::box(
            ax,
            in.guard_radial_extent_mm + kRadialBias,  // DX = radial
            in.guard_thickness_mm,                     // DY = tangential
            in.guard_height_mm);                       // DZ = axial
    };

    // ── Sub-feature 1: upper guard ───────────────────────────────────────
    const TopoDS_Shape upperGuard = buildGuardBox(+1.0);
    // ── Sub-feature 2: lower guard ───────────────────────────────────────
    const TopoDS_Shape lowerGuard = buildGuardBox(-1.0);

    // Fuse both onto the case body.
    const TopoDS_Shape afterUpper = pr::fuse(wp.shape(), upperGuard);
    const TopoDS_Shape newShape   = pr::fuse(afterUpper, lowerGuard);

    // ── Volumes ──────────────────────────────────────────────────────────
    const double perGuardVol = in.guard_radial_extent_mm *
                               in.guard_thickness_mm *
                               in.guard_height_mm;
    const double totalAdded = 2.0 * perGuardVol;

    // ── Signature ────────────────────────────────────────────────────────
    json params = {
        { "case_axis_loc",       { in.case_axis.Location().X(),
                                   in.case_axis.Location().Y(),
                                   in.case_axis.Location().Z() } },
        { "case_axis_dir",       { axial.X(), axial.Y(), axial.Z() } },
        { "case_outer_radius_mm", in.case_outer_radius_mm },
        { "crown_axis_origin",   { in.crown_axis_origin.X(),
                                   in.crown_axis_origin.Y(),
                                   in.crown_axis_origin.Z() } },
        { "crown_axis_dir",      { in.crown_axis_dir.X(),
                                   in.crown_axis_dir.Y(),
                                   in.crown_axis_dir.Z() } },
        { "guard_height_mm",      in.guard_height_mm },
        { "guard_thickness_mm",   in.guard_thickness_mm },
        { "guard_radial_extent_mm", in.guard_radial_extent_mm },
        { "guard_separation_mm",  in.guard_separation_mm },
        { "guard_chamfer_mm",     in.guard_chamfer_mm },
    };
    json pattern = {
        { "kind",                kSkillId },
        { "is_compound",         true },
        { "is_watch_feature",    true },
        { "subfeature_count",    2 },
        { "subfeatures", json::array({
            { { "op", "upper_guard_fuse" },
              { "height_mm",    in.guard_height_mm },
              { "thickness_mm", in.guard_thickness_mm },
              { "radial_extent_mm", in.guard_radial_extent_mm } },
            { { "op", "lower_guard_fuse" },
              { "height_mm",    in.guard_height_mm },
              { "thickness_mm", in.guard_thickness_mm },
              { "radial_extent_mm", in.guard_radial_extent_mm } },
        }) },
        { "guard_count",          2 },
        { "guard_thickness_mm",   in.guard_thickness_mm },
        { "guard_height_mm",      in.guard_height_mm },
        { "guard_separation_mm",  in.guard_separation_mm },
        { "derived_volume_added_mm3", totalAdded },
        { "protrusion_pair",      true },
        { "radial_dir",           { radialOut.X(), radialOut.Y(), radialOut.Z() } },
        { "axis_dir",             { axial.X(), axial.Y(), axial.Z() } },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "additive_form;chamfer_mill";
    tooling.tool_dia_mm       = std::max(in.guard_thickness_mm, 1.0);
    tooling.tool_length_mm    = in.guard_height_mm + 2.0;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 2;
    tooling.cutting_speed_sfm = 200.0;
    tooling.feed_per_tooth_mm = 0.03;
    tooling.stock_removed_mm3 = 0.0;
    tooling.est_cycle_time_s  = std::max(4.0, totalAdded / 100.0);
    tooling.extra = {
        { "feature_standard", "Panerai-style crown guard shoulders" },
        { "added_volume_mm3", totalAdded },
    };

    FeatureSignature sig { kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);  // keep full chain history
    wpNew->addFeature(sig);

    spdlog::debug("skill::crown_guard_compound applied: H={} T={} sep={}",
                  in.guard_height_mm, in.guard_thickness_mm, in.guard_separation_mm);

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
            { "protrusion_pair", true },
        };
        out.push_back(r);
    }
    return out;
}

}  // namespace koocadcam::skill::crown_guard_compound
