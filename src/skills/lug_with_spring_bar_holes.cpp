// @lat: [[engine/skills#lug_with_spring_bar_holes]]

#include "lug_with_spring_bar_holes.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <gp_Ax2.hxx>
#include <gp_Pnt.hxx>

#include <algorithm>
#include <array>
#include <cmath>
#include <memory>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::skill::lug_with_spring_bar_holes {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

namespace {

// Bergeon / Seiko service-catalog standard spring bar diameters.
constexpr std::array<double, 3> kSpringBarStandardDias { 1.5, 1.6, 1.8 };

bool isStandardSpringBar(double dia_mm, double tol_mm = 0.05)
{
    for (double d : kSpringBarStandardDias)
        if (std::abs(d - dia_mm) < tol_mm) return true;
    return false;
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.case_outer_radius_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "lug_with_spring_bar_holes: case_outer_radius_mm must be > 0");
    }
    if (in.lug_length_mm <= 0.0 || in.lug_width_mm <= 0.0 ||
        in.lug_thickness_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "lug_with_spring_bar_holes: lug dimensions must all be > 0");
    }

    if (!isStandardSpringBar(in.spring_bar_dia_mm)) {
        r.add("DFM-SPRINGBAR-SIZE", "error",
              "lug_with_spring_bar_holes: spring_bar_dia " +
              std::to_string(in.spring_bar_dia_mm) +
              " mm not in Bergeon catalog (1.5/1.6/1.8 mm)");
    }
    if (in.spring_bar_dia_mm < 0.8) {
        r.add("DFM-002", "error",
              "lug_with_spring_bar_holes: spring_bar_dia " +
              std::to_string(in.spring_bar_dia_mm) + " mm < min drill 0.8 mm");
    }

    // Bar-to-lug-end material check.
    if (in.lug_length_mm < 4.0 * in.spring_bar_dia_mm) {
        r.add("DFM-LUG-LENGTH", "error",
              "lug_with_spring_bar_holes: lug_length " +
              std::to_string(in.lug_length_mm) +
              " mm < 4 × spring_bar_dia (" +
              std::to_string(4.0 * in.spring_bar_dia_mm) +
              ") — insufficient material");
    }
    if (in.lug_width_mm < 2.0 * in.spring_bar_dia_mm) {
        r.add("DFM-LUG-WIDTH", "error",
              "lug_with_spring_bar_holes: lug_width " +
              std::to_string(in.lug_width_mm) +
              " mm < 2 × spring_bar_dia (" +
              std::to_string(2.0 * in.spring_bar_dia_mm) + ")");
    }
    if (in.lug_length_mm > 16.0) {
        r.add("DFM-LUG-LENGTH", "warning",
              "lug_with_spring_bar_holes: lug_length " +
              std::to_string(in.lug_length_mm) +
              " mm > 16 mm — exceeds typical watch proportions");
    }

    if (in.corner_chamfer_mm > std::min(in.lug_thickness_mm, in.lug_width_mm) / 4.0) {
        r.add("DFM-LUG-CHAMFER", "warning",
              "lug_with_spring_bar_holes: corner_chamfer too large for lug section");
    }

    if (in.spring_bar_inset_mm < in.spring_bar_dia_mm) {
        r.add("DFM-LUG-INSET", "error",
              "lug_with_spring_bar_holes: spring_bar_inset (" +
              std::to_string(in.spring_bar_inset_mm) +
              ") < spring_bar_dia — hole breaks through lug end");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────
//
// Sub-feature chain (4 ops):
//   1. lug box fuse        (additive — rectangular protrusion off case wall)
//   2. corner chamfer #1   (two top outer edges, approximated as small cuts)
//   3. spring-bar hole #1  (through-hole at lug end 1)
//   4. spring-bar hole #2  (through-hole at lug end 2)

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "lug_with_spring_bar_holes DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    // Lug local frame at the angular position around the case.
    const double a = in.lug_angle_deg * M_PI / 180.0;
    // Radial outward unit (XY plane).
    const gp_Dir radialOut(std::cos(a), std::sin(a), 0.0);
    // Tangential CCW unit.
    const gp_Dir tangentCCW(-std::sin(a), std::cos(a), 0.0);
    // Axial = case axis (typically +Z).
    const gp_Dir axial = in.case_axis.Direction();

    // Anchor point on the case OD at lug height.
    const gp_Pnt caseSurfacePt(
        in.case_axis.Location().X() + in.case_outer_radius_mm * radialOut.X(),
        in.case_axis.Location().Y() + in.case_outer_radius_mm * radialOut.Y(),
        in.lug_attach_z_mm);

    // ── Sub-feature 1: build the lug box (additive) ──────────────────────
    // Origin: case surface, centered tangentially and axially.
    // Box frame: main = radialOut (so DZ extrudes outward by lug_length),
    //            xDir = axial    (so DX extrudes axially by lug_width).
    //            yDir = main × xDir = radialOut × axial = -tangentCCW
    //                                          → DY extrudes by lug_thickness
    //                                          along -tangentCCW direction.
    const gp_Pnt boxOrigin(
        caseSurfacePt.X() - axial.X() * (in.lug_width_mm / 2.0)
                         + tangentCCW.X() * (in.lug_thickness_mm / 2.0),
        caseSurfacePt.Y() - axial.Y() * (in.lug_width_mm / 2.0)
                         + tangentCCW.Y() * (in.lug_thickness_mm / 2.0),
        caseSurfacePt.Z() - axial.Z() * (in.lug_width_mm / 2.0)
                         + tangentCCW.Z() * (in.lug_thickness_mm / 2.0));
    const gp_Ax2 boxAx(boxOrigin, radialOut, axial);
    const TopoDS_Shape lugBox = pr::box(
        boxAx, in.lug_length_mm, in.lug_thickness_mm, in.lug_width_mm);

    // ── Sub-feature 2: corner chamfer — approximated as 2 small box cuts
    //    on the outer tip of the lug (the "front" face at radialOut · lug_length).
    const double cR = in.corner_chamfer_mm;
    const gp_Pnt tipCenter(
        caseSurfacePt.X() + radialOut.X() * in.lug_length_mm,
        caseSurfacePt.Y() + radialOut.Y() * in.lug_length_mm,
        caseSurfacePt.Z());
    // Two small chamfer tools: at +axial corner and -axial corner of the tip.
    auto buildChamferTool = [&](double sgn) {
        const gp_Pnt corner(
            tipCenter.X() - radialOut.X() * cR + axial.X() * sgn * in.lug_width_mm / 2.0,
            tipCenter.Y() - radialOut.Y() * cR + axial.Y() * sgn * in.lug_width_mm / 2.0,
            tipCenter.Z() - radialOut.Z() * cR + axial.Z() * sgn * in.lug_width_mm / 2.0);
        const gp_Ax2 cAx(corner, radialOut, axial);
        return pr::box(cAx, cR * 2.0, in.lug_thickness_mm + 0.1,
                       sgn > 0 ? cR : -cR + 0.01);
    };
    // (Slice-1: chamfer modelled as metadata; we don't subtract a literal box
    // because it could leave fragile sliver geometry on the lug tip.  The
    // chamfer is recorded in the signature for CAPP scheduling.)
    (void)buildChamferTool;

    // Fuse the lug onto the case body.
    const TopoDS_Shape caseWithLug = pr::fuse(wp.shape(), lugBox);

    // ── Sub-feature 3 & 4: spring-bar through holes ──────────────────────
    // Two coaxial holes, axis = axial (case axis direction), passing through
    // the lug at +/- (lug_width/2 - spring_bar_inset) from lug centre.
    const double rBar = in.spring_bar_dia_mm / 2.0;
    const double holeOffset = in.lug_width_mm / 2.0 - in.spring_bar_inset_mm;
    // Hole cylinders are oriented along axial direction; they need to be long
    // enough to pass clean through the lug thickness.  Extend a bit on both
    // sides of the lug (+ a generous overhang).
    const double cylH = in.lug_thickness_mm * 2.0 + in.lug_width_mm + 4.0;

    auto buildSpringBarHole = [&](double axialSign) {
        // Hole centre: on the lug centerline (radially halfway out), offset
        // axially by axialSign × holeOffset.  Origin of the cylinder is the
        // axial-low end of the through cut.
        const gp_Pnt holeCentre(
            caseSurfacePt.X() + radialOut.X() * (in.lug_length_mm / 2.0)
                              + axial.X()    * axialSign * holeOffset,
            caseSurfacePt.Y() + radialOut.Y() * (in.lug_length_mm / 2.0)
                              + axial.Y()    * axialSign * holeOffset,
            caseSurfacePt.Z() + radialOut.Z() * (in.lug_length_mm / 2.0)
                              + axial.Z()    * axialSign * holeOffset);
        // Start the cylinder cylH/2 below the centre along the cylinder axis.
        // We choose the cut axis = tangentCCW (cross-bore through the lug
        // perpendicular to the axial direction — typical watch spring-bar
        // orientation).
        const gp_Pnt cylBase(
            holeCentre.X() - tangentCCW.X() * cylH / 2.0,
            holeCentre.Y() - tangentCCW.Y() * cylH / 2.0,
            holeCentre.Z() - tangentCCW.Z() * cylH / 2.0);
        const gp_Ax2 hAx(cylBase, tangentCCW);
        return pr::cylinder(hAx, rBar, cylH);
    };

    const TopoDS_Shape hole1 = buildSpringBarHole(+1.0);
    const TopoDS_Shape hole2 = buildSpringBarHole(-1.0);

    // Fuse the two hole cutters then single subtractive cut.
    const TopoDS_Shape holeCutter = pr::fuse(hole1, hole2);
    const TopoDS_Shape newShape   = pr::cut(caseWithLug, holeCutter);

    // ── Signature ────────────────────────────────────────────────────────
    json params = {
        { "case_axis_loc",         { in.case_axis.Location().X(),
                                     in.case_axis.Location().Y(),
                                     in.case_axis.Location().Z() } },
        { "case_axis_dir",         { axial.X(), axial.Y(), axial.Z() } },
        { "case_outer_radius_mm",  in.case_outer_radius_mm },
        { "lug_angle_deg",         in.lug_angle_deg },
        { "lug_attach_z_mm",       in.lug_attach_z_mm },
        { "lug_length_mm",         in.lug_length_mm },
        { "lug_width_mm",          in.lug_width_mm },
        { "lug_thickness_mm",      in.lug_thickness_mm },
        { "corner_chamfer_mm",     in.corner_chamfer_mm },
        { "spring_bar_dia_mm",     in.spring_bar_dia_mm },
        { "spring_bar_inset_mm",   in.spring_bar_inset_mm },
    };
    json pattern = {
        { "kind",                kSkillId },
        { "is_compound",         true },
        { "subfeature_count",    4 },
        { "subfeatures", json::array({
            { { "op", "lug_box_fuse" },        { "length_mm", in.lug_length_mm }, { "width_mm", in.lug_width_mm } },
            { { "op", "corner_chamfer" },      { "chamfer_mm", in.corner_chamfer_mm } },
            { { "op", "spring_bar_hole_1" },   { "dia_mm", in.spring_bar_dia_mm } },
            { { "op", "spring_bar_hole_2" },   { "dia_mm", in.spring_bar_dia_mm } },
        }) },
        { "lug_length_mm",       in.lug_length_mm },
        { "lug_width_mm",        in.lug_width_mm },
        { "lug_thickness_mm",    in.lug_thickness_mm },
        { "spring_bar_dia_mm",   in.spring_bar_dia_mm },
        { "hole_count",          2 },
        { "axis_dir",            { axial.X(), axial.Y(), axial.Z() } },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "additive_form;drill";
    tooling.tool_dia_mm       = in.spring_bar_dia_mm;
    tooling.tool_length_mm    = in.lug_width_mm + 2.0;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 2;
    tooling.cutting_speed_sfm = 200.0;
    tooling.feed_per_tooth_mm = 0.03;
    const double lugVol =
        in.lug_length_mm * in.lug_width_mm * in.lug_thickness_mm;
    const double holeVol =
        2.0 * M_PI * rBar * rBar * in.lug_width_mm;
    tooling.stock_removed_mm3 = std::max(0.0, holeVol);   // additive net = 0 for lug
    tooling.est_cycle_time_s  = std::max(4.0, lugVol / 200.0);
    tooling.extra = {
        { "tool_sequence", {
            { { "tool_type", "additive_form" },
              { "note", "lug shaped from solid stock OR welded — modelled as fuse" } },
            { { "tool_type", "drill" }, { "tool_dia_mm", in.spring_bar_dia_mm },
              { "pass_count", 2 } },
        } },
        { "feature_standard", "Bergeon spring bar Ø " +
                              std::to_string(in.spring_bar_dia_mm) + " mm" },
        { "net_volume_change_mm3", lugVol - holeVol },
    };

    FeatureSignature sig { kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);  // keep full chain history
    wpNew->addFeature(sig);

    spdlog::debug("skill::lug_with_spring_bar_holes applied: L={} W={} T={} bar={}",
                  in.lug_length_mm, in.lug_width_mm, in.lug_thickness_mm,
                  in.spring_bar_dia_mm);

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
            { "source", "metadata_replay" },
            { "is_compound", true },
        };
        out.push_back(r);
    }
    return out;
}

}  // namespace koocadcam::skill::lug_with_spring_bar_holes
