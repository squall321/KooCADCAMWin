// @lat: [[engine/skills#gas_strut_hinge_compound]]

#include "gas_strut_hinge_compound.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepAdaptor_Surface.hxx>
#include <gp.hxx>
#include <gp_Ax2.hxx>
#include <gp_Cylinder.hxx>
#include <gp_Pnt.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <vector>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::skill::gas_strut_hinge_compound {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.pivot_bore_dia_mm < 3.0) {
        r.add("DFM-GAS-PIVOT", "error",
              "gas_strut_hinge_compound: pivot_bore_dia " +
              std::to_string(in.pivot_bore_dia_mm) +
              " mm < 3 mm (M3 minimum strut pin)");
    }
    const double spacing = std::hypot(
        in.door_pivot_x_mm - in.frame_pivot_x_mm,
        in.door_pivot_y_mm - in.frame_pivot_y_mm);
    if (spacing <= in.pivot_bore_dia_mm * 3.0) {
        r.add("DFM-GAS-SPACING", "error",
              "gas_strut_hinge_compound: pivot spacing " +
              std::to_string(spacing) +
              " mm must be > 3 × pivot_bore_dia (" +
              std::to_string(3.0 * in.pivot_bore_dia_mm) + " mm)");
    }
    if (in.clevis_pocket_width_mm <= in.pivot_bore_dia_mm) {
        r.add("DFM-GAS-CLEVIS", "error",
              "gas_strut_hinge_compound: clevis_pocket_width must be > "
              "pivot_bore_dia (else clevis cannot enter)");
    }
    if (in.clevis_pocket_width_mm < 0.8) {
        r.add("DFM-002", "error",
              "gas_strut_hinge_compound: clevis_pocket_width " +
              std::to_string(in.clevis_pocket_width_mm) + " mm < 0.8 mm");
    }
    if (in.ball_groove_inner_dia_mm <= in.pivot_bore_dia_mm) {
        r.add("DFM-GAS-BALLGROOVE", "error",
              "gas_strut_hinge_compound: ball_groove_inner_dia must be > "
              "pivot_bore_dia");
    }
    if (in.ball_groove_outer_dia_mm <= in.ball_groove_inner_dia_mm) {
        r.add("DFM-GAS-BALLGROOVE", "error",
              "gas_strut_hinge_compound: ball_groove_outer_dia must be > inner_dia");
    }
    if (in.lightening_hole_dia_mm > spacing * 0.5) {
        r.add("DFM-GAS-LIGHTEN", "error",
              "gas_strut_hinge_compound: lightening_hole_dia " +
              std::to_string(in.lightening_hole_dia_mm) +
              " mm > 0.5 × pivot spacing — bracket weakens");
    }
    if (in.bracket_thickness_mm <= in.ball_groove_depth_mm + 0.5) {
        r.add("DFM-GAS-THICK", "error",
              "gas_strut_hinge_compound: bracket too thin for ball groove");
    }
    if (in.clevis_pocket_depth_mm <= 0.0
        || in.ball_groove_depth_mm <= 0.0
        || in.bracket_thickness_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "gas_strut_hinge_compound: depths must be > 0");
    }
    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "gas_strut_hinge_compound DFM failed:";
        for (const auto& f : dfm.findings) msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    auto entryId = wp.resolve(in.entry_face);
    if (!entryId) throw SkillError("gas_strut_hinge_compound: entry_face datum unresolved");

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double kOverhang = 0.05;
    const gp_Dir adir = in.axis_dir;

    if (std::abs(adir.X()) > 1e-6 || std::abs(adir.Y()) > 1e-6
        || adir.Z() >= 0) {
        throw SkillError("gas_strut_hinge_compound: only axis_dir = -Z supported in slice 1");
    }

    const double zTop = zMax;

    // ── Sub-feature 1: frame pivot bore (through) ─────────────────────
    const gp_Pnt fpStart(in.frame_pivot_x_mm, in.frame_pivot_y_mm,
                         zTop + kOverhang);
    const gp_Ax2 fpAx(fpStart, gp_Dir(0.0, 0.0, -1.0));
    const TopoDS_Shape framePivot = pr::cylinder(
        fpAx, in.pivot_bore_dia_mm / 2.0,
        in.bracket_thickness_mm + 2.0 * kOverhang);

    // ── Sub-feature 2: door pivot bore (through) ──────────────────────
    const gp_Pnt dpStart(in.door_pivot_x_mm, in.door_pivot_y_mm,
                         zTop + kOverhang);
    const gp_Ax2 dpAx(dpStart, gp_Dir(0.0, 0.0, -1.0));
    const TopoDS_Shape doorPivot = pr::cylinder(
        dpAx, in.pivot_bore_dia_mm / 2.0,
        in.bracket_thickness_mm + 2.0 * kOverhang);

    // ── Sub-feature 3: clevis pocket — rectangular between pivots ─────
    const double mx = (in.frame_pivot_x_mm + in.door_pivot_x_mm) / 2.0;
    const double my = (in.frame_pivot_y_mm + in.door_pivot_y_mm) / 2.0;
    const gp_Pnt clevisOrigin(
        mx - in.clevis_pocket_length_mm / 2.0,
        my - in.clevis_pocket_width_mm  / 2.0,
        zTop - in.clevis_pocket_depth_mm);
    const gp_Ax2 clevisAx(clevisOrigin, gp::DZ());
    const TopoDS_Shape clevisPocket = pr::box(
        clevisAx,
        in.clevis_pocket_length_mm,
        in.clevis_pocket_width_mm,
        in.clevis_pocket_depth_mm + kOverhang);

    // ── Sub-feature 4: ball-end retention groove — annular ring on top
    //                  face concentric with the DOOR pivot
    const gp_Pnt grooveStart(
        in.door_pivot_x_mm, in.door_pivot_y_mm, zTop + kOverhang);
    const gp_Ax2 grooveAx(grooveStart, gp_Dir(0.0, 0.0, -1.0));
    const TopoDS_Shape ballGroove = pr::annularRing(
        grooveAx,
        in.ball_groove_outer_dia_mm / 2.0,
        in.ball_groove_inner_dia_mm / 2.0,
        in.ball_groove_depth_mm + kOverhang);

    // ── Sub-feature 5: lightening hole — small through-bore at midpoint
    const gp_Pnt lightStart(mx, my, zTop + kOverhang);
    const gp_Ax2 lightAx(lightStart, gp_Dir(0.0, 0.0, -1.0));
    const TopoDS_Shape lightHole = pr::cylinder(
        lightAx, in.lightening_hole_dia_mm / 2.0,
        in.bracket_thickness_mm + 2.0 * kOverhang);

    // Fuse all then cut
    TopoDS_Shape fused = pr::fuse(framePivot, doorPivot);
    fused = pr::fuse(fused, clevisPocket);
    fused = pr::fuse(fused, ballGroove);
    fused = pr::fuse(fused, lightHole);
    const TopoDS_Shape newShape = pr::cut(wp.shape(), fused);

    const double pivotSpacing = std::hypot(
        in.door_pivot_x_mm - in.frame_pivot_x_mm,
        in.door_pivot_y_mm - in.frame_pivot_y_mm);

    json params = {
        { "entry_face_id",             *entryId },
        { "frame_pivot_x_mm",          in.frame_pivot_x_mm },
        { "frame_pivot_y_mm",          in.frame_pivot_y_mm },
        { "door_pivot_x_mm",           in.door_pivot_x_mm },
        { "door_pivot_y_mm",           in.door_pivot_y_mm },
        { "axis_dir",                  { adir.X(), adir.Y(), adir.Z() } },
        { "bracket_thickness_mm",      in.bracket_thickness_mm },
        { "pivot_bore_dia_mm",         in.pivot_bore_dia_mm },
        { "clevis_pocket_length_mm",   in.clevis_pocket_length_mm },
        { "clevis_pocket_width_mm",    in.clevis_pocket_width_mm },
        { "clevis_pocket_depth_mm",    in.clevis_pocket_depth_mm },
        { "ball_groove_inner_dia_mm",  in.ball_groove_inner_dia_mm },
        { "ball_groove_outer_dia_mm",  in.ball_groove_outer_dia_mm },
        { "ball_groove_depth_mm",      in.ball_groove_depth_mm },
        { "lightening_hole_dia_mm",    in.lightening_hole_dia_mm },
    };
    json pattern = {
        { "kind",                      kSkillId },
        { "is_compound",               true },
        { "subfeature_count",          5 },
        { "pivot_spacing_mm",          pivotSpacing },
        { "pivot_bore_dia_mm",         in.pivot_bore_dia_mm },
        { "through_bore_count",        3 },     // 2 pivots + 1 lightening
        { "annular_groove_count",      1 },
        { "rect_pocket_count",         1 },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "drill;end_mill;counterbore";
    tooling.tool_dia_mm       = std::max({ in.pivot_bore_dia_mm,
                                           in.clevis_pocket_width_mm,
                                           in.ball_groove_outer_dia_mm });
    tooling.tool_length_mm    = in.bracket_thickness_mm + 5.0;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 4;
    tooling.cutting_speed_sfm = 320.0;
    tooling.feed_per_tooth_mm = 0.035;
    const double pivR  = in.pivot_bore_dia_mm / 2.0;
    const double gOR   = in.ball_groove_outer_dia_mm / 2.0;
    const double gIR   = in.ball_groove_inner_dia_mm / 2.0;
    const double lR    = in.lightening_hole_dia_mm / 2.0;
    tooling.stock_removed_mm3 =
        2.0 * M_PI * pivR * pivR * in.bracket_thickness_mm
        + in.clevis_pocket_length_mm * in.clevis_pocket_width_mm
              * in.clevis_pocket_depth_mm
        + M_PI * (gOR * gOR - gIR * gIR) * in.ball_groove_depth_mm
        + M_PI * lR * lR * in.bracket_thickness_mm;
    tooling.est_cycle_time_s = std::max(20.0,
        in.bracket_thickness_mm * 0.4
        + in.clevis_pocket_length_mm * 0.2 + 5.0);
    tooling.extra = {
        { "tool_sequence", {
            { { "tool_type", "drill" },
              { "tool_dia_mm", in.pivot_bore_dia_mm },
              { "pass_count", 2 },
              { "purpose", "frame & door pivot bores (DIN 71803)" } },
            { { "tool_type", "end_mill" },
              { "tool_dia_mm", in.clevis_pocket_width_mm },
              { "purpose", "SAE clevis pocket (ISO 8434-2)" } },
            { { "tool_type", "counterbore" },
              { "tool_dia_mm", in.ball_groove_outer_dia_mm },
              { "purpose", "ball-end snap retention groove (DIN 71803)" } },
            { { "tool_type", "drill" },
              { "tool_dia_mm", in.lightening_hole_dia_mm },
              { "purpose", "central lightening through-hole" } },
        } },
        { "standard_ref", "DIN 71803 + ISO 8434-2" },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    wpNew->addFeature(sig);

    spdlog::debug("skill::gas_strut_hinge_compound applied: spacing={} pivot={} faces {}→{}",
                  pivotSpacing, in.pivot_bore_dia_mm,
                  wp.faceCount(), wpNew->faceCount());

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;

    // PRIMARY: find pair of Z-axis cylinders of similar dia (the two pivots),
    // separated by a measurable distance.
    struct CylZ {
        double radius;
        gp_Pnt center;
    };
    std::vector<CylZ> cyls;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFaceCylinder(i)) continue;
        BRepAdaptor_Surface surf(wp.face(i));
        const gp_Cylinder c = surf.Cylinder();
        const gp_Dir d = c.Axis().Direction();
        if (std::abs(std::abs(d.Z()) - 1.0) > 1e-2) continue;
        cyls.push_back({ c.Radius(), c.Axis().Location() });
    }

    double bestSep = 0.0;
    double pivotR  = 0.0;
    gp_Pnt frameCenter(0, 0, 0), doorCenter(0, 0, 0);
    for (size_t i = 0; i < cyls.size(); ++i) {
        for (size_t j = i + 1; j < cyls.size(); ++j) {
            if (std::abs(cyls[i].radius - cyls[j].radius) > 0.5) continue;
            if (cyls[i].radius < 1.0) continue;
            const double sep = std::hypot(
                cyls[i].center.X() - cyls[j].center.X(),
                cyls[i].center.Y() - cyls[j].center.Y());
            if (sep < cyls[i].radius * 3.0) continue;
            if (sep > bestSep) {
                bestSep = sep;
                pivotR  = cyls[i].radius;
                frameCenter = cyls[i].center;
                doorCenter  = cyls[j].center;
            }
        }
    }

    if (bestSep > 0.0) {
        // Count off-axis cylinders (lightening hole) & rect pocket faces
        int extraCyl = 0;
        for (const auto& c : cyls) {
            const double dFrame = std::hypot(
                c.center.X() - frameCenter.X(),
                c.center.Y() - frameCenter.Y());
            const double dDoor = std::hypot(
                c.center.X() - doorCenter.X(),
                c.center.Y() - doorCenter.Y());
            if (dFrame > 1e-3 && dDoor > 1e-3
                && c.radius < pivotR) {
                extraCyl++;
            }
        }

        double xMin, yMin, zMin, xMax, yMax, zMax;
        wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);

        json recovered = {
            { "frame_pivot_x_mm",       frameCenter.X() },
            { "frame_pivot_y_mm",       frameCenter.Y() },
            { "door_pivot_x_mm",        doorCenter.X() },
            { "door_pivot_y_mm",        doorCenter.Y() },
            { "axis_dir",               { 0.0, 0.0, -1.0 } },
            { "bracket_thickness_mm",   zMax - zMin },
            { "pivot_bore_dia_mm",      2.0 * pivotR },
        };
        json matched = {
            { "pivot_spacing", bestSep },
            { "extra_cyl_count", extraCyl },
        };
        out.push_back(RecognizedFeature{ kSkillId, recovered, 0.7, matched });
    }

    // FALLBACK: metadata replay
    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        out.push_back(RecognizedFeature{
            kSkillId, f.params, 1.0,
            { { "via", "metadata_replay" }, { "subfeature_count", 5 } }
        });
    }
    return out;
}

}  // namespace koocadcam::skill::gas_strut_hinge_compound
