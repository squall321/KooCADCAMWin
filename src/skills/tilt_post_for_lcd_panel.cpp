// @lat: [[engine/skills#tilt_post_for_lcd_panel]]

#include "tilt_post_for_lcd_panel.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepAdaptor_Surface.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <gp.hxx>
#include <gp_Ax2.hxx>
#include <gp_Cylinder.hxx>
#include <gp_Pnt.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <array>
#include <cmath>

namespace koocadcam::skill::tilt_post_for_lcd_panel {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Panel tilt-post spec table ───────────────────────────────────────────
//
// Pin Ø + pocket depth + back-support pillar W × D × H, per LCD panel
// size class.  Circlip groove width per DIN 471 (≈ 0.1·pin_Ø for small
// shafts).

namespace {

struct PanelSpec {
    const char* size;
    double pin_dia_mm;
    double pocket_depth_mm;
    double pillar_w_mm;
    double pillar_d_mm;
    double pillar_h_mm;
    double circlip_groove_w_mm;  // DIN 471 axial groove width
    double circlip_groove_depth_mm;
    double circlip_z_offset_mm;  // from pocket entry
};

constexpr std::array<PanelSpec, 5> kPanelTable {{
    { "PANEL-7",   3.0,  6.0,  8.0,  6.0, 12.0, 0.4, 0.3, 1.5 },
    { "PANEL-10",  4.0,  8.0, 10.0,  8.0, 16.0, 0.5, 0.4, 2.0 },
    { "PANEL-13",  5.0, 10.0, 12.0, 10.0, 20.0, 0.6, 0.5, 2.5 },
    { "PANEL-15",  6.0, 12.0, 14.0, 12.0, 24.0, 0.7, 0.6, 3.0 },
    { "PANEL-17",  8.0, 14.0, 16.0, 14.0, 28.0, 0.9, 0.8, 3.5 },
}};

const PanelSpec* findSpec(const std::string& size)
{
    for (const auto& g : kPanelTable) {
        if (size == g.size) return &g;
    }
    return nullptr;
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    const PanelSpec* sp = findSpec(in.panel_size);
    if (!sp) {
        r.add("DFM-CONN-SIZE", "error",
              "tilt_post_for_lcd_panel: unknown panel_size '" +
              in.panel_size +
              "' (expected PANEL-7/10/13/15/17)");
        return r;
    }

    if (sp->pin_dia_mm < 0.8) {
        r.add("DFM-002", "error",
              "tilt_post_for_lcd_panel: pin Ø " +
              std::to_string(sp->pin_dia_mm) + " < 0.8 mm");
    }

    const double pillarWall = (sp->pillar_w_mm - sp->pin_dia_mm) / 2.0;
    if (pillarWall < 2.0) {
        r.add("DFM-PILLAR-WALL", "error",
              "tilt_post_for_lcd_panel: pillar wall " +
              std::to_string(pillarWall) + " < 2 mm");
    }

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double pocketEdgeMin = std::min({
        in.position_x_mm - xMin,
        xMax - in.position_x_mm,
        in.position_y_mm - yMin,
    });
    // The pillar grows in +Y from the pivot site; pivot site itself
    // must be at least pillar_w_mm/2 + 1.5 mm from the +X/-X edges.
    if (pocketEdgeMin < sp->pillar_w_mm / 2.0 + 1.5) {
        r.add("DFM-POCKET-WALL", "error",
              "tilt_post_for_lcd_panel: pivot position too close to plate "
              "edge (" + std::to_string(pocketEdgeMin) + " < " +
              std::to_string(sp->pillar_w_mm / 2.0 + 1.5) + ")");
    }

    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────
//
// Sub-features:
//   1. PIVOT POCKET     — blind cylinder cut (CUT)
//   2. CIRCLIP GROOVE   — annular ring cut INSIDE the pivot pocket (CUT)
//   3. BACK SUPPORT     — rectangular pillar fused on top of the plate (FUSE)
//   4. AUTO-EXTEND PLATE — when pillar extends past +Y bbox, fuse extension
//                          plate UNDER the pillar (FUSE)

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "tilt_post_for_lcd_panel DFM failed:";
        for (const auto& f : dfm.findings) msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const PanelSpec* sp = findSpec(in.panel_size);

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double kOverhang = 0.05;
    const double pinR = sp->pin_dia_mm / 2.0;
    const double grooveR = pinR + sp->circlip_groove_depth_mm;

    const double entryZ = (in.axis_dir.Z() < 0) ? zMax : zMin;
    (void)entryZ;
    const double signZ  = (in.axis_dir.Z() < 0) ? -1.0 : 1.0;

    // CONTEXT-AWARE: detect if the back-support pillar would extend
    // beyond the +Y edge of the plate.  If so, FUSE an extension plate
    // under the pillar.
    const double pillarBackY = in.position_y_mm + sp->pillar_d_mm + 1.0;
    const bool needExtend = pillarBackY > yMax;
    TopoDS_Shape currentShape = wp.shape();

    if (needExtend) {
        // Extension plate dimensions: spans the pillar footprint + 2 mm
        // margin, plus enough to attach to the existing yMax edge.
        const double extX0 = in.position_x_mm - sp->pillar_w_mm / 2.0 - 1.5;
        const double extX1 = in.position_x_mm + sp->pillar_w_mm / 2.0 + 1.5;
        const double extY0 = std::max(yMin, yMax - 1.0);  // overlap 1 mm
        const double extY1 = pillarBackY + 1.5;
        const double plateThickness = 2.0;
        const double extZ0 = (in.axis_dir.Z() < 0)
            ? zMax - plateThickness : zMin;
        const gp_Ax2 axExt(gp_Pnt(extX0, extY0, extZ0), gp::DZ());
        const TopoDS_Shape extPlate = pr::box(
            axExt, extX1 - extX0, extY1 - extY0, plateThickness);
        currentShape = pr::fuse(currentShape, extPlate);
    }

    // Refresh bounding box after possible extension.
    {
        Workpiece tmp(currentShape, wp.material());
        tmp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    }
    const double newEntryZ = (in.axis_dir.Z() < 0) ? zMax : zMin;

    // ── Sub-cut 1: pivot pocket (blind cylinder cut) ─────────────────────
    const gp_Ax2 axPocket(
        gp_Pnt(in.position_x_mm, in.position_y_mm,
               newEntryZ + signZ * kOverhang),
        in.axis_dir);
    const TopoDS_Shape pocket = pr::cylinder(
        axPocket, pinR, sp->pocket_depth_mm + kOverhang);

    // ── Sub-cut 2: circlip groove (annular ring) ─────────────────────────
    const double grooveCenterZ = newEntryZ + signZ * sp->circlip_z_offset_mm;
    const double grooveStartZ  = grooveCenterZ - sp->circlip_groove_w_mm / 2.0;
    const gp_Ax2 axGroove(
        gp_Pnt(in.position_x_mm, in.position_y_mm, grooveStartZ),
        gp::DZ());
    const TopoDS_Shape groove = pr::annularRing(
        axGroove, grooveR, pinR * 0.99, sp->circlip_groove_w_mm);

    // FUSE pocket + groove, then CUT once
    TopoDS_Shape pocketCutter = pr::fuse(pocket, groove);
    currentShape = pr::cut(currentShape, pocketCutter);

    // ── Sub-feature 3: back-support pillar (FUSE material) ───────────────
    // Pillar grows from the entry surface in +Z direction, behind the pivot.
    const double pillarX0 = in.position_x_mm - sp->pillar_w_mm / 2.0;
    const double pillarY0 = in.position_y_mm + pinR + 1.0;  // 1 mm gap to pivot
    const double pillarZ0 = (in.axis_dir.Z() < 0) ? newEntryZ : newEntryZ - sp->pillar_h_mm;
    const gp_Ax2 axPillar(
        gp_Pnt(pillarX0, pillarY0, pillarZ0), gp::DZ());
    const TopoDS_Shape pillar = pr::box(
        axPillar, sp->pillar_w_mm, sp->pillar_d_mm, sp->pillar_h_mm);
    currentShape = pr::fuse(currentShape, pillar);

    // ── Signature ────────────────────────────────────────────────────────
    const int subCount = 3 + (needExtend ? 1 : 0);

    json params = {
        { "axis_dir",      { in.axis_dir.X(), in.axis_dir.Y(), in.axis_dir.Z() } },
        { "position_x_mm", in.position_x_mm },
        { "position_y_mm", in.position_y_mm },
        { "panel_size",    in.panel_size },
        { "auto_extended_plate", needExtend },
    };
    json pattern = {
        { "kind",                 kSkillId },
        { "is_compound",          true },
        { "subfeature_count",     subCount },
        { "panel_size",           in.panel_size },
        { "pin_dia_mm",           sp->pin_dia_mm },
        { "pocket_depth_mm",      sp->pocket_depth_mm },
        { "pillar_w_mm",          sp->pillar_w_mm },
        { "pillar_d_mm",          sp->pillar_d_mm },
        { "pillar_h_mm",          sp->pillar_h_mm },
        { "circlip_groove_w_mm",  sp->circlip_groove_w_mm },
        { "auto_extended_plate",  needExtend },
        { "axis_dir",             { in.axis_dir.X(), in.axis_dir.Y(), in.axis_dir.Z() } },
        { "position_x_mm",        in.position_x_mm },
        { "position_y_mm",        in.position_y_mm },
    };

    ToolingMeta tooling;
    tooling.tool_type        = "endmill;groove_form";
    tooling.tool_dia_mm      = sp->pin_dia_mm;
    tooling.tool_length_mm   = sp->pocket_depth_mm + 5.0;
    tooling.tool_material    = "carbide";
    tooling.flute_count      = 2;
    tooling.cutting_speed_sfm = 300.0;
    tooling.feed_per_tooth_mm = 0.04;
    // Pillar is ADDED material so we DO NOT count it; CB pocket removes:
    tooling.stock_removed_mm3 =
        M_PI * pinR * pinR * sp->pocket_depth_mm +
        M_PI * (grooveR * grooveR - pinR * pinR) * sp->circlip_groove_w_mm;
    tooling.est_cycle_time_s = std::max(3.0, sp->pocket_depth_mm * 0.6);
    tooling.extra = {
        { "tool_sequence", {
            { { "tool_type", "endmill" },     { "tool_dia_mm", sp->pin_dia_mm },
              { "depth_mm", sp->pocket_depth_mm } },
            { { "tool_type", "groove_form" }, { "depth_mm", sp->circlip_groove_depth_mm },
              { "width_mm", sp->circlip_groove_w_mm } },
        } },
        { "circlip_standard", "DIN_471" },
        { "panel_size",       in.panel_size },
        { "auto_extended_plate", needExtend },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(currentShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::tilt_post_for_lcd_panel applied: {} at ({},{}) "
                  "extended={}",
                  in.panel_size, in.position_x_mm, in.position_y_mm,
                  needExtend);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;

    // Metadata path
    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        RecognizedFeature rf;
        rf.skill_id         = kSkillId;
        rf.recovered_params = f.params;
        rf.confidence       = 1.0;
        rf.matched_geometry = { { "source", "metadata" },
                                { "is_compound", true } };
        out.push_back(rf);
    }
    if (!out.empty()) return out;

    // Geometric heuristic: cylindrical face matching one of the spec pin Ø.
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFaceCylinder(i)) continue;
        BRepAdaptor_Surface surf(wp.face(i));
        const gp_Cylinder cyl = surf.Cylinder();
        const double dia = 2.0 * cyl.Radius();
        for (const auto& g : kPanelTable) {
            if (std::abs(dia - g.pin_dia_mm) < 0.1) {
                const gp_Pnt loc = cyl.Axis().Location();
                json rp = {
                    { "axis_dir",      json::array({ 0.0, 0.0, -1.0 }) },
                    { "position_x_mm", loc.X() },
                    { "position_y_mm", loc.Y() },
                    { "panel_size",    g.size },
                };
                RecognizedFeature rf;
                rf.skill_id         = kSkillId;
                rf.recovered_params = rp;
                rf.confidence       = 0.55;
                rf.matched_geometry = {
                    { "face_id",      i },
                    { "matched_size", g.size },
                    { "source",       "geometric" },
                };
                out.push_back(rf);
                break;
            }
        }
    }
    return out;
}

}  // namespace koocadcam::skill::tilt_post_for_lcd_panel
