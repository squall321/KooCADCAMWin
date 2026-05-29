// @lat: [[engine/skills#tab_lock_anti_rotation]]

#include "tab_lock_anti_rotation.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <gp.hxx>
#include <gp_Ax2.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::skill::tab_lock_anti_rotation {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;
    if (in.tab_w_mm < 0.8 || in.tab_w_mm > 6.0) {
        r.add("DFM-TAB-W", "error",
              "tab_lock_anti_rotation: tab_w_mm " +
              std::to_string(in.tab_w_mm) +
              " outside SAE J1199 / mill-bit range [0.8, 6.0] mm");
    }
    if (in.tab_d_mm < 0.3 || in.tab_d_mm > 3.0) {
        r.add("DFM-TAB-D", "error",
              "tab_lock_anti_rotation: tab_d_mm " +
              std::to_string(in.tab_d_mm) +
              " outside typical range [0.3, 3.0] mm");
    }
    if (in.tab_axial_z_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "tab_lock_anti_rotation: tab_axial_z_mm must be > 0");
    }
    if (in.bore_dia_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "tab_lock_anti_rotation: bore_dia_mm must be > 0 (informational)");
    }
    if (in.bore_dia_mm > 0.0 && in.tab_d_mm > 0.0) {
        const double boreR = in.bore_dia_mm / 2.0;
        if (boreR < in.tab_d_mm + 1.0) {
            r.add("DFM-TAB-WALL", "error",
                  "tab_lock_anti_rotation: bore radius " +
                  std::to_string(boreR) +
                  " mm < tab_d + 1 mm wall — weakens bore wall");
        }
    }
    if (in.chamfer_size_mm < 0.1) {
        r.add("DFM-TAB-CHAMFER", "warning",
              "tab_lock_anti_rotation: chamfer too small (< 0.1 mm)");
    }
    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "tab_lock_anti_rotation DFM failed:";
        for (const auto& f : dfm.findings) msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const gp_Pnt bL = in.bore_axis.Location();
    const gp_Dir bD = in.bore_axis.Direction();
    const double boreR = in.bore_dia_mm / 2.0;

    // Build a perpendicular frame to the bore axis.
    // Pick localX = any direction perpendicular to bD.
    gp_Dir localX;
    if (std::abs(bD.Z()) < 0.99) {
        const gp_Vec v = gp_Vec(0, 0, 1) - gp_Vec(bD.X(), bD.Y(), bD.Z()) * bD.Z();
        localX = gp_Dir(v);
    } else {
        const gp_Vec v = gp_Vec(1, 0, 0) - gp_Vec(bD.X(), bD.Y(), bD.Z()) * bD.X();
        localX = gp_Dir(v);
    }
    // localY = bD × localX
    const gp_Vec lyVec = gp_Vec(bD.X(), bD.Y(), bD.Z()).Crossed(
                          gp_Vec(localX.X(), localX.Y(), localX.Z()));
    const gp_Dir localY(lyVec);

    // Notch as a box positioned at the bore rim along +localX (angle 0°),
    // extending RADIALLY OUTWARD by tab_d_mm (so it cuts a notch INTO the
    // bore wall).  Width along localY = tab_w_mm.  Height along bD = tab_axial_z_mm.
    //
    // We place the box such that its INNER face touches the bore at radius
    // (boreR - 0.01 — a tiny inset so the cut definitely intersects the
    // bore wall geometry).
    const double radialInset = 0.01;

    auto makeNotchCutter = [&](double angleRad) -> TopoDS_Shape {
        // Radial outward direction at this angle, expressed in (localX, localY).
        const gp_Vec radialVec =
              gp_Vec(localX.X(), localX.Y(), localX.Z()) * std::cos(angleRad)
            + gp_Vec(localY.X(), localY.Y(), localY.Z()) * std::sin(angleRad);
        const gp_Dir radialDir(radialVec);
        // Tangential direction (perpendicular to radial, in plane).
        const gp_Vec tangentVec =
              gp_Vec(localX.X(), localX.Y(), localX.Z()) * (-std::sin(angleRad))
            + gp_Vec(localY.X(), localY.Y(), localY.Z()) * ( std::cos(angleRad));
        const gp_Dir tangentDir(tangentVec);

        // Notch inner-face point: bore centre + (boreR - inset) × radial.
        // Origin of the box: shift to the box corner.
        // gp_Ax2(P, V, Vx): V = local Z (= bD, along bore axis), Vx = local X
        // (= radialDir — extrude DX = tab_d into the wall).
        // DY along (V × Vx) = bD × radialDir = tangentDir.
        // So origin must be at:
        //   centre point - tab_w/2 × tangentDir + (boreR - inset) × radialDir
        // and box dx = tab_d (radial), dy = tab_w (tangential),
        // dz = tab_axial_z (axial).
        const gp_Pnt origin(
            bL.X() + (boreR - radialInset) * radialDir.X()
                   - in.tab_w_mm / 2.0      * tangentDir.X(),
            bL.Y() + (boreR - radialInset) * radialDir.Y()
                   - in.tab_w_mm / 2.0      * tangentDir.Y(),
            bL.Z() + (boreR - radialInset) * radialDir.Z()
                   - in.tab_w_mm / 2.0      * tangentDir.Z());
        const gp_Ax2 ax(origin, bD, radialDir);
        return pr::box(ax, in.tab_d_mm + radialInset,
                            in.tab_w_mm, in.tab_axial_z_mm);
    };

    // ── Sub-features 1 + 2: two diametrically opposite notches ─────────
    const TopoDS_Shape notchA = makeNotchCutter(0.0);
    const TopoDS_Shape notchB = makeNotchCutter(M_PI);

    // ── Sub-feature 3: entry chamfer (cone frustum on the bore rim) ────
    // Bore rim is at the entry plane = bL + 0 × bD (slice 1 convention:
    // bL is the bore mouth).  Cone goes from boreR (small, at entry) to
    // boreR + chamfer_size (large, OUTSIDE, against bD direction).
    const gp_Pnt chamferStart(
        bL.X() - bD.X() * in.chamfer_size_mm,
        bL.Y() - bD.Y() * in.chamfer_size_mm,
        bL.Z() - bD.Z() * in.chamfer_size_mm);
    const gp_Ax2 chamferAx(chamferStart, bD);
    const TopoDS_Shape chamferCutter =
        pr::coneFrustum(chamferAx, boreR + in.chamfer_size_mm,
                                    boreR,
                                    in.chamfer_size_mm);

    // Fuse all cutters then single cut.
    const TopoDS_Shape fused1 = pr::fuse(notchA, notchB);
    const TopoDS_Shape fused2 = pr::fuse(fused1, chamferCutter);
    const TopoDS_Shape newShape = pr::cut(wp.shape(), fused2);

    json params = {
        { "bore_axis_loc",   { bL.X(), bL.Y(), bL.Z() } },
        { "bore_axis_dir",   { bD.X(), bD.Y(), bD.Z() } },
        { "bore_dia_mm",     in.bore_dia_mm },
        { "tab_w_mm",        in.tab_w_mm },
        { "tab_d_mm",        in.tab_d_mm },
        { "tab_axial_z_mm",  in.tab_axial_z_mm },
        { "chamfer_size_mm", in.chamfer_size_mm },
    };
    json pattern = {
        { "kind",              kSkillId },
        { "is_compound",       true },
        { "subfeature_count",  3 },
        { "tab_w_mm",          in.tab_w_mm },
        { "tab_d_mm",          in.tab_d_mm },
        { "tab_axial_z_mm",    in.tab_axial_z_mm },
        { "bore_dia_mm",       in.bore_dia_mm },
        { "notch_angles_deg",  { 0.0, 180.0 } },
        { "chamfer_size_mm",   in.chamfer_size_mm },
        { "bore_axis_dir",     { bD.X(), bD.Y(), bD.Z() } },
    };
    ToolingMeta tooling;
    tooling.tool_type         = "end_mill;chamfer";
    tooling.tool_dia_mm       = in.tab_w_mm;
    tooling.tool_length_mm    = in.tab_axial_z_mm + 5.0;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 3;
    tooling.cutting_speed_sfm = 320.0;
    tooling.feed_per_tooth_mm = 0.025;
    tooling.stock_removed_mm3 =
        2.0 * in.tab_w_mm * in.tab_d_mm * in.tab_axial_z_mm
        + M_PI * boreR * in.chamfer_size_mm * in.chamfer_size_mm;
    tooling.est_cycle_time_s = std::max(3.0,
        in.tab_axial_z_mm * 0.5 + 2.0);
    tooling.extra = {
        { "tool_sequence", {
            { { "tool_type", "end_mill" },
              { "tool_dia_mm", in.tab_w_mm },
              { "depth_mm",    in.tab_d_mm },
              { "purpose",     "tab-lock notch A (0°, SAE J1199)" } },
            { { "tool_type", "end_mill" },
              { "tool_dia_mm", in.tab_w_mm },
              { "depth_mm",    in.tab_d_mm },
              { "purpose",     "tab-lock notch B (180°, diametric)" } },
            { { "tool_type", "chamfer_tool" },
              { "tool_dia_mm", in.bore_dia_mm + 2.0 * in.chamfer_size_mm },
              { "depth_mm",    in.chamfer_size_mm },
              { "angle_deg",   45.0 },
              { "purpose",     "entry chamfer on bore rim" } },
        } },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    wpNew->addFeature(sig);

    spdlog::debug("skill::tab_lock_anti_rotation applied: tab {}x{}x{} bore {}",
                  in.tab_w_mm, in.tab_d_mm, in.tab_axial_z_mm, in.bore_dia_mm);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition — metadata replay ────────────────────────────────────────

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;
    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        out.push_back(RecognizedFeature{
            kSkillId, f.params, 1.0,
            { { "via", "metadata_replay" },
              { "subfeature_count", 3 } }
        });
    }
    return out;
}

}  // namespace koocadcam::skill::tab_lock_anti_rotation
