// @lat: [[engine/skills#overcenter_latch]]

#include "overcenter_latch.hpp"

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

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::skill::overcenter_latch {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;
    if (in.lever_arm_length_mm < 6.0) {
        r.add("DFM-OVERCENTER-ARM", "error",
              "overcenter_latch: lever_arm_length " +
              std::to_string(in.lever_arm_length_mm) +
              " mm < 6 mm functional minimum");
    }
    if (in.lever_arm_width_mm < 0.8) {
        r.add("DFM-002", "error",
              "overcenter_latch: lever_arm_width " +
              std::to_string(in.lever_arm_width_mm) + " mm < 0.8 mm");
    }
    if (in.pivot_bore_dia_mm < 1.6) {
        r.add("DFM-OVERCENTER-PIVOT", "error",
              "overcenter_latch: pivot_bore_dia " +
              std::to_string(in.pivot_bore_dia_mm) +
              " mm < 1.6 mm (M1.6 minimum)");
    }
    if (in.cam_slot_length_mm <= in.cam_slot_width_mm) {
        r.add("DFM-OVERCENTER-CAM", "error",
              "overcenter_latch: cam_slot_length must be > cam_slot_width "
              "(else not a slot)");
    }
    if (in.cam_slot_width_mm < 0.8) {
        r.add("DFM-002", "error",
              "overcenter_latch: cam_slot_width " +
              std::to_string(in.cam_slot_width_mm) + " mm < 0.8 mm");
    }
    if (in.retention_hook_depth_mm > in.pivot_bore_depth_mm) {
        r.add("DFM-OVERCENTER-HOOK", "error",
              "overcenter_latch: retention_hook_depth " +
              std::to_string(in.retention_hook_depth_mm) +
              " mm > pivot_bore_depth (hook breaks through)");
    }
    if (in.lever_pocket_depth_mm <= 0.0
        || in.cam_slot_depth_mm <= 0.0
        || in.retention_hook_depth_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "overcenter_latch: all depths must be > 0");
    }
    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "overcenter_latch DFM failed:";
        for (const auto& f : dfm.findings) msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    auto entryId = wp.resolve(in.entry_face);
    if (!entryId) throw SkillError("overcenter_latch: entry_face datum unresolved");

    if (std::abs(in.axis_dir.X()) > 1e-6 || std::abs(in.axis_dir.Y()) > 1e-6
        || in.axis_dir.Z() >= 0) {
        throw SkillError("overcenter_latch: only axis_dir = -Z supported in slice 1");
    }

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double kOverhang = 0.05;
    const double zTop = zMax;

    // ── Sub-feature 1: lever arm pocket (rectangle along +X) ─────────────
    // Pocket extends from pivot center along +X for lever_arm_length.
    const gp_Pnt leverOrigin(
        in.position_x_mm,
        in.position_y_mm - in.lever_arm_width_mm / 2.0,
        zTop - in.lever_pocket_depth_mm);
    const gp_Ax2 leverAx(leverOrigin, gp::DZ());
    const TopoDS_Shape leverPocket = pr::box(
        leverAx,
        in.lever_arm_length_mm,
        in.lever_arm_width_mm,
        in.lever_pocket_depth_mm + kOverhang);

    // ── Sub-feature 2: cam slot — rectangle (slice-1 simplification of
    //                  stadium shape) at the mid of lever pocket
    const double camX = in.position_x_mm + in.cam_slot_offset_mm;
    const gp_Pnt camOrigin(
        camX,
        in.position_y_mm - in.cam_slot_width_mm / 2.0,
        zTop - in.cam_slot_depth_mm);
    const gp_Ax2 camAx(camOrigin, gp::DZ());
    const TopoDS_Shape camSlot = pr::box(
        camAx,
        in.cam_slot_length_mm,
        in.cam_slot_width_mm,
        in.cam_slot_depth_mm + kOverhang);

    // ── Sub-feature 3: pivot bore (cylinder -Z) ──────────────────────────
    const gp_Pnt pivotStart(in.position_x_mm, in.position_y_mm,
                            zTop + kOverhang);
    const gp_Ax2 pivotAx(pivotStart, gp_Dir(0.0, 0.0, -1.0));
    const TopoDS_Shape pivotBore = pr::cylinder(
        pivotAx, in.pivot_bore_dia_mm / 2.0,
        in.pivot_bore_depth_mm + 2.0 * kOverhang);

    // ── Sub-feature 4: retention hook pocket at far end ─────────────────
    const double hookX = in.position_x_mm + in.lever_arm_length_mm
                         - in.retention_hook_length_mm;
    const gp_Pnt hookOrigin(
        hookX,
        in.position_y_mm - in.retention_hook_width_mm / 2.0,
        zTop - in.retention_hook_depth_mm);
    const gp_Ax2 hookAx(hookOrigin, gp::DZ());
    const TopoDS_Shape hookPocket = pr::box(
        hookAx,
        in.retention_hook_length_mm,
        in.retention_hook_width_mm,
        in.retention_hook_depth_mm + kOverhang);

    // ── Sub-feature 5: grip notch — small box on entry face near pivot ──
    const gp_Pnt notchOrigin(
        in.position_x_mm - in.grip_notch_width_mm,
        in.position_y_mm - in.lever_arm_width_mm,
        zTop - in.grip_notch_depth_mm);
    const gp_Ax2 notchAx(notchOrigin, gp::DZ());
    const TopoDS_Shape gripNotch = pr::box(
        notchAx,
        in.grip_notch_width_mm,
        in.lever_arm_width_mm * 2.0,
        in.grip_notch_depth_mm + kOverhang);

    // Fuse all cutters then cut
    TopoDS_Shape fused = pr::fuse(leverPocket, camSlot);
    fused = pr::fuse(fused, pivotBore);
    fused = pr::fuse(fused, hookPocket);
    fused = pr::fuse(fused, gripNotch);
    const TopoDS_Shape newShape = pr::cut(wp.shape(), fused);

    json params = {
        { "entry_face_id",            *entryId },
        { "position_x_mm",            in.position_x_mm },
        { "position_y_mm",            in.position_y_mm },
        { "axis_dir",                 { in.axis_dir.X(), in.axis_dir.Y(), in.axis_dir.Z() } },
        { "lever_arm_length_mm",      in.lever_arm_length_mm },
        { "lever_arm_width_mm",       in.lever_arm_width_mm },
        { "lever_pocket_depth_mm",    in.lever_pocket_depth_mm },
        { "pivot_bore_dia_mm",        in.pivot_bore_dia_mm },
        { "pivot_bore_depth_mm",      in.pivot_bore_depth_mm },
        { "cam_slot_offset_mm",       in.cam_slot_offset_mm },
        { "cam_slot_length_mm",       in.cam_slot_length_mm },
        { "cam_slot_width_mm",        in.cam_slot_width_mm },
        { "cam_slot_depth_mm",        in.cam_slot_depth_mm },
        { "retention_hook_length_mm", in.retention_hook_length_mm },
        { "retention_hook_width_mm",  in.retention_hook_width_mm },
        { "retention_hook_depth_mm",  in.retention_hook_depth_mm },
        { "grip_notch_width_mm",      in.grip_notch_width_mm },
        { "grip_notch_depth_mm",      in.grip_notch_depth_mm },
    };
    json pattern = {
        { "kind",                kSkillId },
        { "is_compound",         true },
        { "subfeature_count",    5 },
        { "lever_arm_length_mm", in.lever_arm_length_mm },
        { "pivot_bore_dia_mm",   in.pivot_bore_dia_mm },
        { "cam_slot_length_mm",  in.cam_slot_length_mm },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "end_mill;drill";
    tooling.tool_dia_mm       = std::max(in.cam_slot_width_mm, in.pivot_bore_dia_mm);
    tooling.tool_length_mm    = in.lever_pocket_depth_mm + 5.0;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 4;
    tooling.cutting_speed_sfm = 300.0;
    tooling.feed_per_tooth_mm = 0.03;
    const double pivotR = in.pivot_bore_dia_mm / 2.0;
    tooling.stock_removed_mm3 =
        in.lever_arm_length_mm * in.lever_arm_width_mm * in.lever_pocket_depth_mm
        + in.cam_slot_length_mm * in.cam_slot_width_mm * in.cam_slot_depth_mm
        + M_PI * pivotR * pivotR * in.pivot_bore_depth_mm
        + in.retention_hook_length_mm * in.retention_hook_width_mm
              * in.retention_hook_depth_mm
        + in.grip_notch_width_mm * in.lever_arm_width_mm * 2.0
              * in.grip_notch_depth_mm;
    tooling.est_cycle_time_s = std::max(15.0,
        in.lever_arm_length_mm * 0.2 + in.cam_slot_length_mm * 0.3 + 5.0);
    tooling.extra = {
        { "tool_sequence", {
            { { "tool_type", "end_mill" }, { "tool_dia_mm", 4.0 },
              { "purpose", "lever arm pocket (Southco E5 style)" } },
            { { "tool_type", "end_mill" },
              { "tool_dia_mm", in.cam_slot_width_mm },
              { "purpose", "cam slot for over-center pin" } },
            { { "tool_type", "drill" },
              { "tool_dia_mm", in.pivot_bore_dia_mm },
              { "purpose", "pivot pin bore" } },
            { { "tool_type", "end_mill" }, { "tool_dia_mm", 2.0 },
              { "purpose", "retention hook pocket" } },
            { { "tool_type", "end_mill" }, { "tool_dia_mm", 1.5 },
              { "purpose", "grip notch (operator finger)" } },
        } },
        { "standard_ref", "Southco E5 style + DIN 3017 lever geometry" },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    wpNew->addFeature(sig);

    spdlog::debug("skill::overcenter_latch applied: arm={} pivot={} cam={}x{} faces {}→{}",
                  in.lever_arm_length_mm, in.pivot_bore_dia_mm,
                  in.cam_slot_length_mm, in.cam_slot_width_mm,
                  wp.faceCount(), wpNew->faceCount());

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;

    // PRIMARY: geometric — find a single small Z-axis cylinder (pivot) and
    // at least 1 long rectangular pocket.
    int zCylCount = 0;
    double pivotDia = 0.0;
    gp_Pnt pivotCenter(0, 0, 0);
    bool pivotFound = false;

    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFaceCylinder(i)) continue;
        BRepAdaptor_Surface surf(wp.face(i));
        const gp_Cylinder c = surf.Cylinder();
        const gp_Dir d = c.Axis().Direction();
        if (std::abs(std::abs(d.Z()) - 1.0) < 1e-2) {
            // small (pivot-like) cylinder
            if (c.Radius() < 5.0) {
                zCylCount++;
                if (!pivotFound || c.Radius() > pivotDia / 2.0) {
                    pivotDia    = 2.0 * c.Radius();
                    pivotCenter = c.Axis().Location();
                    pivotFound  = true;
                }
            }
        }
    }

    // Count horizontal-bottom pocket faces (potential lever pockets)
    int rectPocketCount = 0;
    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFacePlanar(i)) continue;
        gp_Dir nb;
        try { nb = wp.faceNormal(i); } catch (...) { continue; }
        if (std::abs(nb.Z() - 1.0) > 1e-2) continue;
        const gp_Pnt c = wp.faceCenter(i);
        if (std::abs(c.Z() - zMax) < 1e-3) continue;  // top face
        rectPocketCount++;
    }

    if (pivotFound && rectPocketCount >= 2) {
        json recovered = {
            { "position_x_mm",            pivotCenter.X() },
            { "position_y_mm",            pivotCenter.Y() },
            { "axis_dir",                 { 0.0, 0.0, -1.0 } },
            { "pivot_bore_dia_mm",        pivotDia },
            { "lever_arm_length_mm",      30.0 },  // not recoverable geometrically
        };
        json matched = {
            { "z_cyl_count",      zCylCount },
            { "rect_pocket_count", rectPocketCount },
        };
        out.push_back(RecognizedFeature{ kSkillId, recovered, 0.6, matched });
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

}  // namespace koocadcam::skill::overcenter_latch
