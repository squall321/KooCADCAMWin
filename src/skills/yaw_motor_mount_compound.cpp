// @lat: [[engine/skills#yaw_motor_mount_compound]]

#include "yaw_motor_mount_compound.hpp"

#include "Workpiece.hpp"
#include "_iso_thread_table.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepBuilderAPI_Transform.hxx>
#include <gp.hxx>
#include <gp_Ax1.hxx>
#include <gp_Ax2.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>
#include <gp_Trsf.hxx>
#include <gp_Vec.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <cmath>
#include <memory>
#include <vector>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::skill::yaw_motor_mount_compound {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.face_id < 0 || in.face_id >= wp.faceCount()) {
        r.add("DFM-INPUT", "error",
              "yaw_motor_mount_compound: face_id out of range");
    }
    if (in.mount_pcd_mm <= 0.0 ||
        in.vent_slot_width_mm <= 0.0 ||
        in.vent_slot_length_mm <= 0.0 ||
        in.vent_slot_depth_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "yaw_motor_mount_compound: all dims must be > 0");
        return r;
    }
    if (in.mount_bolt_count <= 4) {
        r.add("DFM-BOLT-COUNT", "error",
              "yaw_motor_mount_compound: mount_bolt_count " +
              std::to_string(in.mount_bolt_count) +
              " must be > 4");
    }
    if (in.mount_bolt_count != 4 && in.mount_bolt_count != 6 &&
        in.mount_bolt_count != 8) {
        r.add("DFM-MOUNT-PATTERN", "warning",
              "yaw_motor_mount_compound: mount_bolt_count " +
              std::to_string(in.mount_bolt_count) +
              " not in {4, 6, 8} standard NEMA/planetary set");
    }
    if (in.vent_count != 4 && in.vent_count != 6) {
        r.add("DFM-VENT-COUNT", "warning",
              "yaw_motor_mount_compound: vent_count " +
              std::to_string(in.vent_count) +
              " not in {4, 6} typical set");
    }
    const auto* spec = thread_table::findMetric(in.bolt_thread_size_key);
    if (!spec) {
        r.add("DFM-THREAD", "error",
              "yaw_motor_mount_compound: thread key '" +
              in.bolt_thread_size_key + "' not in _iso_thread_table");
    }
    // vent slot length must fit on PCD without overlap
    const double arcPerVent = (M_PI * in.mount_pcd_mm) /
                              static_cast<double>(in.vent_count);
    if (in.vent_slot_length_mm >= arcPerVent) {
        r.add("DFM-VENT-FIT", "error",
              "yaw_motor_mount_compound: vent_slot_length " +
              std::to_string(in.vent_slot_length_mm) +
              " mm >= per-vent arc " + std::to_string(arcPerVent) +
              " mm (adjacent vents overlap)");
    }
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "yaw_motor_mount_compound DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const auto* spec = thread_table::findMetric(in.bolt_thread_size_key);
    const double boltHoleDia = spec->clearance_medium_mm;

    const gp_Pnt faceC = wp.faceCenter(in.face_id);
    const gp_Dir n     = wp.faceNormal(in.face_id);
    const gp_Dir drillDir(-n.X(), -n.Y(), -n.Z());

    constexpr double kOverhang = 0.05;
    const double pcdR = in.mount_pcd_mm / 2.0;

    // Build sequential cuts: M bolt holes + N vent slots, each rotated
    // around the centerline by gp_Trsf::SetRotation.
    const gp_Pnt rotCenter(in.center_x_mm, in.center_y_mm, faceC.Z());
    const gp_Ax1 rotAxis(rotCenter, n);

    TopoDS_Shape current = wp.shape();

    // ── Sub-features 1..M: bolt holes (rotation cuts) ──────────────────
    const double boltR    = boltHoleDia / 2.0;
    const double boltDepth = in.vent_slot_depth_mm + 10.0;

    const gp_Pnt boltEntryTpl(
        in.center_x_mm + pcdR + n.X() * kOverhang,
        in.center_y_mm        + n.Y() * kOverhang,
        faceC.Z()             + n.Z() * kOverhang);
    const gp_Ax2 boltAxTpl(boltEntryTpl, drillDir);
    const TopoDS_Shape boltTemplate = pr::cylinder(
        boltAxTpl, boltR, boltDepth + 2.0 * kOverhang);

    for (int i = 0; i < in.mount_bolt_count; ++i) {
        const double theta = (2.0 * M_PI * static_cast<double>(i)) /
                             static_cast<double>(in.mount_bolt_count);
        gp_Trsf rot;
        rot.SetRotation(rotAxis, theta);
        BRepBuilderAPI_Transform xform(boltTemplate, rot, true);
        if (!xform.IsDone()) {
            throw SkillError(
                "yaw_motor_mount_compound: bolt rotation failed");
        }
        current = pr::cut(current, xform.Shape());
    }

    // ── Sub-features M+1..M+N: vent slots (rotation cuts) ──────────────
    // Vent slot at theta=0 located along +X at PCD, length along tangent
    // (= +Y at theta=0), width along radial (= +X).  We build a box at
    // theta=0 then rotate via gp_Trsf for each i.
    //
    // Box axes: gp_Ax2(origin, drillDir, slotX) → DX along slotX, DY along
    // (drillDir × slotX), DZ along drillDir.  We want DX = slot_length
    // (tangent), DY = slot_width (radial), DZ = slot_depth (axial).
    //
    // At theta=0 the tangent direction (CCW) is +Y; the radial direction is
    // +X.  So slotX = +Y (tangent length).
    gp_Dir slotX(0.0, 1.0, 0.0);  // tangent at theta=0 (CCW)
    // Project onto plane perpendicular to n.
    const double dotXn = slotX.X() * n.X() + slotX.Y() * n.Y() + slotX.Z() * n.Z();
    const gp_Vec slotXVec(slotX.X() - dotXn * n.X(),
                          slotX.Y() - dotXn * n.Y(),
                          slotX.Z() - dotXn * n.Z());
    const gp_Dir slotXFinal(slotXVec);
    const gp_Vec slotYVec = gp_Vec(drillDir).Crossed(gp_Vec(slotXFinal));
    const gp_Dir slotYFinal(slotYVec);

    const double L = in.vent_slot_length_mm;
    const double W = in.vent_slot_width_mm;
    const double D = in.vent_slot_depth_mm + kOverhang;

    // Slot at theta=0: centered at (cx + pcdR, cy, faceC.Z()).
    const gp_Pnt slotCenter(
        in.center_x_mm + pcdR,
        in.center_y_mm,
        faceC.Z());
    const gp_Pnt slot0Origin(
        slotCenter.X() - 0.5 * L * slotXFinal.X() -
                          0.5 * W * slotYFinal.X() + n.X() * kOverhang,
        slotCenter.Y() - 0.5 * L * slotXFinal.Y() -
                          0.5 * W * slotYFinal.Y() + n.Y() * kOverhang,
        slotCenter.Z() - 0.5 * L * slotXFinal.Z() -
                          0.5 * W * slotYFinal.Z() + n.Z() * kOverhang);
    const gp_Ax2 slot0Ax(slot0Origin, drillDir, slotXFinal);
    const TopoDS_Shape ventTemplate = pr::box(slot0Ax, L, W, D);

    for (int j = 0; j < in.vent_count; ++j) {
        const double theta = (2.0 * M_PI * static_cast<double>(j)) /
                             static_cast<double>(in.vent_count);
        gp_Trsf rot;
        rot.SetRotation(rotAxis, theta);
        BRepBuilderAPI_Transform xform(ventTemplate, rot, true);
        if (!xform.IsDone()) {
            throw SkillError(
                "yaw_motor_mount_compound: vent rotation failed");
        }
        current = pr::cut(current, xform.Shape());
    }
    const TopoDS_Shape newShape = current;

    // ── Derived volume ──────────────────────────────────────────────────
    const double vBolts = static_cast<double>(in.mount_bolt_count) *
                          M_PI * boltR * boltR * boltDepth;
    const double vVents = static_cast<double>(in.vent_count) *
                          L * W * in.vent_slot_depth_mm;
    const double removedVol = vBolts + vVents;

    json params = {
        { "face_id",              in.face_id },
        { "center_x_mm",          in.center_x_mm },
        { "center_y_mm",          in.center_y_mm },
        { "mount_pcd_mm",         in.mount_pcd_mm },
        { "mount_bolt_count",     in.mount_bolt_count },
        { "bolt_thread_size_key", in.bolt_thread_size_key },
        { "vent_count",           in.vent_count },
        { "vent_slot_width_mm",   in.vent_slot_width_mm },
        { "vent_slot_length_mm",  in.vent_slot_length_mm },
        { "vent_slot_depth_mm",   in.vent_slot_depth_mm },
    };
    json pattern = {
        { "kind",                       kSkillId },
        { "is_compound",                true },
        { "wind_feature_type",          "yaw_motor_mount" },
        { "mount_bolt_count",           in.mount_bolt_count },
        { "vent_count",                 in.vent_count },
        { "mount_pcd_mm",               in.mount_pcd_mm },
        { "bolt_thread_size_key",       in.bolt_thread_size_key },
        { "vent_slot_width_mm",         in.vent_slot_width_mm },
        { "vent_slot_length_mm",        in.vent_slot_length_mm },
        { "subfeature_count",
              in.mount_bolt_count + in.vent_count },
        { "derived_volume_removed_mm3", removedVol },
        { "standard_ref",               "IEC 61400-1 / NEMA flange" },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "drill;end_mill";
    tooling.tool_dia_mm       = in.vent_slot_width_mm;
    tooling.tool_length_mm    = in.vent_slot_depth_mm + 5.0;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 3;
    tooling.cutting_speed_sfm = 250.0;
    tooling.feed_per_tooth_mm = 0.05;
    tooling.stock_removed_mm3 = removedVol;
    tooling.est_cycle_time_s  = 10.0 * static_cast<double>(in.mount_bolt_count) +
                                15.0 * static_cast<double>(in.vent_count);
    tooling.extra = {
        { "wind_feature_type", "yaw_motor_mount" },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::yaw_motor_mount_compound applied: bolts={} vents={} PCD={}",
                  in.mount_bolt_count, in.vent_count, in.mount_pcd_mm);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition: metadata replay ─────────────────────────────────────────

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
            { "source",            "metadata_replay" },
            { "is_compound",       true },
            { "wind_feature_type", "yaw_motor_mount" },
        };
        out.push_back(r);
    }
    return out;
}

}  // namespace koocadcam::skill::yaw_motor_mount_compound
