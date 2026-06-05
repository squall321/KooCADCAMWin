// @lat: [[engine/skills#generator_stator_slot_pattern]]

#include "generator_stator_slot_pattern.hpp"

#include "Workpiece.hpp"
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

namespace koocadcam::skill::generator_stator_slot_pattern {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.face_id < 0 || in.face_id >= wp.faceCount()) {
        r.add("DFM-INPUT", "error",
              "generator_stator_slot_pattern: face_id out of range");
    }
    if (in.stator_inner_dia_mm <= 0.0 ||
        in.stator_outer_dia_mm <= 0.0 ||
        in.slot_width_mm <= 0.0 ||
        in.slot_depth_mm <= 0.0 ||
        in.slot_opening_width_mm <= 0.0 ||
        in.stator_thickness_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "generator_stator_slot_pattern: all dims must be > 0");
        return r;
    }
    if (in.stator_outer_dia_mm <= in.stator_inner_dia_mm) {
        r.add("DFM-STATOR-GEOM", "error",
              "generator_stator_slot_pattern: outer_dia must be > inner_dia");
    }
    if (in.slot_count <= 4) {
        r.add("DFM-SLOT-COUNT", "error",
              "generator_stator_slot_pattern: slot_count " +
              std::to_string(in.slot_count) +
              " must be > 4");
    }
    if (in.slot_count != 36 && in.slot_count != 48 && in.slot_count != 72) {
        r.add("DFM-IEC-60034", "warning",
              "generator_stator_slot_pattern: slot_count " +
              std::to_string(in.slot_count) +
              " not in IEC 60034 utility set {36, 48, 72}");
    }
    // Physical-fit constraint: slot_width × slot_count < π × stator_inner_dia
    // (slots must fit on inner bore circumference without overlap).
    const double allowed = M_PI * in.stator_inner_dia_mm /
                           static_cast<double>(in.slot_count);
    if (in.slot_width_mm >= allowed) {
        r.add("DFM-SLOT-FIT", "error",
              "generator_stator_slot_pattern: slot_width " +
              std::to_string(in.slot_width_mm) +
              " mm >= π·stator_inner_dia/slot_count " +
              std::to_string(allowed) +
              " mm (slots overlap — teeth impossible)");
    }
    if (in.slot_opening_width_mm >= in.slot_width_mm) {
        r.add("DFM-SLOT-OPEN", "error",
              "generator_stator_slot_pattern: opening_width must be "
              "< slot_width (else slot is not closed-back)");
    }
    // Slot depth must fit in stator wall thickness.
    const double wallThk = (in.stator_outer_dia_mm - in.stator_inner_dia_mm) / 2.0;
    if (in.slot_depth_mm >= wallThk - 2.0) {
        r.add("DFM-SLOT-DEPTH", "error",
              "generator_stator_slot_pattern: slot_depth " +
              std::to_string(in.slot_depth_mm) +
              " mm >= wall thickness " + std::to_string(wallThk) +
              " mm − 2 mm yoke (no iron yoke left for flux)");
    }
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "generator_stator_slot_pattern DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const gp_Pnt faceC = wp.faceCenter(in.face_id);
    const gp_Dir n     = wp.faceNormal(in.face_id);
    const gp_Dir drillDir(-n.X(), -n.Y(), -n.Z());

    constexpr double kOverhang = 0.05;
    const double innerR = in.stator_inner_dia_mm / 2.0;

    // Slot template at theta=0: a radial box starting at the inner bore,
    // extending radially outward by slot_depth, axial length = stator
    // thickness, tangential width = slot_width.
    //
    // gp_Ax2(origin, MainDir, XDir): DX along XDir, DY along (MainDir×XDir),
    // DZ along MainDir.  We want:
    //   DX = slot_depth (radial outward, i.e. +X at theta=0)
    //   DY = slot_width (tangential, +Y at theta=0)
    //   DZ = stator_thickness (axial, drillDir = into the body)
    //
    // → MainDir = drillDir, XDir = radial outward (+X at theta=0)
    gp_Dir radialOut(1.0, 0.0, 0.0);
    // Project onto plane perpendicular to n.
    const double dotXn = radialOut.X() * n.X() + radialOut.Y() * n.Y() + radialOut.Z() * n.Z();
    const gp_Vec radialVec(radialOut.X() - dotXn * n.X(),
                           radialOut.Y() - dotXn * n.Y(),
                           radialOut.Z() - dotXn * n.Z());
    const gp_Dir radialFinal(radialVec);
    const gp_Vec tangentVec = gp_Vec(drillDir).Crossed(gp_Vec(radialFinal));
    const gp_Dir tangentFinal(tangentVec);

    // Slot template position: starts at (cx + innerR, cy, faceC.Z()).
    // Origin: shift by -W/2 along tangent so slot is centered tangentially.
    const double L = in.slot_depth_mm + kOverhang;     // radial extent
    const double W = in.slot_width_mm;                  // tangential extent
    const double D = in.stator_thickness_mm + 2.0 * kOverhang;  // axial extent

    const gp_Pnt slot0Origin(
        in.center_x_mm + innerR * radialFinal.X()
                       - 0.5 * W * tangentFinal.X()
                       + n.X() * kOverhang,
        in.center_y_mm + innerR * radialFinal.Y()
                       - 0.5 * W * tangentFinal.Y()
                       + n.Y() * kOverhang,
        faceC.Z()      + innerR * radialFinal.Z()
                       - 0.5 * W * tangentFinal.Z()
                       + n.Z() * kOverhang);
    const gp_Ax2 slot0Ax(slot0Origin, drillDir, radialFinal);
    const TopoDS_Shape slotTemplate = pr::box(slot0Ax, D, L, W);

    // Rotation axis = face normal at stator center.
    const gp_Pnt rotCenter(in.center_x_mm, in.center_y_mm, faceC.Z());
    const gp_Ax1 rotAxis(rotCenter, n);

    TopoDS_Shape current = wp.shape();
    for (int i = 0; i < in.slot_count; ++i) {
        const double theta = (2.0 * M_PI * static_cast<double>(i)) /
                             static_cast<double>(in.slot_count);
        gp_Trsf rot;
        rot.SetRotation(rotAxis, theta);
        BRepBuilderAPI_Transform xform(slotTemplate, rot, true);
        if (!xform.IsDone()) {
            throw SkillError(
                "generator_stator_slot_pattern: slot rotation failed");
        }
        // Sequential pr::cut against running workpiece.
        current = pr::cut(current, xform.Shape());
    }
    const TopoDS_Shape newShape = current;

    // Derived volume.
    const double vSlot = in.slot_width_mm * in.slot_depth_mm *
                         in.stator_thickness_mm;
    const double removedVol = static_cast<double>(in.slot_count) * vSlot;

    json params = {
        { "face_id",                in.face_id },
        { "center_x_mm",            in.center_x_mm },
        { "center_y_mm",            in.center_y_mm },
        { "stator_inner_dia_mm",    in.stator_inner_dia_mm },
        { "stator_outer_dia_mm",    in.stator_outer_dia_mm },
        { "slot_count",             in.slot_count },
        { "slot_width_mm",          in.slot_width_mm },
        { "slot_depth_mm",          in.slot_depth_mm },
        { "slot_opening_width_mm",  in.slot_opening_width_mm },
        { "stator_thickness_mm",    in.stator_thickness_mm },
    };
    json pattern = {
        { "kind",                       kSkillId },
        { "is_compound",                true },
        { "wind_feature_type",          "generator_stator_slot_pattern" },
        { "slot_count",                 in.slot_count },
        { "stator_inner_dia_mm",        in.stator_inner_dia_mm },
        { "slot_width_mm",              in.slot_width_mm },
        { "slot_depth_mm",              in.slot_depth_mm },
        { "slot_opening_width_mm",      in.slot_opening_width_mm },
        { "subfeature_count",           in.slot_count },
        { "derived_volume_removed_mm3", removedVol },
        { "standard_ref",               "IEC 60034" },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "end_mill;broach";
    tooling.tool_dia_mm       = in.slot_width_mm;
    tooling.tool_length_mm    = in.stator_thickness_mm + 5.0;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 4;
    tooling.cutting_speed_sfm = 220.0;
    tooling.feed_per_tooth_mm = 0.04;
    tooling.stock_removed_mm3 = removedVol;
    tooling.est_cycle_time_s  = 8.0 * static_cast<double>(in.slot_count);
    tooling.extra = {
        { "wind_feature_type", "generator_stator_slot_pattern" },
        { "lamination_family", "3-phase wind generator stator" },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::generator_stator_slot_pattern applied: N={} ID={} width={}",
                  in.slot_count, in.stator_inner_dia_mm, in.slot_width_mm);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ─────────────────────────────────────────────────────────
//
// Metadata replay (confidence 1.0) + light geometric path that counts the
// number of slot-shaped pockets concentric with the stator centerline.

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;

    // Metadata replay — exact recovery.
    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        RecognizedFeature r;
        r.skill_id         = kSkillId;
        r.recovered_params = f.params;
        r.confidence       = 1.0;
        r.matched_geometry = {
            { "source",            "metadata_replay" },
            { "is_compound",       true },
            { "wind_feature_type", "generator_stator_slot_pattern" },
        };
        out.push_back(r);
    }

    // Light geometric path: count planar faces whose normals are
    // tangential (perpendicular to +Z) — slot walls.  Group by edge count
    // bucket of ~ slot_count*4.  This is a coarse fingerprint, not exact.
    int planarTangentialCount = 0;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFacePlanar(i)) continue;
        const auto fn = wp.faceNormal(i);
        if (std::abs(fn.Z()) < 0.2) ++planarTangentialCount;
    }
    if (planarTangentialCount >= 16) {
        // Each slot contributes ~2 tangential walls → estimate count.
        const int estSlots = planarTangentialCount / 2;
        json rec = {
            { "slot_count_estimate", estSlots },
        };
        json matched = {
            { "source",                   "geometric_fingerprint" },
            { "planar_tangential_faces",  planarTangentialCount },
        };
        out.push_back(RecognizedFeature{
            kSkillId, rec, 0.55, matched });
    }

    return out;
}

}  // namespace koocadcam::skill::generator_stator_slot_pattern
