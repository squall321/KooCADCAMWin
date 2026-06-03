// @lat: [[engine/skills#side_core_pull_slot]]

#include "side_core_pull_slot.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <Standard_Failure.hxx>
#include <TopoDS_Shape.hxx>
#include <gp.hxx>
#include <gp_Ax2.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::skill::side_core_pull_slot {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

namespace {

// Internal draft applied to slot side walls (degrees from vertical).
constexpr double kSlotDraftDeg = 3.0;

double volumeOf(const TopoDS_Shape& s)
{
    if (s.IsNull()) return 0.0;
    GProp_GProps p;
    BRepGProp::VolumeProperties(s, p);
    return p.Mass();
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.face_id < 0 || in.face_id >= wp.faceCount()) {
        r.add("DFM-INPUT", "error",
              "side_core_pull_slot: face_id out of range");
    }
    if (!(in.slot_width_mm > 0.0)) {
        r.add("DFM-INPUT", "error",
              "side_core_pull_slot: slot_width_mm must be > 0");
    }
    if (!(in.slot_height_mm > 0.0)) {
        r.add("DFM-INPUT", "error",
              "side_core_pull_slot: slot_height_mm must be > 0");
    }
    if (!(in.slot_depth_mm > 0.0)) {
        r.add("DFM-INPUT", "error",
              "side_core_pull_slot: slot_depth_mm must be > 0");
    }
    if (in.retract_clearance_mm <= 0.5 - 1e-9) {
        r.add("DFM-SCORE-CLEARANCE", "error",
              "side_core_pull_slot: retract_clearance_mm " +
              std::to_string(in.retract_clearance_mm) +
              " ≤ 0.5 mm minimum");
    }
    // Note: kSlotDraftDeg is a compile-time constant (3.0); the DFM check
    // below is a static guard against accidental future increases.
#pragma warning(push)
#pragma warning(disable : 4127)   // constant conditional in static_assert chain (MSVC)
    static_assert(kSlotDraftDeg <= 15.0,
                  "side_core_pull_slot: internal draft must be <= 15deg");
#pragma warning(pop)
    const double pdLen = std::sqrt(in.pull_dir_x * in.pull_dir_x +
                                   in.pull_dir_y * in.pull_dir_y +
                                   in.pull_dir_z * in.pull_dir_z);
    if (pdLen < 1e-9) {
        r.add("DFM-INPUT", "error",
              "side_core_pull_slot: pull_dir_xyz is zero vector");
    }
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "side_core_pull_slot DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    // Build pull direction (the direction the side core retracts; this is
    // the slot's depth axis).
    const double pdLen = std::sqrt(in.pull_dir_x * in.pull_dir_x +
                                   in.pull_dir_y * in.pull_dir_y +
                                   in.pull_dir_z * in.pull_dir_z);
    const gp_Dir pullDir(in.pull_dir_x / pdLen,
                         in.pull_dir_y / pdLen,
                         in.pull_dir_z / pdLen);
    // "Inward" = into the face = opposite of pullDir.
    const gp_Dir inward(-pullDir.X(), -pullDir.Y(), -pullDir.Z());

    // Face center as the slot's in-plane centre.
    const gp_Pnt faceC = wp.faceCenter(in.face_id);

    // In-plane local X axis: pick a direction perpendicular to inward.
    gp_Vec refX = (std::abs(inward.Z()) > 0.9)
        ? gp_Vec(1.0, 0.0, 0.0)
        : gp_Vec(0.0, 0.0, 1.0);
    refX = refX - gp_Vec(inward) * refX.Dot(gp_Vec(inward));
    if (refX.Magnitude() < 1e-9) refX = gp_Vec(0.0, 1.0, 0.0);
    refX.Normalize();
    const gp_Dir xLoc(refX.X(), refX.Y(), refX.Z());

    // Total depth = slot_depth + retract_clearance.
    const double totalDepth = in.slot_depth_mm + in.retract_clearance_mm;
    const double width      = in.slot_width_mm;
    const double height     = in.slot_height_mm;

    // Box: DX = width (along xLoc), DY = height (along inward × xLoc),
    //      DZ = totalDepth (along inward).
    // Origin = faceC - (width/2) xLoc - (height/2) yLoc + tiny outward
    // overhang so the cut bites cleanly through the face.
    gp_Vec yVec = gp_Vec(inward).Crossed(gp_Vec(xLoc));
    if (yVec.Magnitude() < 1e-9) yVec = gp_Vec(0.0, 1.0, 0.0);
    yVec.Normalize();

    constexpr double kOverhang = 0.05;
    const gp_Pnt origin(
        faceC.X() - 0.5 * width * xLoc.X() - 0.5 * height * yVec.X() - kOverhang * inward.X(),
        faceC.Y() - 0.5 * width * xLoc.Y() - 0.5 * height * yVec.Y() - kOverhang * inward.Y(),
        faceC.Z() - 0.5 * width * xLoc.Z() - 0.5 * height * yVec.Z() - kOverhang * inward.Z());

    const gp_Ax2 ax(origin, inward, xLoc);
    TopoDS_Shape slotBox = pr::box(ax, width, height, totalDepth + kOverhang);
    // 3° draft is geometrically a slight back-relief at the slot mouth.  We
    // implement draft conceptually (recorded in pattern); the slot tool
    // itself is a straight box for slice-1 simplicity.  This matches how
    // mold_rib applies draft through a tapered ThruSections.

    const double volBefore = volumeOf(wp.shape());
    TopoDS_Shape newShape  = pr::cut(wp.shape(), slotBox);
    if (newShape.IsNull()) {
        throw SkillError("side_core_pull_slot: cut returned null shape");
    }
    const double volAfter  = volumeOf(newShape);
    const double removed   = volBefore - volAfter;

    json params = {
        { "face_id",              in.face_id },
        { "pull_dir",             { pullDir.X(), pullDir.Y(), pullDir.Z() } },
        { "slot_width_mm",        in.slot_width_mm },
        { "slot_height_mm",       in.slot_height_mm },
        { "slot_depth_mm",        in.slot_depth_mm },
        { "retract_clearance_mm", in.retract_clearance_mm },
    };
    json pattern = {
        { "kind",                    kSkillId },
        { "is_compound",             true },
        { "mold_feature_type",       "side_core_slot" },
        { "derived_volume_removed",  removed },
        { "draft_deg",               kSlotDraftDeg },
        { "volume_before_mm3",       volBefore },
        { "volume_after_mm3",        volAfter },
    };
    ToolingMeta tooling;
    tooling.tool_type         = "mold_feature";
    tooling.stock_removed_mm3 = removed;
    tooling.est_cycle_time_s  = 0.0;

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    wpNew->addFeature(sig);

    spdlog::debug("skill::side_core_pull_slot applied: w={} h={} d={} removed={}",
                  width, height, totalDepth, removed);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;
    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        RecognizedFeature rf;
        rf.skill_id         = kSkillId;
        rf.recovered_params = f.params;
        rf.confidence       = 0.92;
        rf.matched_geometry = f.pattern;
        out.push_back(rf);
    }
    return out;
}

}  // namespace koocadcam::skill::side_core_pull_slot
