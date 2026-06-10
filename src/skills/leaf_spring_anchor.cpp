// @lat: [[engine/skills#leaf_spring_anchor]]

#include "leaf_spring_anchor.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::skill::leaf_spring_anchor {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Design constants ─────────────────────────────────────────────────────

namespace {

constexpr double kLipOverhang   = 1.0;     // mm — overhang per side
constexpr double kLipBelowEntry = 1.0;     // mm — lip top sits 1 mm below entry
constexpr double kLipHeight     = 1.0;     // mm — undercut box vertical extent
constexpr double kChamferSize   = 0.5;     // mm — 45° entry chamfer

constexpr double kMinSlotWidth  = 4.0;
constexpr double kMinDepth      = 4.0;

}  // namespace

// ── DFM gates ────────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.slot_length_mm <= 0.0 || in.slot_width_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "leaf_spring_anchor: slot_length_mm and slot_width_mm must be > 0");
        return r;
    }
    if (in.depth_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "leaf_spring_anchor: depth_mm must be > 0");
    }
    if (in.slot_width_mm < kMinSlotWidth) {
        r.add("DFM-ANCHOR-LIP", "error",
              "leaf_spring_anchor: slot_width " +
              std::to_string(in.slot_width_mm) +
              " mm < required " + std::to_string(kMinSlotWidth) +
              " mm (need clearance for 1 mm lips + spring travel)");
    }
    if (in.depth_mm < kMinDepth) {
        r.add("DFM-ANCHOR-DEPTH", "error",
              "leaf_spring_anchor: depth " +
              std::to_string(in.depth_mm) +
              " mm < required " + std::to_string(kMinDepth) +
              " mm (chamfer + lip + working travel)");
    }

    // Slot footprint must be inside workpiece bbox.
    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double halfL = in.slot_length_mm / 2.0;
    const double halfW = in.slot_width_mm  / 2.0;
    // The undercut overhangs by kLipOverhang per side → check that the
    // underwide box fits inside the workpiece.
    const double underHalfW = halfW + kLipOverhang;
    if (in.position_x_mm - halfL < xMin || in.position_x_mm + halfL > xMax ||
        in.position_y_mm - underHalfW < yMin ||
        in.position_y_mm + underHalfW > yMax) {
        r.add("DFM-ANCHOR-LIP", "error",
              "leaf_spring_anchor: undercut footprint exceeds workpiece bbox");
    }
    if (zMax - zMin < in.depth_mm + 1.0) {
        r.add("DFM-ANCHOR-DEPTH", "error",
              "leaf_spring_anchor: workpiece thickness < depth + 1 mm");
    }
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "leaf_spring_anchor DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double zTop = zMax;
    constexpr double kOverhang = 0.05;

    const double halfL = in.slot_length_mm / 2.0;
    const double halfW = in.slot_width_mm  / 2.0;
    const double underHalfW = halfW + kLipOverhang;

    // Sub-feature 1: main slot pocket (full depth, slot_length × slot_width)
    const gp_Pnt slotOrigin(
        in.position_x_mm - halfL,
        in.position_y_mm - halfW,
        zTop - in.depth_mm);
    const TopoDS_Shape slotTool = pr::box(
        gp_Ax2(slotOrigin, gp::DZ()),
        in.slot_length_mm, in.slot_width_mm, in.depth_mm + kOverhang);

    // Sub-feature 2: undercut box (slot_width + 2 × overhang in Y),
    // starts at zTop − (kLipBelowEntry + kLipHeight), extends down by
    // kLipHeight.  The 1 mm overhang on each side creates the retention lip.
    const gp_Pnt undercutOrigin(
        in.position_x_mm - halfL,
        in.position_y_mm - underHalfW,
        zTop - kLipBelowEntry - kLipHeight);
    const TopoDS_Shape undercutTool = pr::box(
        gp_Ax2(undercutOrigin, gp::DZ()),
        in.slot_length_mm, in.slot_width_mm + 2.0 * kLipOverhang,
        kLipHeight);

    // Sub-feature 3: chamfer entry ring (cone frustum: outer dia at top,
    // inner dia at top - chamferSize).  Model with two boxes joined as a
    // bevel ring on the slot perimeter.
    // Simplest: a slightly oversized box only at the top kChamferSize mm.
    const gp_Pnt chamferOrigin(
        in.position_x_mm - halfL - kChamferSize,
        in.position_y_mm - halfW - kChamferSize,
        zTop - kChamferSize);
    const TopoDS_Shape chamferTool = pr::box(
        gp_Ax2(chamferOrigin, gp::DZ()),
        in.slot_length_mm + 2.0 * kChamferSize,
        in.slot_width_mm  + 2.0 * kChamferSize,
        kChamferSize + kOverhang);

    const TopoDS_Shape f1 = pr::fuse(slotTool, undercutTool);
    const TopoDS_Shape fusedCutter = pr::fuse(f1, chamferTool);
    const TopoDS_Shape newShape    = pr::cut(wp.shape(), fusedCutter);

    // ── Signature ──────────────────────────────────────────────────────
    json params = {
        { "position_x_mm",   in.position_x_mm },
        { "position_y_mm",   in.position_y_mm },
        { "axis_dir",        { in.axis_dir.X(), in.axis_dir.Y(), in.axis_dir.Z() } },
        { "slot_length_mm",  in.slot_length_mm },
        { "slot_width_mm",   in.slot_width_mm },
        { "depth_mm",        in.depth_mm },
    };
    json pattern = {
        { "kind",                kSkillId },
        { "is_compound",         true },
        { "subfeature_count",    3 },
        { "slot_length_mm",      in.slot_length_mm },
        { "slot_width_mm",       in.slot_width_mm },
        { "depth_mm",            in.depth_mm },
        { "lip_overhang_mm",     kLipOverhang },
        { "lip_below_entry_mm",  kLipBelowEntry },
        { "lip_height_mm",       kLipHeight },
        { "chamfer_size_mm",     kChamferSize },
        { "undercut_width_mm",   in.slot_width_mm + 2.0 * kLipOverhang },
        { "axis_dir",            { in.axis_dir.X(), in.axis_dir.Y(), in.axis_dir.Z() } },
    };

    ToolingMeta tooling;
    tooling.tool_type   = "end_mill;t_slot_cutter;chamfer_mill";
    tooling.tool_dia_mm = in.slot_width_mm;
    tooling.tool_material = "carbide";
    const double slotV     = in.slot_length_mm * in.slot_width_mm * in.depth_mm;
    const double undercutExtra = in.slot_length_mm * (2.0 * kLipOverhang) * kLipHeight;
    const double chamferExtra  = (in.slot_length_mm + in.slot_width_mm) *
                                 2.0 * kChamferSize * kChamferSize;
    tooling.stock_removed_mm3 = slotV + undercutExtra + chamferExtra;
    tooling.est_cycle_time_s  = std::max(2.0, slotV / 200.0);
    tooling.extra = {
        { "tool_sequence", json::array({
            { { "tool_type", "end_mill" }, { "purpose", "main_slot" },
              { "tool_dia_mm", in.slot_width_mm }, { "depth_mm", in.depth_mm } },
            { { "tool_type", "t_slot_cutter" }, { "purpose", "retention_undercut" },
              { "overhang_mm", kLipOverhang }, { "depth_mm", kLipHeight } },
            { { "tool_type", "chamfer_mill" }, { "purpose", "entry_chamfer" },
              { "chamfer_size_mm", kChamferSize } },
        }) },
        { "standard", "leaf_spring_anchor_45deg_entry_1mm_lip" },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };
    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);  // keep full chain history
    wpNew->addFeature(sig);

    spdlog::debug("skill::leaf_spring_anchor applied: slot {}x{}x{} + undercut + chamfer",
                  in.slot_length_mm, in.slot_width_mm, in.depth_mm);

    return SkillOutput{ wpNew, sig };
}

// ── Recognize: metadata replay ──────────────────────────────────────────

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;
    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        RecognizedFeature r;
        r.skill_id         = kSkillId;
        r.recovered_params = f.params;
        r.confidence       = 1.0;
        r.matched_geometry = { { "source", "metadata_replay" } };
        out.push_back(r);
    }
    return out;
}

}  // namespace koocadcam::skill::leaf_spring_anchor
