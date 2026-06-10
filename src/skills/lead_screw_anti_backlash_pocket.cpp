// @lat: [[engine/skills#lead_screw_anti_backlash_pocket]]

#include "lead_screw_anti_backlash_pocket.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <vector>

namespace koocadcam::skill::lead_screw_anti_backlash_pocket {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

namespace {

constexpr double kAdjScrewPilotDia = 2.5;   // M3 pilot
constexpr double kSlotExtraMm      = 2.0;   // spring slot extends 2 mm past pocket each side
constexpr double kInterPocketWall  = 1.0;   // minimum material between half-nut pockets after slot

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.half_nut_od_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "lead_screw_anti_backlash_pocket: half_nut_od_mm must be > 0");
    }
    if (in.half_nut_l_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "lead_screw_anti_backlash_pocket: half_nut_l_mm must be > 0");
    }
    if (in.spring_slot_w_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "lead_screw_anti_backlash_pocket: spring_slot_w_mm must be > 0");
    }

    if constexpr (kAdjScrewPilotDia < 0.8) {
        r.add("DFM-002", "error",
              "lead_screw_anti_backlash_pocket: adj-screw pilot " +
              std::to_string(kAdjScrewPilotDia) + " mm < 0.8 mm");
    }

    if (in.spring_slot_w_mm > 0.0 &&
        (in.spring_slot_w_mm < 0.3 || in.spring_slot_w_mm > 5.0)) {
        r.add("DFM-AB-SLOT", "warning",
              "lead_screw_anti_backlash_pocket: spring_slot_w_mm " +
              std::to_string(in.spring_slot_w_mm) +
              " outside typical [0.3, 5.0] mm range");
    }

    // The two half-nut pockets straddle the spring slot.  After cutting the
    // slot, the wall between pockets effectively disappears (the slot fills
    // the gap), so the constraint is that pockets remain non-overlapping
    // in the +Z direction even when the slot is present.
    // For slice-1 we just sanity-check pocket+slot+pocket sums.
    if (in.half_nut_l_mm > 0.0 && in.spring_slot_w_mm > 0.0 &&
        2.0 * in.half_nut_l_mm + in.spring_slot_w_mm > 200.0) {
        r.add("DFM-INPUT", "warning",
              "lead_screw_anti_backlash_pocket: combined pocket+slot+pocket > 200 mm; verify stock");
    }
    (void)kInterPocketWall;

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "lead_screw_anti_backlash_pocket DFM failed:";
        for (const auto& f : dfm.findings) msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    auto entryId = wp.resolve(in.entry_face);
    if (!entryId) throw SkillError("lead_screw_anti_backlash_pocket: entry_face datum unresolved");

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);

    constexpr double kOverhang = 0.05;
    const gp_Dir adir = in.axis_dir;

    // Stack layout along axis_dir (downward = -Z for default):
    //   z = topZ             → start
    //   z -= half_nut_l_mm   → end of upper half-nut pocket
    //   z -= spring_slot_w   → end of spring slot
    //   z -= half_nut_l_mm   → end of lower half-nut pocket
    //
    // For axis = -Z (downward), entry is at zMax.

    const double topZ = (adir.Z() < 0) ? zMax : zMin;

    // ── Sub-feature 1: upper half-nut pocket ──────────────────────────
    const double upperStartZ = topZ + kOverhang * ((adir.Z() < 0) ? 1.0 : -1.0);
    const gp_Pnt upperStart(in.position_x_mm, in.position_y_mm, upperStartZ);
    const gp_Ax2 upperAx(upperStart, adir);
    const double upperLen = in.half_nut_l_mm + kOverhang;
    const TopoDS_Shape upperTool =
        pr::cylinder(upperAx, in.half_nut_od_mm / 2.0, upperLen);

    // ── Sub-feature 2: spring relief slot (rectangular thin slab) ─────
    // The slot is a thin box that fully spans the pocket diameter +
    // kSlotExtraMm on each side, centered between the two half-nut pockets.
    const double upperEndZ = (adir.Z() < 0)
        ? upperStartZ - in.half_nut_l_mm
        : upperStartZ + in.half_nut_l_mm;
    const double slotMidZ = (adir.Z() < 0)
        ? upperEndZ - in.spring_slot_w_mm / 2.0
        : upperEndZ + in.spring_slot_w_mm / 2.0;
    const double slotWidth = in.half_nut_od_mm + 2.0 * kSlotExtraMm;
    // Box origin: corner at lower-left, axes: Z = +Z global (slot thickness),
    // X = +X global (slot length), Y = +Y global (slot depth).
    const gp_Pnt slotOrigin(
        in.position_x_mm - slotWidth / 2.0,
        in.position_y_mm - slotWidth / 2.0,
        slotMidZ - in.spring_slot_w_mm / 2.0);
    const gp_Ax2 slotAx(slotOrigin, gp_Dir(0, 0, 1), gp_Dir(1, 0, 0));
    const TopoDS_Shape slotTool = pr::box(slotAx,
                                          slotWidth,
                                          slotWidth,
                                          in.spring_slot_w_mm);

    // ── Sub-feature 3: lower half-nut pocket ──────────────────────────
    const double lowerStartZ = (adir.Z() < 0)
        ? upperEndZ - in.spring_slot_w_mm + kOverhang
        : upperEndZ + in.spring_slot_w_mm - kOverhang;
    const gp_Pnt lowerStart(in.position_x_mm, in.position_y_mm, lowerStartZ);
    const gp_Ax2 lowerAx(lowerStart, adir);
    const TopoDS_Shape lowerTool =
        pr::cylinder(lowerAx, in.half_nut_od_mm / 2.0, in.half_nut_l_mm + kOverhang);

    // ── Sub-feature 4: adjustment-screw pilot (transverse to nut axis) ─
    // The adj-screw enters from +X side perpendicular to the nut axis at
    // the spring slot mid-height; depth = 1.2 × half_nut_od_mm so it
    // crosses the centerline.
    const double adjDepth = in.half_nut_od_mm * 1.2;
    // Start the cylinder outside the +X bbox boundary, axis = -X.
    const gp_Pnt adjStart(xMax + kOverhang, in.position_y_mm, slotMidZ);
    const gp_Ax2 adjAx(adjStart, gp_Dir(-1, 0, 0));
    const TopoDS_Shape adjTool =
        pr::cylinder(adjAx, kAdjScrewPilotDia / 2.0, adjDepth + kOverhang);

    // Two concentric half-nut pockets along axis_dir don't overlap each
    // other (the slot sits between them).  We still bundle everything
    // through cutMany for performance.
    const std::vector<TopoDS_Shape> allTools = {
        upperTool, slotTool, lowerTool, adjTool
    };
    const TopoDS_Shape newShape = pr::cutMany(wp.shape(), allTools);

    constexpr int kSubFeatures = 4;

    json params = {
        { "entry_face_id",      *entryId },
        { "position_x_mm",      in.position_x_mm },
        { "position_y_mm",      in.position_y_mm },
        { "axis_dir",           { adir.X(), adir.Y(), adir.Z() } },
        { "half_nut_od_mm",     in.half_nut_od_mm },
        { "half_nut_l_mm",      in.half_nut_l_mm },
        { "spring_slot_w_mm",   in.spring_slot_w_mm },
    };
    json pattern = {
        { "kind",                kSkillId },
        { "is_compound",         true },
        { "subfeature_count",    kSubFeatures },
        { "half_nut_count",      2 },
        { "spring_slot_count",   1 },
        { "adj_screw_count",     1 },
        { "half_nut_od_mm",      in.half_nut_od_mm },
        { "half_nut_l_mm",       in.half_nut_l_mm },
        { "spring_slot_w_mm",    in.spring_slot_w_mm },
        { "spring_slot_width_mm", in.half_nut_od_mm + 2.0 * kSlotExtraMm },
        { "adj_screw_pilot_dia_mm", kAdjScrewPilotDia },
        { "adj_screw_depth_mm",  adjDepth },
        // The full stack height along axis (this is also the test invariant):
        { "stack_height_mm",     2.0 * in.half_nut_l_mm + in.spring_slot_w_mm },
        { "axis_dir",            { adir.X(), adir.Y(), adir.Z() } },
    };
    ToolingMeta tooling;
    tooling.tool_type        = "boring_bar;end_mill;drill";
    tooling.tool_dia_mm      = in.half_nut_od_mm;
    tooling.tool_length_mm   = in.half_nut_l_mm * 1.5 + 5.0;
    tooling.tool_material    = "carbide";
    tooling.flute_count      = 2;
    tooling.cutting_speed_sfm = 400.0;
    tooling.feed_per_tooth_mm = 0.04;
    const double pocketVol = 2.0 * M_PI * (in.half_nut_od_mm/2.0)
                                       * (in.half_nut_od_mm/2.0) * in.half_nut_l_mm;
    const double slotVol   = (in.half_nut_od_mm + 2.0 * kSlotExtraMm)
                           * (in.half_nut_od_mm + 2.0 * kSlotExtraMm)
                           * in.spring_slot_w_mm;
    const double adjVol    = M_PI * (kAdjScrewPilotDia/2.0)*(kAdjScrewPilotDia/2.0) * adjDepth;
    tooling.stock_removed_mm3 = pocketVol + slotVol + adjVol;
    tooling.est_cycle_time_s  = std::max(5.0,
        2.0 * in.half_nut_l_mm * 0.4 + in.spring_slot_w_mm * 0.5 + 2.0);
    tooling.extra = {
        { "tool_sequence", {
            { { "tool_type", "boring_bar" },
              { "purpose",   "upper half-nut pocket" },
              { "tool_dia_mm", in.half_nut_od_mm },
              { "depth_mm",    in.half_nut_l_mm } },
            { { "tool_type", "end_mill" },
              { "purpose",   "spring relief slot" },
              { "tool_dia_mm", std::max(1.0, in.spring_slot_w_mm) },
              { "depth_mm",    in.half_nut_od_mm + 2.0 * kSlotExtraMm } },
            { { "tool_type", "boring_bar" },
              { "purpose",   "lower half-nut pocket" },
              { "tool_dia_mm", in.half_nut_od_mm },
              { "depth_mm",    in.half_nut_l_mm } },
            { { "tool_type", "drill" },
              { "purpose",   "adj-screw pilot (M3)" },
              { "tool_dia_mm", kAdjScrewPilotDia },
              { "depth_mm",    adjDepth } },
        } },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);  // keep full chain history
    wpNew->addFeature(sig);

    spdlog::debug("skill::lead_screw_anti_backlash_pocket applied: od={} l={} slot={}",
                  in.half_nut_od_mm, in.half_nut_l_mm, in.spring_slot_w_mm);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition (metadata replay) ────────────────────────────────────────

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
            { "subfeature_count",  f.pattern.value("subfeature_count", 0) },
        };
        out.push_back(r);
    }
    return out;
}

}  // namespace koocadcam::skill::lead_screw_anti_backlash_pocket
