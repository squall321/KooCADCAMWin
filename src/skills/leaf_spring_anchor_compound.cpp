// @lat: [[engine/skills#leaf_spring_anchor_compound]]

#include "leaf_spring_anchor_compound.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <gp.hxx>
#include <gp_Ax2.hxx>
#include <gp_Pnt.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <memory>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::skill::leaf_spring_anchor_compound {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── ISO 273 clearance-hole table ─────────────────────────────────────────

namespace {

struct ScrewPilot { const char* size; double pilot_mm; };
constexpr std::array<ScrewPilot, 5> kScrewTable {{
    { "M3", 3.4 },
    { "M4", 4.5 },
    { "M5", 5.5 },
    { "M6", 6.6 },
    { "M8", 9.0 },
}};

}  // namespace

double pilotForScrew(const std::string& screw_M)
{
    for (const auto& e : kScrewTable)
        if (screw_M == e.size) return e.pilot_mm;
    return 0.0;
}

// ── Validation ───────────────────────────────────────────────────────────
//
// SAE J788 leaf-spring mounting bracket DFM gates.

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.slot_length_mm <= 0.0 || in.slot_width_mm <= 0.0
        || in.slot_depth_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "leaf_spring_anchor_compound: slot_length, slot_width, "
              "slot_depth must all be > 0");
        return r;
    }

    if (in.slot_length_mm < 8.0 || in.slot_width_mm < 8.0) {
        r.add("DFM-LEAF-SIZE", "error",
              "leaf_spring_anchor_compound: slot_length/width < 8 mm — "
              "below SAE J788 minimum leaf-spring stock");
    }
    if (in.slot_depth_mm < in.slot_width_mm / 2.0) {
        r.add("DFM-LEAF-DEPTH", "error",
              "leaf_spring_anchor_compound: slot_depth " +
              std::to_string(in.slot_depth_mm) +
              " < slot_width/2 (" + std::to_string(in.slot_width_mm / 2.0) +
              ") — insufficient leaf engagement");
    }

    const double pilot = pilotForScrew(in.screw_M);
    if (pilot <= 0.0) {
        r.add("DFM-LEAF-SCREW", "error",
              "leaf_spring_anchor_compound: unknown screw_M '" + in.screw_M +
              "' (supported: M3/M4/M5/M6/M8)");
    } else if (pilot < 0.8) {
        r.add("DFM-002", "error",
              "leaf_spring_anchor_compound: derived pilot " +
              std::to_string(pilot) + " mm < min drill 0.8 mm");
    }

    // Screw spacing requires slot length sufficient for two pilots.
    const double spacing = in.slot_length_mm * 0.5;
    if (pilot > 0.0 && spacing < pilot * 3.0) {
        r.add("DFM-LEAF-SPACING", "error",
              "leaf_spring_anchor_compound: screw spacing " +
              std::to_string(spacing) +
              " < 3 × pilot — screws too close to walls");
    }

    if (!wp.shape().IsNull()) {
        double xMin, yMin, zMin, xMax, yMax, zMax;
        wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
        const double thickness = zMax - zMin;
        // Need slot_depth + 2 mm wall + screw pilot depth (~ 2 × pilot).
        const double minThk = in.slot_depth_mm + 2.0 + 2.0 * std::max(pilot, 4.0);
        if (thickness < minThk) {
            r.add("DFM-LEAF-WALL", "warning",
                  "leaf_spring_anchor_compound: stock thickness " +
                  std::to_string(thickness) + " < " +
                  std::to_string(minThk) +
                  " mm — pilots may exit far face");
        }
    }

    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────
//
// Sub-feature chain:
//   1. Rectangular slot box (length × width × depth)
//   2. Undercut step (slot_length × (width + 2) × 1 mm at slot floor)
//   3. Screw pilot 1 (offset by +length/4 along X)
//   4. Screw pilot 2 (offset by -length/4 along X)

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "leaf_spring_anchor_compound DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    auto faceId = wp.resolve(in.face_id);
    if (!faceId) throw SkillError(
        "leaf_spring_anchor_compound: face_id datum unresolved");

    const double pilot = pilotForScrew(in.screw_M);

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    constexpr double kOverhang = 0.05;
    constexpr double kUndercutOverhang = 1.0;        // 1 mm retention overhang

    const double topZ = zMax;
    const double slotBottomZ = topZ - in.slot_depth_mm;

    // Sub-feature 1: main rectangular slot.
    const gp_Pnt slotOrigin(
        in.position_x_mm - in.slot_length_mm / 2.0,
        in.position_y_mm - in.slot_width_mm  / 2.0,
        slotBottomZ);
    const gp_Ax2 slotAx(slotOrigin, gp::DZ());
    const TopoDS_Shape slotTool = pr::box(
        slotAx, in.slot_length_mm, in.slot_width_mm,
        in.slot_depth_mm + kOverhang);

    // Sub-feature 2: retention undercut — wider box at the floor that
    // extends 1 mm past each long side.  Height = 1.0 mm (the undercut
    // band).  Sits at the same slot bottom Z.
    const double undercutW = in.slot_width_mm + 2.0 * kUndercutOverhang;
    const gp_Pnt undercutOrigin(
        in.position_x_mm - in.slot_length_mm / 2.0,
        in.position_y_mm - undercutW / 2.0,
        slotBottomZ);
    const gp_Ax2 undercutAx(undercutOrigin, gp::DZ());
    const TopoDS_Shape undercutTool = pr::box(
        undercutAx, in.slot_length_mm, undercutW, kUndercutOverhang);

    // Sub-features 3 & 4: two screw pilot holes.  Centered along slot
    // length, ±length/4 from center, on the slot Y centerline.  Drill
    // through the part (or to slot_depth + 2 × pilot, whichever is less).
    const double pilotDepth = std::min(
        in.slot_depth_mm + 2.0 * pilot, topZ - zMin - 0.5);

    const double screwOffset = in.slot_length_mm / 4.0;
    const std::array<double, 2> screwX {
        in.position_x_mm - screwOffset,
        in.position_x_mm + screwOffset,
    };

    const gp_Pnt p1(screwX[0], in.position_y_mm, topZ + kOverhang);
    const gp_Pnt p2(screwX[1], in.position_y_mm, topZ + kOverhang);
    const gp_Ax2 ax1(p1, gp_Dir(0.0, 0.0, -1.0));
    const gp_Ax2 ax2(p2, gp_Dir(0.0, 0.0, -1.0));
    const TopoDS_Shape pilotTool1 = pr::cylinder(ax1, pilot / 2.0,
                                                  pilotDepth + kOverhang);
    const TopoDS_Shape pilotTool2 = pr::cylinder(ax2, pilot / 2.0,
                                                  pilotDepth + kOverhang);

    // Compose: fuse all four cutters then single cut.
    TopoDS_Shape fused = pr::fuse(slotTool, undercutTool);
    fused = pr::fuse(fused, pilotTool1);
    fused = pr::fuse(fused, pilotTool2);

    const TopoDS_Shape newShape = pr::cut(wp.shape(), fused);

    // ── Signature ────────────────────────────────────────────────────────
    json params = {
        { "face_id",        *faceId },
        { "position_x_mm",  in.position_x_mm },
        { "position_y_mm",  in.position_y_mm },
        { "axis_dir",       { in.axis_dir.X(), in.axis_dir.Y(), in.axis_dir.Z() } },
        { "slot_length_mm", in.slot_length_mm },
        { "slot_width_mm",  in.slot_width_mm },
        { "slot_depth_mm",  in.slot_depth_mm },
        { "screw_M",        in.screw_M },
    };
    json pattern = {
        { "kind",                       kSkillId },
        { "is_compound",                true },
        { "subfeature_count",           4 },
        { "subfeatures", json::array({
            { { "op", "rect_slot" },
              { "length_mm", in.slot_length_mm },
              { "width_mm",  in.slot_width_mm },
              { "depth_mm",  in.slot_depth_mm } },
            { { "op", "retention_undercut" },
              { "overhang_mm", kUndercutOverhang },
              { "band_mm",     kUndercutOverhang } },
            { { "op", "screw_pilot_1" },
              { "diameter_mm", pilot },
              { "depth_mm",    pilotDepth },
              { "position_x_offset_mm", -screwOffset } },
            { { "op", "screw_pilot_2" },
              { "diameter_mm", pilot },
              { "depth_mm",    pilotDepth },
              { "position_x_offset_mm",  screwOffset } },
        }) },
        { "slot_length_mm",             in.slot_length_mm },
        { "slot_width_mm",              in.slot_width_mm },
        { "slot_depth_mm",              in.slot_depth_mm },
        { "screw_M",                    in.screw_M },
        { "pilot_dia_mm",               pilot },
        { "pilot_depth_mm",             pilotDepth },
        { "undercut_overhang_mm",       kUndercutOverhang },
        { "screw_count",                2 },
        { "screw_offset_mm",            screwOffset },
        { "slot_wall_count",            4 },
        { "undercut_step_count",        2 },
        { "axis_dir",                   { in.axis_dir.X(), in.axis_dir.Y(), in.axis_dir.Z() } },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "endmill;chamfer_tool;drill";
    tooling.tool_dia_mm       = in.slot_width_mm;
    tooling.tool_length_mm    = pilotDepth + 5.0;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 4;
    tooling.cutting_speed_sfm = 300.0;
    tooling.feed_per_tooth_mm = 0.05;
    const double slotVol  = in.slot_length_mm * in.slot_width_mm * in.slot_depth_mm;
    const double underVol = in.slot_length_mm * undercutW * kUndercutOverhang
                          - in.slot_length_mm * in.slot_width_mm * kUndercutOverhang;
    const double pilotVol = 2.0 * M_PI * (pilot / 2.0) * (pilot / 2.0) * pilotDepth;
    tooling.stock_removed_mm3 = slotVol + underVol + pilotVol;
    tooling.est_cycle_time_s  = std::max(4.0,
        in.slot_depth_mm / 25.0 + pilotDepth / 30.0 * 2.0);
    tooling.extra = {
        { "tool_sequence", {
            { { "tool_type", "endmill" },
              { "tool_dia_mm", in.slot_width_mm },
              { "depth_mm", in.slot_depth_mm } },
            { { "tool_type", "chamfer_tool" },
              { "overhang_mm", kUndercutOverhang } },
            { { "tool_type", "drill" },
              { "tool_dia_mm", pilot },
              { "depth_mm", pilotDepth },
              { "pass_count", 2 } },
        } },
        { "feature_standard",
          "SAE J788 leaf-spring anchor + ISO 273 clearance pilots" },
    };

    FeatureSignature sig { kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::leaf_spring_anchor_compound applied: {}x{}x{} screw={}",
                  in.slot_length_mm, in.slot_width_mm, in.slot_depth_mm, in.screw_M);

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
            { "source", "metadata_replay" },
            { "is_compound", true },
            { "subfeature_count", 4 },
        };
        out.push_back(r);
    }
    return out;
}

}  // namespace koocadcam::skill::leaf_spring_anchor_compound
