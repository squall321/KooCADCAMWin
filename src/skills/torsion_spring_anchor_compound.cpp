// @lat: [[engine/skills#torsion_spring_anchor_compound]]

#include "torsion_spring_anchor_compound.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <gp.hxx>
#include <gp_Ax2.hxx>
#include <gp_Pnt.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <memory>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::skill::torsion_spring_anchor_compound {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────
//
// ISO 8459 + DIN 2089-2 torsion-spring assembly gates.

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.pivot_bore <= 0.0 || in.pivot_depth_mm <= 0.0
        || in.arm_slot_width <= 0.0 || in.arm_slot_length <= 0.0
        || in.anchor_radius_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "torsion_spring_anchor_compound: pivot_bore, pivot_depth, "
              "arm_slot_width, arm_slot_length, anchor_radius must all be > 0");
        return r;
    }

    if (in.pivot_bore < 3.0) {
        r.add("DFM-TORSION-BORE", "error",
              "torsion_spring_anchor_compound: pivot_bore " +
              std::to_string(in.pivot_bore) +
              " mm < 3 mm (below ISO 8459 torsion spring coil ID range)");
    }

    // Wire-dia heuristic: spring wire >= 15% of pivot bore.
    if (in.arm_slot_width < in.pivot_bore * 0.15) {
        r.add("DFM-TORSION-WIRE", "error",
              "torsion_spring_anchor_compound: arm_slot_width " +
              std::to_string(in.arm_slot_width) +
              " < 0.15 × pivot_bore (" +
              std::to_string(in.pivot_bore * 0.15) +
              ") — spring wire would not clear");
    }

    // Avoid over-aggressive slots that swallow the part.
    if (in.arm_slot_length > in.pivot_bore * 3.0) {
        r.add("DFM-TORSION-SLOT", "warning",
              "torsion_spring_anchor_compound: arm_slot_length " +
              std::to_string(in.arm_slot_length) +
              " > 3 × pivot_bore (" + std::to_string(in.pivot_bore * 3.0) +
              ") — excessive material loss around pivot");
    }

    // Anchor pilot drill diameter heuristic.
    const double pilotDia = in.arm_slot_width * 0.8;
    if (pilotDia < 0.8) {
        r.add("DFM-002", "error",
              "torsion_spring_anchor_compound: derived anchor pilot " +
              std::to_string(pilotDia) + " mm < min drill 0.8 mm");
    }

    // Anchor radius must place pilots outside the pivot bore (clear of bore wall).
    if (in.anchor_radius_mm < in.pivot_bore / 2.0 + pilotDia / 2.0 + 0.5) {
        r.add("DFM-TORSION-ANCHOR", "error",
              "torsion_spring_anchor_compound: anchor_radius " +
              std::to_string(in.anchor_radius_mm) +
              " too small — anchor pilots overlap pivot bore");
    }

    if (!wp.shape().IsNull()) {
        double xMin, yMin, zMin, xMax, yMax, zMax;
        wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
        const double thickness = zMax - zMin;
        if (thickness < in.pivot_depth_mm + 2.0) {
            r.add("DFM-TORSION-WALL", "error",
                  "torsion_spring_anchor_compound: stock thickness " +
                  std::to_string(thickness) +
                  " < pivot_depth + 2 mm wall (" +
                  std::to_string(in.pivot_depth_mm + 2.0) + ")");
        }
    }

    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────
//
// Sub-feature chain:
//   1. Pivot bore (cylinder, depth = pivot_depth_mm)
//   2. Arm-clearance slot (box extending radially from pivot wall)
//   3. Anchor-leg pilot 1 (cylinder at anchor_radius × cos(θ₁), sin(θ₁))
//   4. Anchor-leg pilot 2 (cylinder at anchor_radius × cos(θ₂), sin(θ₂))

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "torsion_spring_anchor_compound DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    auto faceId = wp.resolve(in.face_id);
    if (!faceId) throw SkillError(
        "torsion_spring_anchor_compound: face_id datum unresolved");

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    constexpr double kOverhang = 0.05;

    const double topZ = zMax;
    const gp_Dir adir = in.axis_dir;
    const double pivotR = in.pivot_bore / 2.0;
    const double pilotDia = in.arm_slot_width * 0.8;

    // Sub-feature 1: pivot bore cylinder.
    const gp_Pnt pivotStart(in.position_x_mm, in.position_y_mm,
                             topZ + kOverhang);
    const gp_Ax2 pivotAx(pivotStart, adir);
    const TopoDS_Shape pivotTool = pr::cylinder(
        pivotAx, pivotR, in.pivot_depth_mm + kOverhang);

    // Sub-feature 2: arm-clearance slot — radial slot along +X from pivot,
    // approximated as a centered box from (pivotR) to (pivotR + slot_len)
    // along +X, width = arm_slot_width, depth = pivot_depth.
    // Box origin: lower-left corner.
    const gp_Pnt slotOrigin(
        in.position_x_mm + pivotR - kOverhang,
        in.position_y_mm - in.arm_slot_width / 2.0,
        topZ - in.pivot_depth_mm);
    const gp_Ax2 slotAx(slotOrigin, gp::DZ());
    const TopoDS_Shape slotTool = pr::box(
        slotAx,
        in.arm_slot_length + kOverhang,
        in.arm_slot_width,
        in.pivot_depth_mm + kOverhang);

    // Sub-features 3 & 4: anchor-leg pilots at (anchor_radius, angle).
    const double a1Rad = in.anchor_angle_1_deg * M_PI / 180.0;
    const double a2Rad = in.anchor_angle_2_deg * M_PI / 180.0;
    const double pilotDepth = in.pivot_depth_mm + 2.0;

    const gp_Pnt p1Start(
        in.position_x_mm + in.anchor_radius_mm * std::cos(a1Rad),
        in.position_y_mm + in.anchor_radius_mm * std::sin(a1Rad),
        topZ + kOverhang);
    const gp_Ax2 pilot1Ax(p1Start, adir);
    const TopoDS_Shape pilotTool1 = pr::cylinder(
        pilot1Ax, pilotDia / 2.0, pilotDepth + kOverhang);

    const gp_Pnt p2Start(
        in.position_x_mm + in.anchor_radius_mm * std::cos(a2Rad),
        in.position_y_mm + in.anchor_radius_mm * std::sin(a2Rad),
        topZ + kOverhang);
    const gp_Ax2 pilot2Ax(p2Start, adir);
    const TopoDS_Shape pilotTool2 = pr::cylinder(
        pilot2Ax, pilotDia / 2.0, pilotDepth + kOverhang);

    // Compose: fuse all four cutters then single cut.
    TopoDS_Shape fused = pr::fuse(pivotTool, slotTool);
    fused = pr::fuse(fused, pilotTool1);
    fused = pr::fuse(fused, pilotTool2);

    const TopoDS_Shape newShape = pr::cut(wp.shape(), fused);

    // ── Signature ────────────────────────────────────────────────────────
    json params = {
        { "face_id",            *faceId },
        { "position_x_mm",      in.position_x_mm },
        { "position_y_mm",      in.position_y_mm },
        { "axis_dir",           { adir.X(), adir.Y(), adir.Z() } },
        { "pivot_bore",         in.pivot_bore },
        { "pivot_depth_mm",     in.pivot_depth_mm },
        { "arm_slot_width",     in.arm_slot_width },
        { "arm_slot_length",    in.arm_slot_length },
        { "anchor_angle_1_deg", in.anchor_angle_1_deg },
        { "anchor_angle_2_deg", in.anchor_angle_2_deg },
        { "anchor_radius_mm",   in.anchor_radius_mm },
    };
    json pattern = {
        { "kind",                       kSkillId },
        { "is_compound",                true },
        { "subfeature_count",           4 },
        { "subfeatures", json::array({
            { { "op", "pivot_bore" },
              { "diameter_mm", in.pivot_bore },
              { "depth_mm",    in.pivot_depth_mm } },
            { { "op", "arm_clearance_slot" },
              { "width_mm",    in.arm_slot_width },
              { "length_mm",   in.arm_slot_length },
              { "depth_mm",    in.pivot_depth_mm } },
            { { "op", "anchor_pilot_1" },
              { "diameter_mm", pilotDia },
              { "depth_mm",    pilotDepth },
              { "angle_deg",   in.anchor_angle_1_deg } },
            { { "op", "anchor_pilot_2" },
              { "diameter_mm", pilotDia },
              { "depth_mm",    pilotDepth },
              { "angle_deg",   in.anchor_angle_2_deg } },
        }) },
        { "pivot_bore",                 in.pivot_bore },
        { "pivot_depth_mm",             in.pivot_depth_mm },
        { "arm_slot_width",             in.arm_slot_width },
        { "arm_slot_length",            in.arm_slot_length },
        { "anchor_radius_mm",           in.anchor_radius_mm },
        { "anchor_pilot_dia_mm",        pilotDia },
        { "anchor_pilot_depth_mm",      pilotDepth },
        { "anchor_pilot_count",         2 },
        { "anchor_angle_1_deg",         in.anchor_angle_1_deg },
        { "anchor_angle_2_deg",         in.anchor_angle_2_deg },
        { "pivot_cyl_face_count",       1 },
        { "slot_planar_wall_count",     2 },
        { "axis_dir",                   { adir.X(), adir.Y(), adir.Z() } },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "endmill;drill";
    tooling.tool_dia_mm       = in.pivot_bore;
    tooling.tool_length_mm    = pilotDepth + 5.0;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 4;
    tooling.cutting_speed_sfm = 300.0;
    tooling.feed_per_tooth_mm = 0.05;
    const double pivotVol = M_PI * pivotR * pivotR * in.pivot_depth_mm;
    const double slotVol  = in.arm_slot_width * in.arm_slot_length
                          * in.pivot_depth_mm;
    const double pilotVol = 2.0 * M_PI * (pilotDia / 2.0) * (pilotDia / 2.0)
                          * pilotDepth;
    tooling.stock_removed_mm3 = pivotVol + slotVol + pilotVol;
    tooling.est_cycle_time_s  = std::max(3.0,
        in.pivot_depth_mm / 25.0 + pilotDepth / 30.0 * 2.0);
    tooling.extra = {
        { "tool_sequence", {
            { { "tool_type", "endmill" },
              { "tool_dia_mm", in.pivot_bore },
              { "depth_mm",    in.pivot_depth_mm } },
            { { "tool_type", "endmill" },
              { "tool_dia_mm", in.arm_slot_width },
              { "depth_mm",    in.pivot_depth_mm } },
            { { "tool_type", "drill" },
              { "tool_dia_mm", pilotDia },
              { "depth_mm",    pilotDepth },
              { "pass_count",  2 } },
        } },
        { "feature_standard",
          "ISO 8459 / DIN 2089-2 torsion-spring assembly" },
    };

    FeatureSignature sig { kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::torsion_spring_anchor_compound applied: pivot={} arm={}x{}",
                  in.pivot_bore, in.arm_slot_width, in.arm_slot_length);

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

}  // namespace koocadcam::skill::torsion_spring_anchor_compound
