// @lat: [[engine/skills#cam_lock_cavity]]

#include "cam_lock_cavity.hpp"

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

namespace koocadcam::skill::cam_lock_cavity {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Catalog constants (Master Lock 7100 / generic cam lock) ─────────────

namespace {

constexpr double kBodyDia        = 19.0;
constexpr double kCapDia         = 22.0;
constexpr double kCapDepth       = 1.5;
constexpr double kCamSlotLength  = 35.0;
constexpr double kCamSlotHeight  = 6.0;
constexpr double kCamSlotDepth   = 8.0;       // slot depth on far face

constexpr double kMinPanelThickness = 20.0;

}  // namespace

// ── DFM gates ────────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (!std::isfinite(in.position_x_mm) || !std::isfinite(in.position_y_mm)) {
        r.add("DFM-INPUT", "error",
              "cam_lock_cavity: position_xy must be finite");
        return r;
    }

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double thickness = zMax - zMin;
    if (thickness < kMinPanelThickness) {
        r.add("DFM-LOCK-BODY", "error",
              "cam_lock_cavity: panel thickness " +
              std::to_string(thickness) + " mm < required " +
              std::to_string(kMinPanelThickness) + " mm");
    }
    const double capR = kCapDia / 2.0;
    if (in.position_x_mm - capR < xMin || in.position_x_mm + capR > xMax ||
        in.position_y_mm - capR < yMin || in.position_y_mm + capR > yMax) {
        r.add("DFM-LOCK-BODY", "error",
              "cam_lock_cavity: cap counterbore exceeds workpiece bbox");
    }
    // Cam slot extends ±slot_length/2 along X.
    if (in.position_x_mm - kCamSlotLength / 2.0 < xMin ||
        in.position_x_mm + kCamSlotLength / 2.0 > xMax)
    {
        r.add("DFM-LOCK-CAM", "error",
              "cam_lock_cavity: cam slot (35 mm) exceeds workpiece X bbox");
    }
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "cam_lock_cavity DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double zTop = zMax, zBot = zMin;
    const double thickness = zTop - zBot;
    constexpr double kOverhang = 0.05;

    // Sub-feature 1: Ø19 mm through-bore (axis = -Z from zTop to zBot)
    const gp_Pnt boreTop(in.position_x_mm, in.position_y_mm, zTop + kOverhang);
    const gp_Ax2 boreAx(boreTop, gp_Dir(0, 0, -1));
    const TopoDS_Shape bodyTool = pr::cylinder(
        boreAx, kBodyDia / 2.0, thickness + 2.0 * kOverhang);

    // Sub-feature 2: cam slot box (centered on bore axis, near zBot)
    // The slot is on the FAR face — i.e. the rotation plane of the cam arm.
    // Box origin in (X, Y, Z) = bottomCenter − (length/2, height/2, 0).
    const gp_Pnt slotOrigin(
        in.position_x_mm - kCamSlotLength / 2.0,
        in.position_y_mm - kCamSlotHeight / 2.0,
        zBot - kOverhang);
    const TopoDS_Shape camSlotTool = pr::box(
        gp_Ax2(slotOrigin, gp::DZ()),
        kCamSlotLength, kCamSlotHeight, kCamSlotDepth + kOverhang);

    // Sub-feature 3: cap counterbore on key side (Ø22 × 1.5 mm)
    const gp_Pnt capTop(in.position_x_mm, in.position_y_mm, zTop + kOverhang);
    const gp_Ax2 capAx(capTop, gp_Dir(0, 0, -1));
    const TopoDS_Shape capTool = pr::cylinder(
        capAx, kCapDia / 2.0, kCapDepth + kOverhang);

    const TopoDS_Shape f1 = pr::fuse(bodyTool, camSlotTool);
    const TopoDS_Shape fusedCutter = pr::fuse(f1, capTool);
    const TopoDS_Shape newShape    = pr::cut(wp.shape(), fusedCutter);

    // ── Signature ──────────────────────────────────────────────────────
    json params = {
        { "position_x_mm", in.position_x_mm },
        { "position_y_mm", in.position_y_mm },
        { "axis_dir",      { in.axis_dir.X(), in.axis_dir.Y(), in.axis_dir.Z() } },
    };
    json pattern = {
        { "kind",             kSkillId },
        { "is_compound",      true },
        { "subfeature_count", 3 },
        { "body_dia_mm",      kBodyDia },
        { "cap_dia_mm",       kCapDia },
        { "cap_depth_mm",     kCapDepth },
        { "cam_slot_length_mm", kCamSlotLength },
        { "cam_slot_height_mm", kCamSlotHeight },
        { "cam_slot_depth_mm",  kCamSlotDepth },
        { "panel_thickness_mm", thickness },
        { "axis_dir",        { in.axis_dir.X(), in.axis_dir.Y(), in.axis_dir.Z() } },
    };

    ToolingMeta tooling;
    tooling.tool_type   = "drill;end_mill;counterbore";
    tooling.tool_dia_mm = kBodyDia;
    tooling.tool_material = "carbide";
    const double bodyV = M_PI * (kBodyDia / 2.0) * (kBodyDia / 2.0) * thickness;
    const double slotV = kCamSlotLength * kCamSlotHeight * kCamSlotDepth;
    const double capR  = kCapDia / 2.0, bR = kBodyDia / 2.0;
    const double capV  = M_PI * (capR * capR - bR * bR) * kCapDepth;
    tooling.stock_removed_mm3 = bodyV + slotV + capV;
    tooling.est_cycle_time_s  = std::max(3.0, bodyV / 200.0);
    tooling.extra = {
        { "tool_sequence", json::array({
            { { "tool_type", "drill" }, { "tool_dia_mm", kBodyDia },
              { "depth_mm", thickness }, { "purpose", "body_through_bore" } },
            { { "tool_type", "end_mill" }, { "tool_dia_mm", kCamSlotHeight },
              { "purpose", "cam_arm_slot" }, { "slot_length_mm", kCamSlotLength } },
            { { "tool_type", "counterbore" }, { "tool_dia_mm", kCapDia },
              { "depth_mm", kCapDepth }, { "purpose", "decorative_cap_recess" } },
        }) },
        { "standard", "cam_lock_19mm" },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };
    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);  // keep full chain history
    wpNew->addFeature(sig);

    spdlog::debug("skill::cam_lock_cavity applied: Ø{} body + Ø{} cap + {}×{} slot",
                  kBodyDia, kCapDia, kCamSlotLength, kCamSlotHeight);

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

}  // namespace koocadcam::skill::cam_lock_cavity
