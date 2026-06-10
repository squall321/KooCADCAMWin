// @lat: [[engine/skills#concealed_hinge_cup]]

#include "concealed_hinge_cup.hpp"

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

namespace koocadcam::skill::concealed_hinge_cup {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Euro 32 mm system constants ─────────────────────────────────────────

namespace {

constexpr double kCupDia        = 35.0;   // mm — industry standard
constexpr double kCupDepth      = 11.5;   // mm — typical for Blum Compact
constexpr double kFixingPitch   = 32.0;   // mm — pitch between pilot holes
constexpr double kFixingPilotDia = 3.5;   // mm — M3.5 wood / chipboard screw
constexpr double kFixingDepth   = 10.0;   // mm

constexpr double kPanelMinThickness = 14.0;  // cup depth + 2.5 mm

}  // namespace

// ── DFM gates ────────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (!std::isfinite(in.position_x_mm) || !std::isfinite(in.position_y_mm)) {
        r.add("DFM-INPUT", "error",
              "concealed_hinge_cup: position_xy must be finite");
        return r;
    }
    if (in.edge_offset_mm < 3.0 || in.edge_offset_mm > 10.0) {
        r.add("DFM-32SYSTEM", "error",
              "concealed_hinge_cup: edge_offset_mm " +
              std::to_string(in.edge_offset_mm) +
              " outside Euro-system range [3, 10] mm");
    }

    // Panel-thickness check.  Cup must not break through opposite face.
    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double panelThickness = zMax - zMin;
    if (panelThickness < kPanelMinThickness) {
        r.add("DFM-HINGE-CUP", "error",
              "concealed_hinge_cup: panel thickness " +
              std::to_string(panelThickness) + " mm < required " +
              std::to_string(kPanelMinThickness) + " mm");
    }

    // The 32 mm pilot line must fit on the workpiece.  Pilots sit at
    // (cx ± 16 mm, cy − edge_offset_mm) relative to the cup center.
    const double pilotLeftX  = in.position_x_mm - kFixingPitch / 2.0;
    const double pilotRightX = in.position_x_mm + kFixingPitch / 2.0;
    const double pilotY      = in.position_y_mm - in.edge_offset_mm;
    if (pilotLeftX  < xMin || pilotRightX > xMax ||
        pilotY     < yMin || pilotY      > yMax) {
        r.add("DFM-32SYSTEM", "error",
              "concealed_hinge_cup: 32 mm pilot line extends beyond workpiece bbox");
    }
    // Cup boundary check.
    const double cupR = kCupDia / 2.0;
    if (in.position_x_mm - cupR < xMin || in.position_x_mm + cupR > xMax ||
        in.position_y_mm - cupR < yMin || in.position_y_mm + cupR > yMax) {
        r.add("DFM-HINGE-CUP", "error",
              "concealed_hinge_cup: 35 mm cup extends beyond workpiece bbox");
    }
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "concealed_hinge_cup DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double zTop = zMax;
    constexpr double kOverhang = 0.05;

    // Cup bore ───────────────────────────────────────────────────────────
    const gp_Pnt cupTop(in.position_x_mm, in.position_y_mm, zTop + kOverhang);
    const gp_Ax2 cupAx(cupTop, gp_Dir(0, 0, -1));
    const TopoDS_Shape cupTool = pr::cylinder(
        cupAx, kCupDia / 2.0, kCupDepth + kOverhang);

    // Two M3.5 fixing pilots ─────────────────────────────────────────────
    const double pilotLeftX  = in.position_x_mm - kFixingPitch / 2.0;
    const double pilotRightX = in.position_x_mm + kFixingPitch / 2.0;
    const double pilotY      = in.position_y_mm - in.edge_offset_mm;

    auto pilotTool = [&](double px, double py) {
        const gp_Pnt p(px, py, zTop + kOverhang);
        const gp_Ax2 ax(p, gp_Dir(0, 0, -1));
        return pr::cylinder(ax, kFixingPilotDia / 2.0, kFixingDepth + kOverhang);
    };
    const TopoDS_Shape pilotL = pilotTool(pilotLeftX,  pilotY);
    const TopoDS_Shape pilotR = pilotTool(pilotRightX, pilotY);

    const TopoDS_Shape fused1 = pr::fuse(cupTool, pilotL);
    const TopoDS_Shape fusedCutter = pr::fuse(fused1, pilotR);
    const TopoDS_Shape newShape    = pr::cut(wp.shape(), fusedCutter);

    // ── Signature ──────────────────────────────────────────────────────
    json params = {
        { "position_x_mm",  in.position_x_mm },
        { "position_y_mm",  in.position_y_mm },
        { "axis_dir",       { in.axis_dir.X(), in.axis_dir.Y(), in.axis_dir.Z() } },
        { "edge_offset_mm", in.edge_offset_mm },
    };
    json pattern = {
        { "kind",                  kSkillId },
        { "is_compound",           true },
        { "subfeature_count",      3 },
        { "cup_dia_mm",            kCupDia },
        { "cup_depth_mm",          kCupDepth },
        { "fixing_pitch_mm",       kFixingPitch },
        { "fixing_pilot_dia_mm",   kFixingPilotDia },
        { "fixing_depth_mm",       kFixingDepth },
        { "fixing_positions",      json::array({
            json::array({ pilotLeftX,  pilotY }),
            json::array({ pilotRightX, pilotY }),
        }) },
        { "axis_dir",              { in.axis_dir.X(), in.axis_dir.Y(), in.axis_dir.Z() } },
    };

    ToolingMeta tooling;
    tooling.tool_type   = "forstner_bit;drill";
    tooling.tool_dia_mm = kCupDia;
    tooling.tool_material = "carbide";
    const double cupV  = M_PI * (kCupDia / 2.0) * (kCupDia / 2.0) * kCupDepth;
    const double oneFixV = M_PI * (kFixingPilotDia / 2.0) * (kFixingPilotDia / 2.0)
                           * kFixingDepth;
    tooling.stock_removed_mm3 = cupV + 2.0 * oneFixV;
    tooling.est_cycle_time_s  = std::max(3.0, cupV / 200.0);
    tooling.extra = {
        { "tool_sequence", json::array({
            { { "tool_type", "forstner_bit" }, { "tool_dia_mm", kCupDia },
              { "depth_mm", kCupDepth }, { "purpose", "cup_bore" } },
            { { "tool_type", "drill" }, { "tool_dia_mm", kFixingPilotDia },
              { "depth_mm", kFixingDepth }, { "pass_count", 2 },
              { "purpose", "fixing_screw_pilot" } },
        }) },
        { "standard", "DIN-30798-32mm-system" },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };
    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);  // keep full chain history
    wpNew->addFeature(sig);

    spdlog::debug("skill::concealed_hinge_cup applied: Ø{}×{} + 2× Ø{} pilots @{}",
                  kCupDia, kCupDepth, kFixingPilotDia, kFixingPitch);

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

}  // namespace koocadcam::skill::concealed_hinge_cup
