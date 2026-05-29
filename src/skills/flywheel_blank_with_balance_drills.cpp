// @lat: [[engine/skills#flywheel_blank_with_balance_drills]]

#include "flywheel_blank_with_balance_drills.hpp"

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

namespace koocadcam::skill::flywheel_blank_with_balance_drills {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.outer_dia_mm <= 0.0 || in.thickness_mm <= 0.0 ||
        in.bore_dia_mm  <= 0.0) {
        r.add("DFM-INPUT", "error",
              "flywheel_blank_with_balance_drills: positive outer/thickness/bore required");
        return r;
    }

    if (in.bore_dia_mm < 0.8) {
        r.add("DFM-FLYWHEEL-BORE", "error",
              "flywheel_blank_with_balance_drills: bore_dia < 0.8 mm");
    }
    if (in.bore_dia_mm >= in.outer_dia_mm) {
        r.add("DFM-INPUT", "error",
              "flywheel_blank_with_balance_drills: bore_dia >= outer_dia");
    }

    if (in.balance_drill_count < 2 || in.balance_drill_count > 24) {
        r.add("DFM-FLYWHEEL-COUNT", "error",
              "flywheel_blank_with_balance_drills: balance_drill_count " +
              std::to_string(in.balance_drill_count) +
              " outside [2, 24] (ISO 1940 typical)");
    }
    if (in.balance_drill_dia_mm < 0.8) {
        r.add("DFM-002", "error",
              "flywheel_blank_with_balance_drills: balance drill < 0.8 mm");
    }
    if (in.balance_drill_depth_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "flywheel_blank_with_balance_drills: balance drill depth must be > 0");
    }
    if (in.balance_drill_depth_mm >= in.thickness_mm) {
        r.add("DFM-FLYWHEEL-DEPTH", "error",
              "flywheel_blank_with_balance_drills: drill depth " +
              std::to_string(in.balance_drill_depth_mm) +
              " >= thickness " + std::to_string(in.thickness_mm) +
              " (drill would exit opposite face)");
    }

    const double oRad      = in.outer_dia_mm / 2.0;
    const double bRad      = in.bore_dia_mm  / 2.0;
    const double pitchR    = (in.hole_radius_mm > 0.0)
                              ? in.hole_radius_mm
                              : 0.85 * oRad;
    const double drillR    = in.balance_drill_dia_mm / 2.0;
    if (pitchR + drillR >= oRad) {
        r.add("DFM-FLYWHEEL-RADIUS", "error",
              "flywheel_blank_with_balance_drills: drill exits outer rim "
              "(pitch_r + drill_r >= outer_r)");
    }
    if (pitchR - drillR <= bRad) {
        r.add("DFM-FLYWHEEL-RADIUS", "error",
              "flywheel_blank_with_balance_drills: drill collides with bore "
              "(pitch_r - drill_r <= bore_r)");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "flywheel_blank_with_balance_drills DFM failed:";
        for (const auto& f : dfm.findings) msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const double oRad     = in.outer_dia_mm / 2.0;
    const double bRad     = in.bore_dia_mm  / 2.0;
    const double Z        = in.thickness_mm;
    const int    N        = in.balance_drill_count;
    const double pitchR   = (in.hole_radius_mm > 0.0)
                              ? in.hole_radius_mm
                              : 0.85 * oRad;
    const double drillR   = in.balance_drill_dia_mm / 2.0;
    const double drillH   = in.balance_drill_depth_mm;
    const double kOverhang = 0.05;
    const double angOff   = in.angle_offset_deg * M_PI / 180.0;

    // ── Tool 1: through bore
    const gp_Ax2 axBore(gp_Pnt(0, 0, -kOverhang), gp::DZ());
    const TopoDS_Shape boreTool =
        pr::cylinder(axBore, bRad, Z + 2.0 * kOverhang);

    // ── Tools 2..N+1: balance drills.  Blind from the +Z face, depth drillH.
    std::vector<TopoDS_Shape> tools;
    tools.reserve(static_cast<size_t>(N + 1));
    tools.push_back(boreTool);

    json holePositions = json::array();
    for (int i = 0; i < N; ++i) {
        const double th = angOff + (2.0 * M_PI * i) / N;
        const double x = pitchR * std::cos(th);
        const double y = pitchR * std::sin(th);
        // Drill starts at z = Z + overhang, goes down by drillH + overhang.
        const gp_Pnt p(x, y, Z + kOverhang);
        const gp_Ax2 ax(p, gp_Dir(0.0, 0.0, -1.0));
        tools.push_back(pr::cylinder(ax, drillR, drillH + kOverhang));
        holePositions.push_back({ { "x", x }, { "y", y },
                                  { "angle_deg", th * 180.0 / M_PI } });
    }

    const TopoDS_Shape newShape = pr::cutMany(wp.shape(), tools);
    const int subCount = 1 + N;

    json params = {
        { "outer_dia_mm",           in.outer_dia_mm },
        { "thickness_mm",           in.thickness_mm },
        { "bore_dia_mm",            in.bore_dia_mm },
        { "balance_drill_count",    N },
        { "balance_drill_dia_mm",   in.balance_drill_dia_mm },
        { "balance_drill_depth_mm", in.balance_drill_depth_mm },
        { "hole_radius_mm",         pitchR },
        { "angle_offset_deg",       in.angle_offset_deg },
    };
    json pattern = {
        { "kind",                   kSkillId },
        { "is_compound",            true },
        { "subfeature_count",       subCount },
        { "outer_dia_mm",           in.outer_dia_mm },
        { "thickness_mm",           in.thickness_mm },
        { "bore_dia_mm",            in.bore_dia_mm },
        { "balance_drill_count",    N },
        { "balance_drill_dia_mm",   in.balance_drill_dia_mm },
        { "balance_drill_depth_mm", in.balance_drill_depth_mm },
        { "hole_radius_mm",         pitchR },
        { "hole_positions",         holePositions },
    };

    ToolingMeta tooling;
    tooling.tool_type     = "drill";
    tooling.tool_dia_mm   = in.balance_drill_dia_mm;
    tooling.tool_material = "carbide";
    tooling.stock_removed_mm3 =
        M_PI * bRad * bRad * Z +
        static_cast<double>(N) * M_PI * drillR * drillR * drillH;
    tooling.extra = {
        { "tool_sequence", {
            { { "tool_type", "drill" }, { "tool_dia_mm", in.bore_dia_mm },
              { "note", "central bore" } },
            { { "tool_type", "drill" }, { "tool_dia_mm", in.balance_drill_dia_mm },
              { "pass_count", N }, { "note", "balance drills" } },
        } }
    };

    FeatureSignature sig { kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    wpNew->addFeature(sig);

    spdlog::debug("skill::flywheel_blank_with_balance_drills applied: OD={} N_drill={}",
                  in.outer_dia_mm, N);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────

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
            { "source",           "feature_history_replay" },
            { "subfeature_count", f.pattern.value("subfeature_count", 0) },
        };
        out.push_back(r);
    }
    return out;
}

}  // namespace koocadcam::skill::flywheel_blank_with_balance_drills
