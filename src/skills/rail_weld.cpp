// @lat: [[engine/skills#rail_weld]]

#include "rail_weld.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <string>

namespace koocadcam::skill::rail_weld {

using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.weld_method != "thermite" && in.weld_method != "flash_butt") {
        r.add("DFM-RW-METHOD", "error",
              std::string(kSkillId) + ": weld_method '" + in.weld_method +
              "' not in {thermite, flash_butt}");
    }

    if (in.rail_grade.empty()) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": rail_grade must be non-empty");
    }

    if (in.joint_count < 1) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": joint_count must be >= 1 (got " +
              std::to_string(in.joint_count) + ")");
    } else if (in.joint_count > 50) {
        r.add("DFM-RW-CAMPAIGN", "info",
              std::string(kSkillId) + ": joint_count " +
              std::to_string(in.joint_count) +
              " > 50 — split into multiple shifts for thermal management");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "rail_weld DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    json params = {
        { "weld_method", in.weld_method },
        { "rail_grade",  in.rail_grade },
        { "joint_count", in.joint_count },
    };
    json pattern = {
        { "kind",             kSkillId },
        { "weld_method",      in.weld_method },
        { "rail_grade",       in.rail_grade },
        { "joint_count",      in.joint_count },
        { "geometry_changed", false },
    };

    ToolingMeta tooling;
    // Thermite crucible+mould kit vs flash-butt head are very different.
    tooling.tool_type         = (in.weld_method == "flash_butt")
                                    ? "flash_butt_welder"
                                    : "thermite_crucible_kit";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "consumable_kit";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    // Thermite ~ 25 min per joint incl. cooldown; flash-butt ~ 4 min per joint.
    const double perJoint = (in.weld_method == "flash_butt") ? 240.0 : 1500.0;
    tooling.est_cycle_time_s  = perJoint * static_cast<double>(in.joint_count);
    tooling.extra = {
        { "process",     "rail_weld" },
        { "weld_method", in.weld_method },
        { "rail_grade",  in.rail_grade },
        { "joint_count", in.joint_count },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::rail_weld applied: method={} grade={} joints={}",
                  in.weld_method, in.rail_grade, in.joint_count);

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
        rf.confidence       = 1.0;
        rf.matched_geometry = { { "source", "metadata" } };
        out.push_back(rf);
    }
    return out;
}

}  // namespace koocadcam::skill::rail_weld
