// @lat: [[engine/skills#recipe_step]]

#include "recipe_step.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <cmath>

namespace koocadcam::skill::recipe_step {

using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.step_id < 0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": step_id must be >= 0 (got " +
              std::to_string(in.step_id) + ")");
    }
    if (in.parameter_name.empty()) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": parameter_name must not be empty");
    }
    if (in.tolerance < 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": tolerance must be >= 0 (got " +
              std::to_string(in.tolerance) + ")");
    }
    if (in.tolerance > std::abs(in.target_value) && in.target_value != 0.0) {
        r.add("DFM-RECIPE-TOL", "info",
              std::string(kSkillId) + ": tolerance " +
              std::to_string(in.tolerance) +
              " exceeds |target_value| " +
              std::to_string(std::abs(in.target_value)) +
              " — verify tolerance band is correct");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "recipe_step DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    json params = {
        { "step_id",        in.step_id },
        { "parameter_name", in.parameter_name },
        { "target_value",   in.target_value },
        { "tolerance",      in.tolerance },
    };
    json pattern = {
        { "kind",             kSkillId },
        { "step_id",          in.step_id },
        { "parameter_name",   in.parameter_name },
        { "target_value",     in.target_value },
        { "tolerance",        in.tolerance },
        { "geometry_changed", false },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "recipe_executor";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "n/a";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    tooling.est_cycle_time_s  = 0.0;
    tooling.extra = {
        { "process",        "recipe_step_record" },
        { "step_id",        in.step_id },
        { "parameter_name", in.parameter_name },
        { "target_value",   in.target_value },
        { "tolerance",      in.tolerance },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    // No geometric change — clone shape + material verbatim.
    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::recipe_step applied: step={} param={} target={} tol={}",
                  in.step_id, in.parameter_name, in.target_value, in.tolerance);

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

}  // namespace koocadcam::skill::recipe_step
