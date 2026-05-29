// @lat: [[engine/skills#schedule_parameter_ramp]]

#include "schedule_parameter_ramp.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

namespace koocadcam::skill::schedule_parameter_ramp {

using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.ramp_time_min <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": ramp_time_min must be > 0 (got " +
              std::to_string(in.ramp_time_min) + ")");
    }
    if (in.steps < 1) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": steps must be >= 1 (got " +
              std::to_string(in.steps) + ")");
    }
    if (in.start_value == in.end_value && r.passed) {
        r.add("DFM-RAMP-SLOPE", "info",
              std::string(kSkillId) +
              ": start_value == end_value — ramp is flat, verify intent");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "schedule_parameter_ramp DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const double stepDelta = (in.end_value - in.start_value) /
                             static_cast<double>(in.steps);

    json params = {
        { "start_value",   in.start_value },
        { "end_value",     in.end_value },
        { "ramp_time_min", in.ramp_time_min },
        { "steps",         in.steps },
    };
    json pattern = {
        { "kind",             kSkillId },
        { "start_value",      in.start_value },
        { "end_value",        in.end_value },
        { "ramp_time_min",    in.ramp_time_min },
        { "steps",            in.steps },
        { "step_delta",       stepDelta },
        { "geometry_changed", false },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "ramp_scheduler";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "n/a";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    tooling.est_cycle_time_s  = in.ramp_time_min * 60.0;  // schedule duration
    tooling.extra = {
        { "process",       "parameter_ramp_schedule" },
        { "start_value",   in.start_value },
        { "end_value",     in.end_value },
        { "ramp_time_min", in.ramp_time_min },
        { "steps",         in.steps },
        { "step_delta",    stepDelta },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    // No geometric change — clone shape + material verbatim.
    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::schedule_parameter_ramp applied: {}→{} over {}min in {} steps",
                  in.start_value, in.end_value, in.ramp_time_min, in.steps);

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

}  // namespace koocadcam::skill::schedule_parameter_ramp
