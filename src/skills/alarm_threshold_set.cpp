// @lat: [[engine/skills#alarm_threshold_set]]

#include "alarm_threshold_set.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

namespace koocadcam::skill::alarm_threshold_set {

using nlohmann::json;

namespace {

bool isKnownAction(const std::string& a)
{
    return a == "alert" || a == "shutdown" || a == "interlock";
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.alarm_id.empty()) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": alarm_id must not be empty");
    }
    if (in.parameter_name.empty()) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": parameter_name must not be empty");
    }
    if (in.low >= in.high) {
        r.add("DFM-ALARM-BAND", "error",
              std::string(kSkillId) + ": low (" +
              std::to_string(in.low) +
              ") must be < high (" +
              std::to_string(in.high) + ")");
    }
    if (!isKnownAction(in.action)) {
        r.add("DFM-ALARM-ACTION", "info",
              std::string(kSkillId) + ": action '" + in.action +
              "' unrecognised — expected 'alert' | 'shutdown' | 'interlock'");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "alarm_threshold_set DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const std::string action = isKnownAction(in.action) ? in.action : "alert";

    json params = {
        { "alarm_id",       in.alarm_id },
        { "parameter_name", in.parameter_name },
        { "low",            in.low },
        { "high",           in.high },
        { "action",         in.action },
    };
    json pattern = {
        { "kind",             kSkillId },
        { "alarm_id",         in.alarm_id },
        { "parameter_name",   in.parameter_name },
        { "low",              in.low },
        { "high",             in.high },
        { "action",           action },
        { "geometry_changed", false },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "process_alarm";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "n/a";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    tooling.est_cycle_time_s  = 0.0;
    tooling.extra = {
        { "process",        "alarm_threshold_register" },
        { "alarm_id",       in.alarm_id },
        { "parameter_name", in.parameter_name },
        { "low",            in.low },
        { "high",           in.high },
        { "action",         action },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    // No geometric change — clone shape + material verbatim.
    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::alarm_threshold_set applied: id={} param={} band=[{}, {}] act={}",
                  in.alarm_id, in.parameter_name, in.low, in.high, action);

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

}  // namespace koocadcam::skill::alarm_threshold_set
