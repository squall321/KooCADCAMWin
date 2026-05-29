// @lat: [[engine/skills#cooking_op]]

#include "cooking_op.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <string>
#include <unordered_set>

namespace koocadcam::skill::cooking_op {

using nlohmann::json;

namespace {

const std::unordered_set<std::string>& knownMethods()
{
    static const std::unordered_set<std::string> s = {
        "steam", "bake", "fry", "sterilize",
    };
    return s;
}

std::string toolTypeFor(const std::string& method)
{
    if (method == "steam")     return "steam_cooker";
    if (method == "bake")      return "convection_oven";
    if (method == "fry")       return "industrial_fryer";
    if (method == "sterilize") return "retort_sterilizer";
    return "thermal_processor";
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.method.empty() || knownMethods().count(in.method) == 0) {
        r.add("DFM-INPUT", "error",
              "cooking_op unknown method '" + in.method +
              "' (expected steam | bake | fry | sterilize)");
    }

    if (in.temp_c <= 0.0 || in.temp_c > 300.0) {
        r.add("DFM-COOK-TEMP", "error",
              "cooking_op temp_c " + std::to_string(in.temp_c) +
              " outside (0, 300] °C range");
    }

    if (in.time_min <= 0.0 || in.time_min > 600.0) {
        r.add("DFM-COOK-TIME", "error",
              "cooking_op time_min " + std::to_string(in.time_min) +
              " outside (0, 600] minute range");
    }

    if (in.target_internal_temp_c <= 0.0 ||
        in.target_internal_temp_c > 200.0) {
        r.add("DFM-COOK-INTERNAL", "error",
              "cooking_op target_internal_temp_c " +
              std::to_string(in.target_internal_temp_c) +
              " outside (0, 200] °C range");
    } else if (in.temp_c > 0.0 &&
               in.target_internal_temp_c > in.temp_c) {
        r.add("DFM-COOK-LETHALITY", "info",
              "cooking_op target_internal_temp_c " +
              std::to_string(in.target_internal_temp_c) +
              " °C exceeds chamber temp_c " + std::to_string(in.temp_c) +
              " °C — core cannot exceed ambient");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "cooking_op DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    json params = {
        { "method",                  in.method },
        { "temp_c",                  in.temp_c },
        { "time_min",                in.time_min },
        { "target_internal_temp_c",  in.target_internal_temp_c },
    };

    json pattern = {
        { "kind",                    kSkillId },
        { "method",                  in.method },
        { "temp_c",                  in.temp_c },
        { "time_min",                in.time_min },
        { "target_internal_temp_c",  in.target_internal_temp_c },
        { "process_family",          "food_processing" },
        { "geometric_change",        false },
        { "is_process_only",         true },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code",     f.code },
                            { "severity", f.severity },
                            { "message",  f.message }});

    ToolingMeta tooling;
    tooling.tool_type         = toolTypeFor(in.method);
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "stainless_316l_food_grade";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    tooling.est_cycle_time_s  = in.time_min * 60.0;
    tooling.extra = {
        { "process",                "food_cooking" },
        { "method",                 in.method },
        { "temp_c",                 in.temp_c },
        { "time_min",               in.time_min },
        { "target_internal_temp_c", in.target_internal_temp_c },
        { "process_category",       "food_processing" },
        { "dfm_findings",           findings },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug("skill::cooking_op applied: method={} T={}C t={}min Tc={}C",
                  in.method, in.temp_c, in.time_min,
                  in.target_internal_temp_c);

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
        rf.matched_geometry = { { "source", "feature_history" } };
        out.push_back(rf);
    }
    return out;
}

}  // namespace koocadcam::skill::cooking_op
