// @lat: [[engine/skills#fermentation]]

#include "fermentation.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <string>

namespace koocadcam::skill::fermentation {

using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.starter_culture.empty()) {
        r.add("DFM-INPUT", "error",
              "fermentation starter_culture must not be empty");
    }

    if (in.temp_c <= 0.0 || in.temp_c > 60.0) {
        r.add("DFM-FERM-TEMP", "error",
              "fermentation temp_c " + std::to_string(in.temp_c) +
              " outside (0, 60] °C range");
    }

    if (in.time_h <= 0.0 || in.time_h > 720.0) {
        r.add("DFM-FERM-TIME", "error",
              "fermentation time_h " + std::to_string(in.time_h) +
              " outside (0, 720] hour range");
    }

    if (in.ph_target < 2.5 || in.ph_target > 7.0) {
        r.add("DFM-FERM-PH", "error",
              "fermentation ph_target " + std::to_string(in.ph_target) +
              " outside [2.5, 7.0] range");
    } else if (in.ph_target > 5.5) {
        r.add("DFM-FERM-PH-HIGH", "info",
              "fermentation ph_target " + std::to_string(in.ph_target) +
              " > 5.5 — insufficient acidification for many fermented "
              "food safety models");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "fermentation DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    json params = {
        { "temp_c",          in.temp_c },
        { "time_h",          in.time_h },
        { "starter_culture", in.starter_culture },
        { "ph_target",       in.ph_target },
    };

    json pattern = {
        { "kind",             kSkillId },
        { "temp_c",           in.temp_c },
        { "time_h",           in.time_h },
        { "starter_culture",  in.starter_culture },
        { "ph_target",        in.ph_target },
        { "process_family",   "food_processing" },
        { "geometric_change", false },
        { "is_process_only",  true },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code",     f.code },
                            { "severity", f.severity },
                            { "message",  f.message }});

    ToolingMeta tooling;
    tooling.tool_type         = "fermentation_vessel";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "stainless_316l_food_grade";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    tooling.est_cycle_time_s  = in.time_h * 3600.0;
    tooling.extra = {
        { "process",          "food_fermentation" },
        { "temp_c",           in.temp_c },
        { "time_h",           in.time_h },
        { "starter_culture",  in.starter_culture },
        { "ph_target",        in.ph_target },
        { "process_category", "food_processing" },
        { "dfm_findings",     findings },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug("skill::fermentation applied: T={}C t={}h starter={} pH={}",
                  in.temp_c, in.time_h, in.starter_culture, in.ph_target);

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

}  // namespace koocadcam::skill::fermentation
