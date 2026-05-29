// @lat: [[engine/skills#food_mixing]]

#include "food_mixing.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <string>
#include <unordered_set>

namespace koocadcam::skill::food_mixing {

using nlohmann::json;

namespace {

const std::unordered_set<std::string>& knownMixers()
{
    static const std::unordered_set<std::string> s = {
        "ribbon", "paddle", "planetary", "high_shear", "double_cone",
    };
    return s;
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.mixer_type.empty() || knownMixers().count(in.mixer_type) == 0) {
        r.add("DFM-INPUT", "error",
              "food_mixing unknown mixer_type '" + in.mixer_type +
              "' (expected ribbon | paddle | planetary | high_shear | "
              "double_cone)");
    }

    if (in.batch_kg <= 0.0 || in.batch_kg > 10000.0) {
        r.add("DFM-MIX-BATCH", "error",
              "food_mixing batch_kg " + std::to_string(in.batch_kg) +
              " outside (0, 10000] kg range");
    }

    if (in.mix_time_min <= 0.0 || in.mix_time_min > 240.0) {
        r.add("DFM-MIX-TIME", "error",
              "food_mixing mix_time_min " + std::to_string(in.mix_time_min) +
              " outside (0, 240] minute range");
    } else if (in.mix_time_min < 1.0) {
        r.add("DFM-MIX-TIME-SHORT", "info",
              "food_mixing mix_time_min " + std::to_string(in.mix_time_min) +
              " < 1 min — homogeneity may be incomplete");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "food_mixing DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    json params = {
        { "mixer_type",   in.mixer_type },
        { "batch_kg",     in.batch_kg },
        { "mix_time_min", in.mix_time_min },
    };

    json pattern = {
        { "kind",             kSkillId },
        { "mixer_type",       in.mixer_type },
        { "batch_kg",         in.batch_kg },
        { "mix_time_min",     in.mix_time_min },
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
    tooling.tool_type         = "food_mixer_" + in.mixer_type;
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "stainless_316l_food_grade";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    tooling.est_cycle_time_s  = in.mix_time_min * 60.0;
    tooling.extra = {
        { "process",          "food_bulk_mixing" },
        { "mixer_type",       in.mixer_type },
        { "batch_kg",         in.batch_kg },
        { "mix_time_min",     in.mix_time_min },
        { "process_category", "food_processing" },
        { "dfm_findings",     findings },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug("skill::food_mixing applied: mixer={} batch={}kg time={}min",
                  in.mixer_type, in.batch_kg, in.mix_time_min);

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

}  // namespace koocadcam::skill::food_mixing
