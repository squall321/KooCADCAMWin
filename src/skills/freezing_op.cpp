// @lat: [[engine/skills#freezing_op]]

#include "freezing_op.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <string>

namespace koocadcam::skill::freezing_op {

using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.final_temp_c >= 0.0) {
        r.add("DFM-INPUT", "error",
              "freezing_op final_temp_c must be < 0 °C (got " +
              std::to_string(in.final_temp_c) + ")");
    } else if (in.final_temp_c < -80.0) {
        r.add("DFM-FREEZE-TEMP-LOW", "info",
              "freezing_op final_temp_c " + std::to_string(in.final_temp_c) +
              " °C below practical industrial freezer range (≥ -80 °C typical)");
    }

    if (in.freezing_time_min <= 0.0 || in.freezing_time_min > 720.0) {
        r.add("DFM-FREEZE-TIME", "error",
              "freezing_op freezing_time_min " +
              std::to_string(in.freezing_time_min) +
              " outside (0, 720] minute range");
    } else if (in.freezing_time_min > 240.0) {
        r.add("DFM-FREEZE-SLOW", "info",
              "freezing_op freezing_time_min " +
              std::to_string(in.freezing_time_min) +
              " > 240 min — slow freezing risks large ice crystals + "
              "texture damage");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "freezing_op DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    json params = {
        { "final_temp_c",      in.final_temp_c },
        { "freezing_time_min", in.freezing_time_min },
    };

    json pattern = {
        { "kind",              kSkillId },
        { "final_temp_c",      in.final_temp_c },
        { "freezing_time_min", in.freezing_time_min },
        { "process_family",    "food_processing" },
        { "geometric_change",  false },
        { "is_process_only",   true },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code",     f.code },
                            { "severity", f.severity },
                            { "message",  f.message }});

    ToolingMeta tooling;
    // IQF tunnel is the dominant industrial archetype; use that name as
    // the canonical tool_type for this skill family.
    tooling.tool_type         = "iqf_freezing_tunnel";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "stainless_316l_food_grade";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    tooling.est_cycle_time_s  = in.freezing_time_min * 60.0;
    tooling.extra = {
        { "process",           "food_rapid_freezing" },
        { "final_temp_c",      in.final_temp_c },
        { "freezing_time_min", in.freezing_time_min },
        { "process_category",  "food_processing" },
        { "dfm_findings",      findings },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug("skill::freezing_op applied: final={}C time={}min",
                  in.final_temp_c, in.freezing_time_min);

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

}  // namespace koocadcam::skill::freezing_op
