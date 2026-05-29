// @lat: [[engine/skills#glass_melt]]

#include "glass_melt.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <string>

namespace koocadcam::skill::glass_melt {

using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.melt_temp_c < 1100.0 || in.melt_temp_c > 1700.0) {
        r.add("DFM-MELT-TEMP", "error",
              std::string(kSkillId) + ": melt_temp_c " +
              std::to_string(in.melt_temp_c) +
              " outside typical glass melt range [1100, 1700] °C");
    }
    if (in.fining_temp_c < in.melt_temp_c || in.fining_temp_c > 1700.0) {
        r.add("DFM-MELT-FINING", "error",
              std::string(kSkillId) + ": fining_temp_c " +
              std::to_string(in.fining_temp_c) +
              " must be ≥ melt_temp_c and ≤ 1700 °C (got melt=" +
              std::to_string(in.melt_temp_c) + ")");
    }
    if (!in.batch_composition.is_object() || in.batch_composition.empty()) {
        r.add("DFM-MELT-BATCH", "error",
              std::string(kSkillId) +
              ": batch_composition must be a non-empty JSON object "
              "(oxide → wt.% mapping)");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "glass_melt DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    json params = {
        { "melt_temp_c",       in.melt_temp_c },
        { "fining_temp_c",     in.fining_temp_c },
        { "batch_composition", in.batch_composition },
    };

    json pattern = {
        { "kind",               kSkillId },
        { "melt_temp_c",        in.melt_temp_c },
        { "fining_temp_c",      in.fining_temp_c },
        { "batch_composition",  in.batch_composition },
        { "geometric_change",   false },
        { "process_category",   "glass" },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code", f.code },
                            { "severity", f.severity },
                            { "message", f.message }});

    ToolingMeta tooling;
    tooling.tool_type         = "glass_furnace";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "refractory_lining";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    // Glass tank melt = continuous; per-batch fictive cycle ≈ 24 h.
    tooling.est_cycle_time_s  = 24.0 * 60.0 * 60.0;
    tooling.extra = {
        { "melt_temp_c",        in.melt_temp_c },
        { "fining_temp_c",      in.fining_temp_c },
        { "batch_composition",  in.batch_composition },
        { "dfm_findings",       findings },
        { "process_category",   "glass_melt" },
        { "geometric_no_op",    true },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug("skill::glass_melt applied: T_melt={}°C T_fine={}°C",
                  in.melt_temp_c, in.fining_temp_c);

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

}  // namespace koocadcam::skill::glass_melt
