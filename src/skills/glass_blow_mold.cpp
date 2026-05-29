// @lat: [[engine/skills#glass_blow_mold]]

#include "glass_blow_mold.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <string>

namespace koocadcam::skill::glass_blow_mold {

using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.mold_id.empty()) {
        r.add("DFM-BLOW-MOLD", "error",
              std::string(kSkillId) + ": mold_id must be non-empty");
    }
    if (in.parison_weight_g <= 0.0 || in.parison_weight_g > 5000.0) {
        r.add("DFM-BLOW-PARISON", "error",
              std::string(kSkillId) + ": parison_weight_g " +
              std::to_string(in.parison_weight_g) +
              " outside operating range (0, 5000] g");
    }
    if (in.blow_pressure_kpa < 50.0 || in.blow_pressure_kpa > 500.0) {
        r.add("DFM-BLOW-PRESSURE", "error",
              std::string(kSkillId) + ": blow_pressure_kpa " +
              std::to_string(in.blow_pressure_kpa) +
              " outside typical bottle-blow range [50, 500] kPa");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "glass_blow_mold DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    json params = {
        { "mold_id",            in.mold_id },
        { "parison_weight_g",   in.parison_weight_g },
        { "blow_pressure_kpa",  in.blow_pressure_kpa },
    };

    json pattern = {
        { "kind",               kSkillId },
        { "mold_id",            in.mold_id },
        { "parison_weight_g",   in.parison_weight_g },
        { "blow_pressure_kpa",  in.blow_pressure_kpa },
        { "geometric_change",   false },
        { "process_category",   "glass" },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code", f.code },
                            { "severity", f.severity },
                            { "message", f.message }});

    ToolingMeta tooling;
    tooling.tool_type         = "blow_mold";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "cast_iron";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    // IS-machine bottle cycle ≈ 10 s per cavity.
    tooling.est_cycle_time_s  = 10.0;
    tooling.extra = {
        { "mold_id",            in.mold_id },
        { "parison_weight_g",   in.parison_weight_g },
        { "blow_pressure_kpa",  in.blow_pressure_kpa },
        { "dfm_findings",       findings },
        { "process_category",   "glass_blow_mold" },
        { "geometric_no_op",    true },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug("skill::glass_blow_mold applied: mold={} parison={}g p={}kPa",
                  in.mold_id, in.parison_weight_g, in.blow_pressure_kpa);

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

}  // namespace koocadcam::skill::glass_blow_mold
