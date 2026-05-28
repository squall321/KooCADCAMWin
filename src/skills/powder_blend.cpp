// @lat: [[engine/skills#powder_blend]]

#include "powder_blend.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <cmath>
#include <string>

namespace koocadcam::skill::powder_blend {

using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.components.empty()) {
        r.add("DFM-PB-COMPONENTS", "error",
              std::string(kSkillId) + ": at least 1 component required (got 0)");
        (void)wp;
        return r;
    }

    double sum = 0.0;
    for (size_t i = 0; i < in.components.size(); ++i) {
        const auto& c = in.components[i];
        if (c.material.empty()) {
            r.add("DFM-PB-MAT-EMPTY", "error",
                  std::string(kSkillId) + ": component[" +
                  std::to_string(i) + "].material is empty");
        }
        if (c.fraction <= 0.0 || c.fraction > 1.0) {
            r.add("DFM-PB-FRAC", "error",
                  std::string(kSkillId) + ": component[" +
                  std::to_string(i) + "].fraction must be in (0, 1], got " +
                  std::to_string(c.fraction));
        }
        sum += c.fraction;
    }

    if (std::fabs(sum - 1.0) > 1e-3) {
        r.add("DFM-PB-FRAC", "error",
              std::string(kSkillId) + ": fractions must sum to 1.0 ± 1e-3 (got " +
              std::to_string(sum) + ")");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = std::string(kSkillId) + " DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    // Build component JSON array for both params + pattern.
    json compsJson = json::array();
    for (const auto& c : in.components) {
        compsJson.push_back({
            { "material", c.material },
            { "fraction", c.fraction },
        });
    }

    json params = {
        { "components", compsJson },
    };

    json pattern = {
        { "kind",               kSkillId },
        { "is_process_only",    true },
        { "geometric_change",   false },
        { "process_family",     "powder_metallurgy" },
        { "process_subtype",    "blend" },
        { "components",         compsJson },
        { "component_count",    static_cast<int>(in.components.size()) },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code",     f.code },
                            { "severity", f.severity },
                            { "message",  f.message }});

    ToolingMeta tooling;
    tooling.tool_type         = "v_blender";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "stainless_steel_316l_chamber";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    // Blending cycle ~ 30 min for a typical batch.
    tooling.est_cycle_time_s  = 30.0 * 60.0;
    tooling.extra = {
        { "process",         "powder_blend" },
        { "process_family",  "powder_metallurgy" },
        { "components",      compsJson },
        { "component_count", static_cast<int>(in.components.size()) },
        { "dfm_findings",    findings },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug("skill::powder_blend applied: components={}",
                  in.components.size());

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

}  // namespace koocadcam::skill::powder_blend
