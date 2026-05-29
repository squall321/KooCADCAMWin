// @lat: [[engine/skills#float_glass]]

#include "float_glass.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <string>

namespace koocadcam::skill::float_glass {

using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.glass_thickness_mm < 0.4 || in.glass_thickness_mm > 25.0) {
        r.add("DFM-FLOAT-THICK", "error",
              std::string(kSkillId) + ": glass_thickness_mm " +
              std::to_string(in.glass_thickness_mm) +
              " outside Pilkington float range [0.4, 25.0] mm");
    }
    if (in.ribbon_speed_m_min <= 0.0 || in.ribbon_speed_m_min > 50.0) {
        r.add("DFM-FLOAT-SPEED", "error",
              std::string(kSkillId) + ": ribbon_speed_m_min " +
              std::to_string(in.ribbon_speed_m_min) +
              " outside operating range (0, 50] m/min");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "float_glass DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    json params = {
        { "glass_thickness_mm",  in.glass_thickness_mm },
        { "ribbon_speed_m_min",  in.ribbon_speed_m_min },
    };

    json pattern = {
        { "kind",                kSkillId },
        { "glass_thickness_mm",  in.glass_thickness_mm },
        { "ribbon_speed_m_min",  in.ribbon_speed_m_min },
        { "geometric_change",    false },
        { "process_category",    "glass" },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code", f.code },
                            { "severity", f.severity },
                            { "message", f.message }});

    ToolingMeta tooling;
    tooling.tool_type         = "tin_bath";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "molten_tin";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    // Float line is continuous; per-square-metre dwell on tin bath ≈ 60 s
    // at nominal speed.
    tooling.est_cycle_time_s  = 60.0;
    tooling.extra = {
        { "glass_thickness_mm",  in.glass_thickness_mm },
        { "ribbon_speed_m_min",  in.ribbon_speed_m_min },
        { "dfm_findings",        findings },
        { "process_category",    "float_glass" },
        { "geometric_no_op",     true },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug("skill::float_glass applied: thickness={}mm speed={}m/min",
                  in.glass_thickness_mm, in.ribbon_speed_m_min);

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

}  // namespace koocadcam::skill::float_glass
