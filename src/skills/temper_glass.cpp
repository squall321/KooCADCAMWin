// @lat: [[engine/skills#temper_glass]]

#include "temper_glass.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <string>

namespace koocadcam::skill::temper_glass {

using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.peak_temp_c < 600.0 || in.peak_temp_c > 700.0) {
        r.add("DFM-TEMPER-PEAK", "error",
              std::string(kSkillId) + ": peak_temp_c " +
              std::to_string(in.peak_temp_c) +
              " outside soda-lime temper range [600, 700] °C");
    }
    if (in.quench_temp_c < 0.0 || in.quench_temp_c > 200.0 ||
        in.quench_temp_c >= in.peak_temp_c) {
        r.add("DFM-TEMPER-QUENCH", "error",
              std::string(kSkillId) + ": quench_temp_c " +
              std::to_string(in.quench_temp_c) +
              " must be ∈ [0, 200] °C and < peak_temp_c " +
              std::to_string(in.peak_temp_c));
    }
    if (in.surface_stress_mpa < 40.0 || in.surface_stress_mpa > 200.0) {
        r.add("DFM-TEMPER-STRESS", "error",
              std::string(kSkillId) + ": surface_stress_mpa " +
              std::to_string(in.surface_stress_mpa) +
              " outside producible range [40, 200] MPa");
    } else if (in.surface_stress_mpa < 69.0) {
        r.add("DFM-TEMPER-STRESS", "info",
              std::string(kSkillId) + ": surface_stress_mpa " +
              std::to_string(in.surface_stress_mpa) +
              " < 69 MPa — does NOT meet ANSI Z97.1 / EN 12150 'fully tempered' "
              "threshold (heat-strengthened only)");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "temper_glass DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const bool fully_tempered = (in.surface_stress_mpa >= 69.0);

    json params = {
        { "peak_temp_c",         in.peak_temp_c },
        { "quench_temp_c",       in.quench_temp_c },
        { "surface_stress_mpa",  in.surface_stress_mpa },
    };

    json pattern = {
        { "kind",                kSkillId },
        { "peak_temp_c",         in.peak_temp_c },
        { "quench_temp_c",       in.quench_temp_c },
        { "surface_stress_mpa",  in.surface_stress_mpa },
        { "fully_tempered",      fully_tempered },
        { "geometric_change",    false },
        { "process_category",    "glass" },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code", f.code },
                            { "severity", f.severity },
                            { "message", f.message }});

    ToolingMeta tooling;
    tooling.tool_type         = "tempering_furnace";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "refractory_lining";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    // Heat (~ 3 min per mm) + quench (~ 1 min); rough cycle 5 min.
    tooling.est_cycle_time_s  = 5.0 * 60.0;
    tooling.extra = {
        { "peak_temp_c",         in.peak_temp_c },
        { "quench_temp_c",       in.quench_temp_c },
        { "surface_stress_mpa",  in.surface_stress_mpa },
        { "fully_tempered",      fully_tempered },
        { "dfm_findings",        findings },
        { "process_category",    "temper_glass" },
        { "geometric_no_op",     true },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug("skill::temper_glass applied: T_peak={}°C T_quench={}°C σ={}MPa",
                  in.peak_temp_c, in.quench_temp_c, in.surface_stress_mpa);

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

}  // namespace koocadcam::skill::temper_glass
