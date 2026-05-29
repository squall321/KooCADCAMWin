// @lat: [[engine/skills#laminate_glass]]

#include "laminate_glass.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <string>

namespace koocadcam::skill::laminate_glass {

using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.interlayer_thickness_mm < 0.38 || in.interlayer_thickness_mm > 2.28) {
        r.add("DFM-LAMI-INTERLAYER", "error",
              std::string(kSkillId) + ": interlayer_thickness_mm " +
              std::to_string(in.interlayer_thickness_mm) +
              " outside producible PVB range [0.38, 2.28] mm "
              "(0.38 / 0.76 / 1.52 / 2.28 are the standard plies)");
    }
    if (in.autoclave_temp_c < 120.0 || in.autoclave_temp_c > 160.0) {
        r.add("DFM-LAMI-TEMP", "error",
              std::string(kSkillId) + ": autoclave_temp_c " +
              std::to_string(in.autoclave_temp_c) +
              " outside PVB bonding window [120, 160] °C");
    }
    if (in.autoclave_pressure_kpa < 800.0 || in.autoclave_pressure_kpa > 1500.0) {
        r.add("DFM-LAMI-PRESSURE", "error",
              std::string(kSkillId) + ": autoclave_pressure_kpa " +
              std::to_string(in.autoclave_pressure_kpa) +
              " outside typical laminating range [800, 1500] kPa");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "laminate_glass DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    json params = {
        { "interlayer_thickness_mm",  in.interlayer_thickness_mm },
        { "autoclave_temp_c",         in.autoclave_temp_c },
        { "autoclave_pressure_kpa",   in.autoclave_pressure_kpa },
    };

    json pattern = {
        { "kind",                     kSkillId },
        { "interlayer_thickness_mm",  in.interlayer_thickness_mm },
        { "autoclave_temp_c",         in.autoclave_temp_c },
        { "autoclave_pressure_kpa",   in.autoclave_pressure_kpa },
        { "interlayer_material",      "PVB" },
        { "geometric_change",         false },
        { "process_category",         "glass" },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code", f.code },
                            { "severity", f.severity },
                            { "message", f.message }});

    ToolingMeta tooling;
    tooling.tool_type         = "autoclave";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "steel_pressure_vessel";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    // Lamination autoclave cycle ≈ 4 h (ramp + hold + cool).
    tooling.est_cycle_time_s  = 4.0 * 60.0 * 60.0;
    tooling.extra = {
        { "interlayer_thickness_mm",  in.interlayer_thickness_mm },
        { "autoclave_temp_c",         in.autoclave_temp_c },
        { "autoclave_pressure_kpa",   in.autoclave_pressure_kpa },
        { "interlayer_material",      "PVB" },
        { "dfm_findings",             findings },
        { "process_category",         "laminate_glass" },
        { "geometric_no_op",          true },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug("skill::laminate_glass applied: PVB={}mm T={}°C p={}kPa",
                  in.interlayer_thickness_mm, in.autoclave_temp_c,
                  in.autoclave_pressure_kpa);

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

}  // namespace koocadcam::skill::laminate_glass
