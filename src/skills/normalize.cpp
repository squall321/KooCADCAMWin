// @lat: [[engine/skills#normalize]]

#include "normalize.hpp"

#include "_heat_treatment_common.hpp"
#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

namespace koocadcam::skill::normalize {

namespace htc = koocadcam::skill::heat_treatment_common;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    htc::gateHoldTime(r, kSkillId, in.hold_time_min);

    if (in.normalize_temperature_c <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) +
              ": normalize_temperature_c must be > 0 (got " +
              std::to_string(in.normalize_temperature_c) + ")");
    }

    if (htc::gateMaterialKnown(r, kSkillId, in.material)) {
        const auto* m = htc::lookupMaterial(in.material);
        htc::gateTempRange(r, kSkillId, in.material, "normalize",
                           in.normalize_temperature_c,
                           m->normalize_temp_min_c, m->normalize_temp_max_c);
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "normalize DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    json params = {
        { "material",                in.material },
        { "normalize_temperature_c", in.normalize_temperature_c },
        { "hold_time_min",           in.hold_time_min },
    };
    json pattern = {
        { "kind",                       kSkillId },
        { "material",                   in.material },
        { "normalize_temperature_c",    in.normalize_temperature_c },
        { "hold_time_min",              in.hold_time_min },
        { "cooling_rate",               "air" },
        { "expected_grain_size",        "fine" },
        { "residual_stress",            "low" },
        { "geometry_changed",           false },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code", f.code },
                            { "severity", f.severity },
                            { "message", f.message }});

    ToolingMeta tooling = htc::buildFurnaceTooling(
        "normalize", in.normalize_temperature_c, in.hold_time_min);
    tooling.extra["cooling_rate"]        = "air";
    tooling.extra["expected_grain_size"] = "fine";
    tooling.extra["dfm_findings"]        = findings;
    tooling.extra["effect"]              = "homogenize microstructure (fine grain)";

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = htc::cloneWithSignature(wp, sig);

    spdlog::debug("skill::normalize applied: material={} T={}°C hold={}min",
                  in.material, in.normalize_temperature_c, in.hold_time_min);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    return htc::replayFromFeatures(wp, kSkillId);
}

}  // namespace koocadcam::skill::normalize
