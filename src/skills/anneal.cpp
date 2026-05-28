// @lat: [[engine/skills#anneal]]

#include "anneal.hpp"

#include "_heat_treatment_common.hpp"
#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

namespace koocadcam::skill::anneal {

namespace htc = koocadcam::skill::heat_treatment_common;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    htc::gateHoldTime(r, kSkillId, in.hold_time_min);

    if (in.temperature_c <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": temperature_c must be > 0 (got " +
              std::to_string(in.temperature_c) + ")");
    }

    if (in.cooling_rate != "slow_furnace" && in.cooling_rate != "air") {
        r.add("DFM-HT-COOLING", "info",
              std::string(kSkillId) + ": cooling_rate '" + in.cooling_rate +
              "' unrecognised — expected 'slow_furnace' or 'air'");
    }

    if (htc::gateMaterialKnown(r, kSkillId, in.material)) {
        const auto* m = htc::lookupMaterial(in.material);
        htc::gateTempRange(r, kSkillId, in.material, "anneal",
                           in.temperature_c,
                           m->anneal_temp_min_c, m->anneal_temp_max_c);
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "anneal DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const double expectedHB = htc::annealedHB(in.material);

    json params = {
        { "material",       in.material },
        { "temperature_c",  in.temperature_c },
        { "hold_time_min",  in.hold_time_min },
        { "cooling_rate",   in.cooling_rate },
    };
    json pattern = {
        { "kind",                 kSkillId },
        { "material",             in.material },
        { "temperature_c",        in.temperature_c },
        { "hold_time_min",        in.hold_time_min },
        { "cooling_rate",         in.cooling_rate },
        { "expected_hardness_hb", expectedHB },
        { "residual_stress",      "low" },
        { "grain_size",           "coarse" },
        { "geometry_changed",     false },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code", f.code },
                            { "severity", f.severity },
                            { "message", f.message }});

    ToolingMeta tooling = htc::buildFurnaceTooling(
        "anneal", in.temperature_c, in.hold_time_min);
    tooling.extra["cooling_rate"]         = in.cooling_rate;
    tooling.extra["expected_hardness_hb"] = expectedHB;
    tooling.extra["dfm_findings"]         = findings;
    tooling.extra["effect"]               = "soften + stress-relieve";

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = htc::cloneWithSignature(wp, sig);

    spdlog::debug("skill::anneal applied: material={} T={}°C hold={}min cooling={}",
                  in.material, in.temperature_c, in.hold_time_min, in.cooling_rate);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// Anneal leaves no geometric trace.  We can ONLY recognize a prior anneal
// by replaying signatures previously stamped onto the workpiece.

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    return htc::replayFromFeatures(wp, kSkillId);
}

}  // namespace koocadcam::skill::anneal
