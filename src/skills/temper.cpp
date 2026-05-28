// @lat: [[engine/skills#temper]]

#include "temper.hpp"

#include "_heat_treatment_common.hpp"
#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

namespace koocadcam::skill::temper {

namespace htc = koocadcam::skill::heat_treatment_common;
using nlohmann::json;

namespace {

// Industry-standard tempering window.
constexpr double kTemperMinC = 150.0;
constexpr double kTemperMaxC = 700.0;

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    htc::gateHoldTime(r, kSkillId, in.hold_time_min);

    if (in.temperature_c <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) +
              ": temperature_c must be > 0 (got " +
              std::to_string(in.temperature_c) + ")");
    } else if (in.temperature_c < kTemperMinC || in.temperature_c > kTemperMaxC) {
        r.add("DFM-HT-TEMPER-RANGE", "info",
              std::string(kSkillId) + ": temperature_c " +
              std::to_string(in.temperature_c) +
              " °C outside typical temper window [150, 700] °C — "
              "below 150 °C achieves negligible relief; above 700 °C "
              "approaches the lower critical temperature");
    }

    if (htc::gateMaterialKnown(r, kSkillId, in.material)) {
        const auto* m = htc::lookupMaterial(in.material);
        if (m->hardened_hrc_max <= 0.0) {
            r.add("DFM-HT-NOT-TEMPERABLE", "info",
                  std::string(kSkillId) + ": material '" + in.material +
                  "' is not quench-hardenable — temper cycle has no measurable "
                  "effect (no martensite to relieve)");
        }
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "temper DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const auto hardenedPair = htc::hardenedHRC(in.material);
    const double hrcMax     = hardenedPair.first;
    const double postHrc    = htc::temperedHRC(hrcMax, in.temperature_c);

    json params = {
        { "material",       in.material },
        { "temperature_c",  in.temperature_c },
        { "hold_time_min",  in.hold_time_min },
    };
    json pattern = {
        { "kind",                kSkillId },
        { "material",            in.material },
        { "temperature_c",       in.temperature_c },
        { "hold_time_min",       in.hold_time_min },
        { "post_temper_hrc",     postHrc },
        { "toughness",           "improved" },
        { "geometry_changed",    false },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code", f.code },
                            { "severity", f.severity },
                            { "message", f.message }});

    ToolingMeta tooling = htc::buildFurnaceTooling(
        "temper", in.temperature_c, in.hold_time_min);
    tooling.extra["post_temper_hrc"] = postHrc;
    tooling.extra["dfm_findings"]    = findings;
    tooling.extra["effect"]          = "trade hardness for toughness";

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = htc::cloneWithSignature(wp, sig);

    spdlog::debug("skill::temper applied: material={} T={}°C hold={}min → HRC≈{}",
                  in.material, in.temperature_c, in.hold_time_min, postHrc);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    return htc::replayFromFeatures(wp, kSkillId);
}

}  // namespace koocadcam::skill::temper
