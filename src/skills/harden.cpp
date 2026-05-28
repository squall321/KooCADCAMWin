// @lat: [[engine/skills#harden]]

#include "harden.hpp"

#include "_heat_treatment_common.hpp"
#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

namespace koocadcam::skill::harden {

namespace htc = koocadcam::skill::heat_treatment_common;
using nlohmann::json;

namespace {

// Known quench-medium set.  Anything outside this triggers a (info-level)
// DFM finding — operations still proceed because shop-floor recipes do
// invent custom media for special alloys.
bool isKnownMedium(const std::string& m)
{
    return m == "water" || m == "oil" || m == "polymer" || m == "air";
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    htc::gateHoldTime(r, kSkillId, in.hold_time_min);

    if (in.austenitize_temperature_c <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) +
              ": austenitize_temperature_c must be > 0 (got " +
              std::to_string(in.austenitize_temperature_c) + ")");
    }

    if (!isKnownMedium(in.quench_medium)) {
        r.add("DFM-HT-MEDIUM", "info",
              std::string(kSkillId) + ": quench_medium '" + in.quench_medium +
              "' unrecognised — expected 'water' | 'oil' | 'polymer' | 'air'");
    }

    if (htc::gateMaterialKnown(r, kSkillId, in.material)) {
        const auto* m = htc::lookupMaterial(in.material);
        htc::gateTempRange(r, kSkillId, in.material, "austenitize",
                           in.austenitize_temperature_c,
                           m->austenitize_temp_min_c, m->austenitize_temp_max_c);
        htc::gateQuenchCompat(r, kSkillId, in.material, in.quench_medium);

        // Specific class-level guidance: non-ferrous materials with HRC = 0
        // (stainless_316, brass_360, pure aluminium) cannot be quench-hardened.
        if (m->hardened_hrc_max <= 0.0) {
            r.add("DFM-HT-NOT-HARDENABLE", "info",
                  std::string(kSkillId) + ": material '" + in.material +
                  "' is NOT quench-hardenable (no martensitic transformation) — "
                  "consider 'normalize' or 'anneal' instead");
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
        std::string msg = "harden DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const auto [expectedHRC, martPct] = htc::hardenedHRC(in.material);

    json params = {
        { "material",                   in.material },
        { "austenitize_temperature_c",  in.austenitize_temperature_c },
        { "hold_time_min",              in.hold_time_min },
        { "quench_medium",              in.quench_medium },
    };
    json pattern = {
        { "kind",                          kSkillId },
        { "material",                      in.material },
        { "austenitize_temperature_c",     in.austenitize_temperature_c },
        { "hold_time_min",                 in.hold_time_min },
        { "quench_medium",                 in.quench_medium },
        { "expected_hardness_hrc",         expectedHRC },
        { "martensite_pct",                martPct },
        { "geometry_changed",              false },
        { "requires_temper",               true },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code", f.code },
                            { "severity", f.severity },
                            { "message", f.message }});

    ToolingMeta tooling = htc::buildFurnaceTooling(
        "harden", in.austenitize_temperature_c, in.hold_time_min);
    tooling.extra["quench_medium"]          = in.quench_medium;
    tooling.extra["expected_hardness_hrc"]  = expectedHRC;
    tooling.extra["martensite_pct"]         = martPct;
    tooling.extra["dfm_findings"]           = findings;
    tooling.extra["effect"]                 = "harden via martensitic transformation";
    tooling.extra["followup_recommended"]   = "temper";

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = htc::cloneWithSignature(wp, sig);

    spdlog::debug("skill::harden applied: material={} T_aust={}°C hold={}min "
                  "quench={} → HRC≈{}",
                  in.material, in.austenitize_temperature_c, in.hold_time_min,
                  in.quench_medium, expectedHRC);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    return htc::replayFromFeatures(wp, kSkillId);
}

}  // namespace koocadcam::skill::harden
