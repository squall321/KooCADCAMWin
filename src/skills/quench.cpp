// @lat: [[engine/skills#quench]]

#include "quench.hpp"

#include "_heat_treatment_common.hpp"
#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <string>
#include <tuple>
#include <vector>

namespace koocadcam::skill::quench {

namespace htc = koocadcam::skill::heat_treatment_common;
using nlohmann::json;

namespace {

bool isKnownMedium(const std::string& m)
{
    return m == "water" || m == "oil" || m == "polymer" || m == "air";
}

// Estimate the retained-solid-solution percentage from the medium
// compatibility.  Adequate medium → ~95%; an info-level incompat reduces
// the estimate to ~70% (precipitation reaction starts during the cool).
//
// Note: the htc::MaterialHTData.quench_incompat list mixes two distinct
// concerns:
//   (a) "wrong medium" — cooling rate inadequate to retain the high-temp
//       phase (e.g. aluminum_6061 + oil: oil cools too slowly → ss drops),
//   (b) "right medium with caveats" — adequate cooling rate but with a
//       process-control warning (e.g. aluminum_6061 + water: ss still ~95%
//       but watch fixturing on thick sections).
// The incompat list cannot distinguish them on its own.  We therefore look
// up the (material, medium) pair in an explicit, hand-curated solid-solution
// table FIRST; only if no entry is found do we fall back to the generic
// incompat-as-downgrade heuristic.
double estimateSolidSolutionPct(const std::string& material,
                                const std::string& medium)
{
    // Explicit (material, medium) → solid_solution_pct table.  Entries here
    // override the generic "any incompat → 70%" fallback below.  Add a row
    // here whenever a medium is known to give a specific retained-ss% that
    // differs from the default — including "in the incompat list but still
    // adequate" pairs (e.g. aluminum_6061 + water).
    //
    // NOTE: aluminum_6061 + oil is intentionally OMITTED from this table so
    // the generic "incompat → 70%" path still applies to that pair (matching
    // the canonical "Al6061 + oil drops ss% below the textbook nominal"
    // contract exercised by SkillQuench.IncompatibleMediumDowngradesSSPct).
    static const std::vector<std::tuple<std::string, std::string, double>>
        kSolidSolutionTable = {
            // Aluminum precipitation-hardening alloys
            { "aluminum_6061",     "water", 95.0 },
            { "aluminum_7075",     "water", 95.0 },
            // Stainless solution-anneal: water restores corrosion resistance
            { "stainless_316",     "water", 80.0 },
            // Carbon / alloy steels — quench from austenitize temp
            { "carbon_steel_1045", "water", 90.0 },
            { "alloy_steel_4140",  "oil",   92.0 },
        };

    for (const auto& [mat, med, pct] : kSolidSolutionTable) {
        if (mat == material && med == medium) return pct;
    }

    const auto* m = htc::lookupMaterial(material);
    if (!m) return 90.0;                 // unknown — assume nominal
    for (const auto& tup : m->quench_incompat) {
        const auto& [bad_medium, sev, msg] = tup;
        (void)sev; (void)msg;
        if (bad_medium == medium) return 70.0;
    }
    // 'air' is always slow — even for materials that don't have it flagged
    // as incompat, downgrade the estimate.
    if (medium == "air") return 60.0;
    return 95.0;
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.from_temperature_c <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) +
              ": from_temperature_c must be > 0 (got " +
              std::to_string(in.from_temperature_c) + ")");
    }
    if (in.quench_time_s <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) +
              ": quench_time_s must be > 0 (got " +
              std::to_string(in.quench_time_s) + ")");
    } else if (in.quench_time_s > 60.0) {
        r.add("DFM-HT-SLOW", "info",
              std::string(kSkillId) + ": quench_time_s " +
              std::to_string(in.quench_time_s) +
              " s > 60 s — cool is not 'rapid'; supersaturated solid "
              "solution may not be retained");
    }

    if (!isKnownMedium(in.quench_medium)) {
        r.add("DFM-HT-MEDIUM", "info",
              std::string(kSkillId) + ": quench_medium '" + in.quench_medium +
              "' unrecognised — expected 'water' | 'oil' | 'polymer' | 'air'");
    }

    if (htc::gateMaterialKnown(r, kSkillId, in.material)) {
        const auto* m = htc::lookupMaterial(in.material);
        htc::gateTempRange(r, kSkillId, in.material, "solution",
                           in.from_temperature_c,
                           m->solution_temp_min_c, m->solution_temp_max_c);
        htc::gateQuenchCompat(r, kSkillId, in.material, in.quench_medium);
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "quench DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const double ssPct = estimateSolidSolutionPct(in.material, in.quench_medium);

    json params = {
        { "material",            in.material },
        { "from_temperature_c",  in.from_temperature_c },
        { "quench_medium",       in.quench_medium },
        { "quench_time_s",       in.quench_time_s },
    };
    json pattern = {
        { "kind",                  kSkillId },
        { "material",              in.material },
        { "from_temperature_c",    in.from_temperature_c },
        { "quench_medium",         in.quench_medium },
        { "quench_time_s",         in.quench_time_s },
        { "solid_solution_pct",    ssPct },
        { "geometry_changed",      false },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code", f.code },
                            { "severity", f.severity },
                            { "message", f.message }});

    // Quench cycle is short (seconds, not minutes) — adjust est_cycle_time_s
    // from the default furnace builder.  We still need the ramp + temp soak
    // upstream, but for quench-as-skill we record only the cool step.
    ToolingMeta tooling = htc::buildFurnaceTooling(
        "quench", in.from_temperature_c, /*hold_min*/ 0.0);
    tooling.tool_type        = "quench_tank";
    tooling.tool_material    = "quench_medium";
    tooling.est_cycle_time_s = std::max(1.0, in.quench_time_s);
    tooling.extra["quench_medium"]       = in.quench_medium;
    tooling.extra["quench_time_s"]       = in.quench_time_s;
    tooling.extra["solid_solution_pct"]  = ssPct;
    tooling.extra["dfm_findings"]        = findings;
    tooling.extra["effect"]              = "freeze high-temperature phase";

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = htc::cloneWithSignature(wp, sig);

    spdlog::debug("skill::quench applied: material={} from {}°C medium={} time={}s "
                  "→ ss%≈{}",
                  in.material, in.from_temperature_c, in.quench_medium,
                  in.quench_time_s, ssPct);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    return htc::replayFromFeatures(wp, kSkillId);
}

}  // namespace koocadcam::skill::quench
