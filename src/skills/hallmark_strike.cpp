// @lat: [[engine/skills#hallmark_strike]]

#include "hallmark_strike.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <string>
#include <unordered_set>

namespace koocadcam::skill::hallmark_strike {

using nlohmann::json;

namespace {

// Recognised millesimal fineness markings.
const std::unordered_set<int>& knownPurities()
{
    static const std::unordered_set<int> s = {
        333, 375, 585, 750, 800, 925, 950, 990, 999,
    };
    return s;
}

std::string purityFamily(int purity)
{
    if (purity == 333 || purity == 375 || purity == 585
     || purity == 750 || purity == 990 || purity == 999) return "gold_or_finest";
    if (purity == 800 || purity == 925)                  return "silver";
    if (purity == 950)                                   return "platinum_or_palladium";
    return "unknown";
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.hallmark_id.empty()) {
        r.add("DFM-INPUT", "error",
              "hallmark_strike hallmark_id must be non-empty");
    }

    if (knownPurities().count(in.alloy_purity) == 0) {
        r.add("DFM-PURITY", "error",
              "hallmark_strike alloy_purity " +
              std::to_string(in.alloy_purity) +
              " not in {333,375,585,750,800,925,950,990,999}");
    }

    if (in.force_kn <= 0.0) {
        r.add("DFM-FORCE", "error",
              "hallmark_strike force_kn must be > 0 (got " +
              std::to_string(in.force_kn) + ")");
    } else {
        if (in.force_kn < 0.2) {
            r.add("DFM-FORCE-LO", "info",
                  "hallmark_strike force_kn " + std::to_string(in.force_kn) +
                  " < 0.2 — likely under-impressed, mark may be illegible");
        }
        if (in.force_kn > 5.0) {
            r.add("DFM-FORCE-HI", "info",
                  "hallmark_strike force_kn " + std::to_string(in.force_kn) +
                  " > 5.0 — punch-through risk on thin section");
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
        std::string msg = "hallmark_strike DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const std::string family = purityFamily(in.alloy_purity);

    json params = {
        { "hallmark_id",  in.hallmark_id },
        { "alloy_purity", in.alloy_purity },
        { "force_kn",     in.force_kn },
    };

    json pattern = {
        { "kind",             kSkillId },
        { "hallmark_id",      in.hallmark_id },
        { "alloy_purity",     in.alloy_purity },
        { "force_kn",         in.force_kn },
        { "purity_family",    family },
        { "process_family",   "assay_hallmarking" },
        { "geometry_changed", false },
        { "is_process_only",  true },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code",     f.code },
                            { "severity", f.severity },
                            { "message",  f.message }});

    ToolingMeta tooling;
    tooling.tool_type         = "hallmark_punch";
    tooling.tool_dia_mm       = 1.0;            // typical mark Ø ≈ 0.6–1.5 mm
    tooling.tool_length_mm    = 40.0;
    tooling.tool_material     = "hardened_tool_steel";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    // Hand-press strike ~2 s per mark.
    tooling.est_cycle_time_s  = 2.0;
    tooling.extra = {
        { "process",          "hallmark_strike" },
        { "hallmark_id",      in.hallmark_id },
        { "alloy_purity",     in.alloy_purity },
        { "purity_family",    family },
        { "force_kn",         in.force_kn },
        { "process_category", "jewelry" },
        { "dfm_findings",     findings },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::hallmark_strike applied: id={} purity={} F={}kN",
                  in.hallmark_id, in.alloy_purity, in.force_kn);

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

}  // namespace koocadcam::skill::hallmark_strike
