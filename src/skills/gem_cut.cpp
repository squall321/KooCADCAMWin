// @lat: [[engine/skills#gem_cut]]

#include "gem_cut.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <string>
#include <unordered_set>

namespace koocadcam::skill::gem_cut {

using nlohmann::json;

namespace {

const std::unordered_set<std::string>& knownStyles()
{
    static const std::unordered_set<std::string> s = {
        "round", "oval", "princess",
    };
    return s;
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.gem_species.empty()) {
        r.add("DFM-INPUT", "error",
              "gem_cut gem_species must be non-empty");
    }

    if (in.cut_style.empty()) {
        r.add("DFM-INPUT", "error",
              "gem_cut cut_style must be non-empty");
    } else if (knownStyles().count(in.cut_style) == 0) {
        r.add("DFM-CUT-STYLE", "error",
              "gem_cut cut_style '" + in.cut_style +
              "' not in {round, oval, princess}");
    }

    if (in.carat <= 0.0) {
        r.add("DFM-CARAT", "error",
              "gem_cut carat must be > 0 (got " +
              std::to_string(in.carat) + ")");
    } else {
        if (in.carat < 0.005) {
            r.add("DFM-CARAT-LO", "info",
                  "gem_cut carat " + std::to_string(in.carat) +
                  " < 0.005 — sub-melee size, hand-cut required");
        }
        if (in.carat > 100.0) {
            r.add("DFM-CARAT-HI", "info",
                  "gem_cut carat " + std::to_string(in.carat) +
                  " > 100 — exceptional stone, custom cutting plan recommended");
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
        std::string msg = "gem_cut DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    // 1 ct = 0.2 g; convert to grams for plan-side mass tracking.
    const double mass_g = in.carat * 0.2;
    // Round brilliant carries ~57 facets; oval ~56; princess ~76.
    int facets = 57;
    if      (in.cut_style == "oval")     facets = 56;
    else if (in.cut_style == "princess") facets = 76;

    json params = {
        { "gem_species", in.gem_species },
        { "cut_style",   in.cut_style },
        { "carat",       in.carat },
    };

    json pattern = {
        { "kind",             kSkillId },
        { "gem_species",      in.gem_species },
        { "cut_style",        in.cut_style },
        { "carat",            in.carat },
        { "mass_g",           mass_g },
        { "facet_count",      facets },
        { "process_family",   "gem_faceting" },
        { "geometry_changed", false },
        { "is_process_only",  true },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code",     f.code },
                            { "severity", f.severity },
                            { "message",  f.message }});

    ToolingMeta tooling;
    tooling.tool_type         = "facet_lap";
    tooling.tool_dia_mm       = 200.0;     // typical 8-inch master lap
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "bonded_diamond";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;       // metadata-only at host level
    // Rough cut for round brilliant ~30 min; bigger stones scale linearly.
    tooling.est_cycle_time_s  = 1800.0 + in.carat * 600.0;
    tooling.extra = {
        { "process",          "gem_cutting" },
        { "gem_species",      in.gem_species },
        { "cut_style",        in.cut_style },
        { "carat",            in.carat },
        { "facet_count",      facets },
        { "process_category", "jewelry" },
        { "dfm_findings",     findings },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::gem_cut applied: species={} style={} carat={}",
                  in.gem_species, in.cut_style, in.carat);

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

}  // namespace koocadcam::skill::gem_cut
