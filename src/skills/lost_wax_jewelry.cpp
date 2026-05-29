// @lat: [[engine/skills#lost_wax_jewelry]]

#include "lost_wax_jewelry.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <string>

namespace koocadcam::skill::lost_wax_jewelry {

using nlohmann::json;

namespace {

// Density (g/cm³) of common jewelry alloys.  Wax is ~0.95 g/cm³, so
// metal-mass = wax_mass × (alloy_density / wax_density).
double densityFor(const std::string& alloy)
{
    if (alloy == "Au24K") return 19.30;
    if (alloy == "Au22K") return 17.80;
    if (alloy == "Au18K") return 15.60;
    if (alloy == "Au14K") return 13.10;
    if (alloy == "Au10K") return 11.60;
    if (alloy == "Ag999") return 10.49;
    if (alloy == "Ag925") return 10.36;
    if (alloy == "Pt999") return 21.45;
    if (alloy == "Pt950") return 20.10;
    if (alloy == "Pd950") return 11.80;
    return 12.00;  // generic / unknown
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.alloy.empty()) {
        r.add("DFM-INPUT", "error",
              "lost_wax_jewelry alloy must be non-empty");
    }

    if (in.wax_weight_g <= 0.0) {
        r.add("DFM-WAX-WEIGHT", "error",
              "lost_wax_jewelry wax_weight_g must be > 0 (got " +
              std::to_string(in.wax_weight_g) + ")");
    } else {
        if (in.wax_weight_g < 0.05) {
            r.add("DFM-WAX-WEIGHT-LO", "info",
                  "lost_wax_jewelry wax_weight_g " +
                  std::to_string(in.wax_weight_g) +
                  " < 0.05 g — micro-pattern, vacuum-assist casting needed");
        }
        if (in.wax_weight_g > 200.0) {
            r.add("DFM-WAX-WEIGHT-HI", "info",
                  "lost_wax_jewelry wax_weight_g " +
                  std::to_string(in.wax_weight_g) +
                  " > 200 g — large tree, switch from centrifugal to gravity pour");
        }
    }

    if (in.sprue_count <= 0) {
        r.add("DFM-SPRUE-COUNT", "error",
              "lost_wax_jewelry sprue_count must be > 0 (got " +
              std::to_string(in.sprue_count) + ")");
    } else if (in.sprue_count > 24) {
        r.add("DFM-SPRUE-MANY", "info",
              "lost_wax_jewelry sprue_count " +
              std::to_string(in.sprue_count) +
              " > 24 — overcrowded tree, fill-yield risk");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "lost_wax_jewelry DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const double wax_density   = 0.95;          // g/cm³
    const double alloy_density = densityFor(in.alloy);
    const double expected_metal_g =
        in.wax_weight_g * (alloy_density / wax_density);

    json params = {
        { "alloy",         in.alloy },
        { "wax_weight_g",  in.wax_weight_g },
        { "sprue_count",   in.sprue_count },
    };

    json pattern = {
        { "kind",             kSkillId },
        { "alloy",            in.alloy },
        { "wax_weight_g",     in.wax_weight_g },
        { "sprue_count",      in.sprue_count },
        { "alloy_density",    alloy_density },
        { "expected_metal_g", expected_metal_g },
        { "process_family",   "investment_casting_jewelry" },
        { "geometry_changed", false },
        { "is_process_only",  true },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code",     f.code },
                            { "severity", f.severity },
                            { "message",  f.message }});

    ToolingMeta tooling;
    tooling.tool_type         = "centrifugal_caster";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "plaster_investment";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    // Burnout cycle ~8 h + cast ~5 min + cool ~2 h ≈ 36000 s.
    tooling.est_cycle_time_s  = 36000.0;
    tooling.extra = {
        { "process",          "lost_wax_casting" },
        { "alloy",            in.alloy },
        { "wax_weight_g",     in.wax_weight_g },
        { "sprue_count",      in.sprue_count },
        { "expected_metal_g", expected_metal_g },
        { "process_category", "jewelry" },
        { "dfm_findings",     findings },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::lost_wax_jewelry applied: alloy={} wax_g={} sprues={}",
                  in.alloy, in.wax_weight_g, in.sprue_count);

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

}  // namespace koocadcam::skill::lost_wax_jewelry
