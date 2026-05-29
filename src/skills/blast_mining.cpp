// @lat: [[engine/skills#blast_mining]]

#include "blast_mining.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <string>
#include <unordered_set>

namespace koocadcam::skill::blast_mining {

using nlohmann::json;

namespace {

const std::unordered_set<std::string>& knownExplosives()
{
    static const std::unordered_set<std::string> s = {
        "anfo", "emulsion", "watergel",
    };
    return s;
}

// Nominal energy density (MJ/kg) by explosive type — used for the signature
// metadata.  Source: typical Orica/Dyno engineering tables.
double energyDensityMJperKg(const std::string& explosive_type)
{
    if (explosive_type == "anfo")     return 3.7;
    if (explosive_type == "emulsion") return 3.0;
    if (explosive_type == "watergel") return 3.2;
    return 0.0;
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.pattern_holes <= 0) {
        r.add("DFM-INPUT", "error",
              "blast_mining pattern_holes must be > 0 (got " +
              std::to_string(in.pattern_holes) + ")");
    }

    if (in.charge_per_hole_kg <= 0.0) {
        r.add("DFM-INPUT", "error",
              "blast_mining charge_per_hole_kg must be > 0 (got " +
              std::to_string(in.charge_per_hole_kg) + ")");
    }

    if (in.explosive_type.empty() ||
        knownExplosives().count(in.explosive_type) == 0) {
        r.add("DFM-EXPLOSIVE-TYPE", "error",
              "blast_mining unknown explosive_type '" + in.explosive_type +
              "' (expected anfo | emulsion | watergel)");
    }

    if (in.charge_per_hole_kg > 500.0) {
        r.add("DFM-CHARGE-HIGH", "info",
              "blast_mining charge_per_hole_kg " +
              std::to_string(in.charge_per_hole_kg) +
              " > 500 kg — verify regulatory limits");
    }

    if (in.pattern_holes > 500) {
        r.add("DFM-PATTERN-LARGE", "info",
              "blast_mining pattern_holes " + std::to_string(in.pattern_holes) +
              " > 500 — mega-blast; sequencing + vibration plan required");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "blast_mining DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const double total_charge_kg = in.pattern_holes * in.charge_per_hole_kg;
    const double total_energy_MJ = total_charge_kg * energyDensityMJperKg(in.explosive_type);

    json params = {
        { "pattern_holes",      in.pattern_holes },
        { "charge_per_hole_kg", in.charge_per_hole_kg },
        { "explosive_type",     in.explosive_type },
    };

    json pattern = {
        { "kind",               kSkillId },
        { "pattern_holes",      in.pattern_holes },
        { "charge_per_hole_kg", in.charge_per_hole_kg },
        { "explosive_type",     in.explosive_type },
        { "total_charge_kg",    total_charge_kg },
        { "total_energy_MJ",    total_energy_MJ },
        { "process_family",     "production_blasting" },
        { "geometry_changed",   false },
        { "is_process_only",    true },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code",     f.code },
                            { "severity", f.severity },
                            { "message",  f.message }});

    ToolingMeta tooling;
    if      (in.explosive_type == "anfo")     tooling.tool_type = "anfo_loader";
    else if (in.explosive_type == "emulsion") tooling.tool_type = "emulsion_pump_truck";
    else if (in.explosive_type == "watergel") tooling.tool_type = "watergel_loader";
    else                                       tooling.tool_type = "blast_loader";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = in.explosive_type;
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    // Loading rate ~ 2 holes/min for a single crew; detonation itself instantaneous.
    tooling.est_cycle_time_s  = (in.pattern_holes / 2.0) * 60.0 + 600.0;
    tooling.extra = {
        { "process",          "production_blast" },
        { "pattern_holes",    in.pattern_holes },
        { "explosive_type",   in.explosive_type },
        { "total_charge_kg",  total_charge_kg },
        { "total_energy_MJ",  total_energy_MJ },
        { "process_category", "mining_production" },
        { "dfm_findings",     findings },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::blast_mining applied: holes={} kg/hole={} type={}",
                  in.pattern_holes, in.charge_per_hole_kg, in.explosive_type);

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

}  // namespace koocadcam::skill::blast_mining
