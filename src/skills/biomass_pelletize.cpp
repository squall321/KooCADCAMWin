// @lat: [[engine/skills#biomass_pelletize]]

#include "biomass_pelletize.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <memory>
#include <string>

namespace koocadcam::skill::biomass_pelletize {

using nlohmann::json;

namespace {

bool isKnownBiomass(const std::string& b)
{
    return b == "wood_sawdust" || b == "straw" ||
           b == "rice_husk"    || b == "miscanthus";
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;
    (void)wp;

    if (in.biomass_type.empty()) {
        r.add("DFM-INPUT", "warning",
              std::string(kSkillId) +
              ": biomass_type empty — defaulting to wood_sawdust");
    } else if (!isKnownBiomass(in.biomass_type)) {
        r.add("DFM-PELLET-FEED", "info",
              std::string(kSkillId) + ": biomass_type '" + in.biomass_type +
              "' not in {wood_sawdust, straw, rice_husk, miscanthus}");
    }

    if (in.pellet_dia_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) +
              ": pellet_dia_mm must be > 0 (got " +
              std::to_string(in.pellet_dia_mm) + ")");
    } else if (in.pellet_dia_mm < 4.0 || in.pellet_dia_mm > 14.0) {
        r.add("DFM-PELLET-DIA", "info",
              std::string(kSkillId) + ": pellet_dia_mm " +
              std::to_string(in.pellet_dia_mm) +
              " outside ENplus typical range [4, 14] — non-standard pellet");
    }

    if (in.throughput_kg_h <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) +
              ": throughput_kg_h must be > 0 (got " +
              std::to_string(in.throughput_kg_h) + ")");
    } else if (in.throughput_kg_h > 10000.0) {
        r.add("DFM-PELLET-RATE", "info",
              std::string(kSkillId) + ": throughput_kg_h " +
              std::to_string(in.throughput_kg_h) +
              " > 10000 — unusually large mill, verify die count");
    }

    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "biomass_pelletize DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const std::string biomass =
        in.biomass_type.empty() ? "wood_sawdust" : in.biomass_type;

    json params = {
        { "biomass_type",    biomass },
        { "pellet_dia_mm",   in.pellet_dia_mm },
        { "throughput_kg_h", in.throughput_kg_h },
    };
    json pattern = {
        { "kind",             kSkillId },
        { "biomass_type",     biomass },
        { "pellet_dia_mm",    in.pellet_dia_mm },
        { "throughput_kg_h",  in.throughput_kg_h },
        { "geometry_changed", false },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code", f.code },
                            { "severity", f.severity },
                            { "message", f.message }});

    ToolingMeta tooling;
    tooling.tool_type         = "ring_die_pellet_mill";
    tooling.tool_dia_mm       = in.pellet_dia_mm;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "alloy_steel_X46Cr13";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    // Reporting cycle = 1 h batch (steady-state throughput).
    tooling.est_cycle_time_s  = 3600.0;
    tooling.extra = {
        { "process",          "biomass_pelletize" },
        { "biomass_type",     biomass },
        { "pellet_dia_mm",    in.pellet_dia_mm },
        { "throughput_kg_h",  in.throughput_kg_h },
        { "process_category", "bioenergy_densification" },
        { "geometric_no_op",  true },
        { "dfm_findings",     findings },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug(
        "skill::biomass_pelletize applied: feed={} dia={}mm rate={}kg/h",
        biomass, in.pellet_dia_mm, in.throughput_kg_h);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;
    for (const auto& f : wp.features()) {
        if (f.skill_id != std::string(kSkillId)) continue;
        RecognizedFeature rf;
        rf.skill_id         = kSkillId;
        rf.recovered_params = f.params;
        rf.confidence       = 1.0;
        rf.matched_geometry = { { "source", "metadata" } };
        out.push_back(rf);
    }
    return out;
}

}  // namespace koocadcam::skill::biomass_pelletize
