// @lat: [[engine/skills#pellet_extrude]]

#include "pellet_extrude.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <string>

namespace koocadcam::skill::pellet_extrude {

using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.polymer.empty()) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": polymer must be non-empty");
    }

    if (in.throughput_kg_h <= 0.0) {
        r.add("DFM-EXT-THROUGHPUT", "error",
              std::string(kSkillId) + ": throughput_kg_h must be > 0 (got " +
              std::to_string(in.throughput_kg_h) + ")");
    } else if (in.throughput_kg_h > 2000.0) {
        r.add("DFM-EXT-THROUGHPUT", "error",
              std::string(kSkillId) + ": throughput_kg_h " +
              std::to_string(in.throughput_kg_h) +
              " > 2000 — beyond single-screw compounder envelope");
    }

    if (in.screw_speed_rpm <= 0.0) {
        r.add("DFM-EXT-SCREW", "error",
              std::string(kSkillId) + ": screw_speed_rpm must be > 0 (got " +
              std::to_string(in.screw_speed_rpm) + ")");
    } else if (in.screw_speed_rpm > 800.0) {
        r.add("DFM-EXT-SCREW", "error",
              std::string(kSkillId) + ": screw_speed_rpm " +
              std::to_string(in.screw_speed_rpm) +
              " > 800 — risks shear-burn / polymer degradation");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = std::string(kSkillId) + " DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    // Rough specific-energy estimate for compounding extrusion: 0.2 kWh/kg
    // for soft polymers, scaling up with screw speed.  This is metadata,
    // not a process simulation.
    const double specEnergy =
        0.15 + 0.0005 * in.screw_speed_rpm;   // kWh / kg, capped soft estimate

    json params = {
        { "polymer",          in.polymer },
        { "throughput_kg_h",  in.throughput_kg_h },
        { "screw_speed_rpm",  in.screw_speed_rpm },
    };
    json pattern = {
        { "kind",                       kSkillId },
        { "polymer",                    in.polymer },
        { "throughput_kg_h",            in.throughput_kg_h },
        { "screw_speed_rpm",            in.screw_speed_rpm },
        { "specific_energy_kwh_kg",     specEnergy },
        { "geometry_changed",           false },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "single_screw_extruder";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "nitrided_steel_barrel";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    // 1 hour cycle as a long continuous run reference.
    tooling.est_cycle_time_s  = 3600.0;
    tooling.extra = {
        { "process",                "polymer_compounding_extrusion" },
        { "polymer",                in.polymer },
        { "throughput_kg_h",        in.throughput_kg_h },
        { "screw_speed_rpm",        in.screw_speed_rpm },
        { "specific_energy_kwh_kg", specEnergy },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::pellet_extrude applied: polymer={} kg/h={} rpm={}",
                  in.polymer, in.throughput_kg_h, in.screw_speed_rpm);

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
        rf.matched_geometry = { { "source", "metadata" } };
        out.push_back(rf);
    }
    return out;
}

}  // namespace koocadcam::skill::pellet_extrude
