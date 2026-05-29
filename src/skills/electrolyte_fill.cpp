// @lat: [[engine/skills#electrolyte_fill]]

#include "electrolyte_fill.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

namespace koocadcam::skill::electrolyte_fill {

using nlohmann::json;

namespace {

bool isKnownElectrolyte(const std::string& e)
{
    return e == "LiPF6_EC_DMC"     ||      // 1 M LiPF6 in EC/DMC (workhorse)
           e == "LiPF6_EC_EMC"     ||
           e == "LiPF6_EC_DEC"     ||
           e == "LiFSI_EMC"        ||      // high-voltage NMC
           e == "LiTFSI_DOL_DME"   ||      // Li-S chemistries
           e == "LiBOB_EC_PC"      ||
           e == "ionic_liquid";            // emerging solid-state precursor
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.volume_ml <= 0.0) {
        r.add("DFM-INPUT", "error",
              "electrolyte_fill: volume_ml must be > 0 (got " +
              std::to_string(in.volume_ml) + ")");
    } else {
        if (in.volume_ml < 0.5) {
            r.add("DFM-FILL-VOL-SMALL", "error",
                  "electrolyte_fill: volume_ml " +
                  std::to_string(in.volume_ml) +
                  " < 0.5 mL — below smallest pouch-cell dose");
        }
        if (in.volume_ml > 200.0) {
            r.add("DFM-FILL-VOL-LARGE", "error",
                  "electrolyte_fill: volume_ml " +
                  std::to_string(in.volume_ml) +
                  " > 200 mL — larger than 100-Ah prismatic cell budget");
        }
    }

    if (in.vacuum_pressure_mbar <= 0.0) {
        r.add("DFM-FILL-VAC-LOW", "error",
              "electrolyte_fill: vacuum_pressure_mbar must be > 0 (got " +
              std::to_string(in.vacuum_pressure_mbar) + ")");
    } else if (in.vacuum_pressure_mbar > 200.0) {
        r.add("DFM-FILL-VAC-HIGH", "info",
              "electrolyte_fill: vacuum_pressure_mbar " +
              std::to_string(in.vacuum_pressure_mbar) +
              " > 200 mbar — insufficient vacuum; air-pocket risk");
    }

    if (!isKnownElectrolyte(in.electrolyte_type)) {
        r.add("DFM-FILL-ELECTROLYTE", "info",
              "electrolyte_fill: electrolyte_type '" + in.electrolyte_type +
              "' not in standard chemistry table");
    }

    if (in.fill_temperature_c < 10.0 || in.fill_temperature_c > 40.0) {
        r.add("DFM-FILL-TEMP", "info",
              "electrolyte_fill: fill_temperature_c " +
              std::to_string(in.fill_temperature_c) +
              " outside typical [10, 40] °C window");
    }

    if (in.dwell_time_min < 0.0) {
        r.add("DFM-INPUT", "error",
              "electrolyte_fill: dwell_time_min must be >= 0");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "electrolyte_fill DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    json params = {
        { "volume_ml",            in.volume_ml },
        { "electrolyte_type",     in.electrolyte_type },
        { "vacuum_pressure_mbar", in.vacuum_pressure_mbar },
        { "fill_temperature_c",   in.fill_temperature_c },
        { "dwell_time_min",       in.dwell_time_min },
    };
    json pattern = {
        { "kind",                 kSkillId },
        { "volume_ml",            in.volume_ml },
        { "electrolyte_type",     in.electrolyte_type },
        { "vacuum_pressure_mbar", in.vacuum_pressure_mbar },
        { "fill_temperature_c",   in.fill_temperature_c },
        { "dwell_time_min",       in.dwell_time_min },
        { "geometry_changed",     false },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "vacuum_fill_nozzle";
    tooling.tool_dia_mm       = 1.2;        // typical fill nozzle ID
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "PTFE_lined_stainless";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    // Cycle: pump-down (~ 60 s) + dose (~ 10 s) + dwell (input).
    tooling.est_cycle_time_s  = 60.0 + 10.0 + in.dwell_time_min * 60.0;
    tooling.extra = {
        { "process",              "vacuum_electrolyte_inject" },
        { "volume_ml",            in.volume_ml },
        { "electrolyte_type",     in.electrolyte_type },
        { "vacuum_pressure_mbar", in.vacuum_pressure_mbar },
        { "process_category",     "battery_manufacturing" },
        { "geometric_no_op",      true },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug(
        "skill::electrolyte_fill applied: vol={}mL type={} vac={}mbar",
        in.volume_ml, in.electrolyte_type, in.vacuum_pressure_mbar);

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

}  // namespace koocadcam::skill::electrolyte_fill
