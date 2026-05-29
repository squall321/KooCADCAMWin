// @lat: [[engine/skills#thermal_vacuum_test]]

#include "thermal_vacuum_test.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <string>

namespace koocadcam::skill::thermal_vacuum_test {

using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.chamber_pressure_torr <= 0.0) {
        r.add("DFM-SPACE-VAC", "error",
              "thermal_vacuum_test chamber_pressure_torr " +
              std::to_string(in.chamber_pressure_torr) +
              " must be > 0");
    } else if (in.chamber_pressure_torr > 1.0e-3) {
        r.add("DFM-SPACE-VAC", "error",
              "thermal_vacuum_test chamber_pressure_torr " +
              std::to_string(in.chamber_pressure_torr) +
              " > 1e-3 torr — insufficient vacuum for orbital TVAC qualification");
    }

    if (in.temp_range_c <= 0.0) {
        r.add("DFM-SPACE-TEMP", "error",
              "thermal_vacuum_test temp_range_c " +
              std::to_string(in.temp_range_c) +
              " must be > 0");
    } else if (in.temp_range_c > 400.0) {
        r.add("DFM-SPACE-TEMP", "error",
              "thermal_vacuum_test temp_range_c " +
              std::to_string(in.temp_range_c) +
              " > 400 °C — exceeds standard TVAC chamber excursion envelope");
    }

    if (in.cycles < 1) {
        r.add("DFM-SPACE-CYC", "error",
              "thermal_vacuum_test cycles " + std::to_string(in.cycles) +
              " < 1 — at least one thermal cycle required");
    } else if (in.cycles > 50) {
        r.add("DFM-SPACE-CYC", "error",
              "thermal_vacuum_test cycles " + std::to_string(in.cycles) +
              " > 50 — exceeds standard qualification campaign envelope");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "thermal_vacuum_test DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    json params = {
        { "chamber_pressure_torr", in.chamber_pressure_torr },
        { "temp_range_c",          in.temp_range_c },
        { "cycles",                in.cycles },
    };
    json pattern = {
        { "kind",                  kSkillId },
        { "chamber_pressure_torr", in.chamber_pressure_torr },
        { "temp_range_c",          in.temp_range_c },
        { "cycles",                in.cycles },
        { "qualification_level",
          in.cycles >= 8 ? "qualification" : "acceptance" },
        { "geometry_changed",      false },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "tvac_chamber";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "stainless_steel_304";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    // Rough budget: 12 hours per cycle (pumpdown + soak + ramp + dwell).
    tooling.est_cycle_time_s  =
        in.cycles * 12.0 * 3600.0;
    tooling.extra = {
        { "process",               "thermal_vacuum_qual" },
        { "chamber_pressure_torr", in.chamber_pressure_torr },
        { "temp_range_c",          in.temp_range_c },
        { "cycles",                in.cycles },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::thermal_vacuum_test applied: P={}torr dT={}C cycles={}",
                  in.chamber_pressure_torr, in.temp_range_c, in.cycles);

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

}  // namespace koocadcam::skill::thermal_vacuum_test
