// @lat: [[engine/skills#cell_formation]]

#include "cell_formation.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

namespace koocadcam::skill::cell_formation {

using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.charge_rate_c <= 0.0) {
        r.add("DFM-INPUT", "error",
              "cell_formation: charge_rate_c must be > 0 (got " +
              std::to_string(in.charge_rate_c) + ")");
    } else {
        if (in.charge_rate_c > 0.5) {
            r.add("DFM-FORM-RATE-FAST", "info",
                  "cell_formation: charge_rate_c " +
                  std::to_string(in.charge_rate_c) +
                  " C > 0.5 C — SEI quality at risk");
        }
        if (in.charge_rate_c < 0.02) {
            r.add("DFM-FORM-RATE-SLOW", "info",
                  "cell_formation: charge_rate_c " +
                  std::to_string(in.charge_rate_c) +
                  " C < 0.02 C — line throughput penalty");
        }
    }

    if (in.target_capacity_mah <= 0.0) {
        r.add("DFM-INPUT", "error",
              "cell_formation: target_capacity_mah must be > 0 (got " +
              std::to_string(in.target_capacity_mah) + ")");
    }

    if (in.cycle_count < 1) {
        r.add("DFM-FORM-CYCLES", "error",
              "cell_formation: cycle_count must be >= 1 (got " +
              std::to_string(in.cycle_count) + ")");
    } else if (in.cycle_count > 20) {
        r.add("DFM-FORM-CYCLES", "info",
              "cell_formation: cycle_count " +
              std::to_string(in.cycle_count) +
              " > 20 — diminishing returns; verify formation recipe");
    }

    if (in.upper_voltage_v < 2.0 || in.upper_voltage_v > 4.5) {
        r.add("DFM-FORM-VOLT", "info",
              "cell_formation: upper_voltage_v " +
              std::to_string(in.upper_voltage_v) +
              " outside typical [2.0, 4.5] V — verify chemistry");
    }

    if (in.aging_days < 0.0) {
        r.add("DFM-FORM-AGING", "error",
              "cell_formation: aging_days must be >= 0 (got " +
              std::to_string(in.aging_days) + ")");
    } else if (in.aging_days > 30.0) {
        r.add("DFM-FORM-AGING", "info",
              "cell_formation: aging_days " +
              std::to_string(in.aging_days) +
              " > 30 — long calendar aging hits WIP cost");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "cell_formation DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    json params = {
        { "charge_rate_c",       in.charge_rate_c },
        { "cycle_count",         in.cycle_count },
        { "target_capacity_mah", in.target_capacity_mah },
        { "upper_voltage_v",     in.upper_voltage_v },
        { "aging_days",          in.aging_days },
    };
    json pattern = {
        { "kind",                kSkillId },
        { "charge_rate_c",       in.charge_rate_c },
        { "cycle_count",         in.cycle_count },
        { "target_capacity_mah", in.target_capacity_mah },
        { "upper_voltage_v",     in.upper_voltage_v },
        { "aging_days",          in.aging_days },
        { "geometry_changed",    false },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "formation_cycler";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "channel_contactor";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    // Cycle time per pass ≈ (1 / rate) hours; aging is post-process calendar.
    const double seconds_per_cycle =
        (in.charge_rate_c > 0.0)
            ? (1.0 / in.charge_rate_c) * 3600.0
            : 0.0;
    tooling.est_cycle_time_s  =
        seconds_per_cycle * static_cast<double>(in.cycle_count) +
        in.aging_days * 86400.0;
    tooling.extra = {
        { "process",             "first_cycle_formation" },
        { "charge_rate_c",       in.charge_rate_c },
        { "cycle_count",         in.cycle_count },
        { "target_capacity_mah", in.target_capacity_mah },
        { "process_category",    "battery_manufacturing" },
        { "geometric_no_op",     true },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug(
        "skill::cell_formation applied: rate={}C cycles={} cap_target={}mAh",
        in.charge_rate_c, in.cycle_count, in.target_capacity_mah);

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

}  // namespace koocadcam::skill::cell_formation
