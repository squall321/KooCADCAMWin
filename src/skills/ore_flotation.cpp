// @lat: [[engine/skills#ore_flotation]]

#include "ore_flotation.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <string>

namespace koocadcam::skill::ore_flotation {

using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.cell_count <= 0) {
        r.add("DFM-INPUT", "error",
              "ore_flotation cell_count must be > 0 (got " +
              std::to_string(in.cell_count) + ")");
    }

    if (in.retention_time_min <= 0.0) {
        r.add("DFM-INPUT", "error",
              "ore_flotation retention_time_min must be > 0 (got " +
              std::to_string(in.retention_time_min) + ")");
    }

    if (in.recovery_pct <= 0.0 || in.recovery_pct > 100.0) {
        r.add("DFM-INPUT", "error",
              "ore_flotation recovery_pct must be in (0, 100] (got " +
              std::to_string(in.recovery_pct) + ")");
    } else if (in.recovery_pct < 50.0) {
        r.add("DFM-RECOVERY-LOW", "info",
              "ore_flotation recovery_pct " + std::to_string(in.recovery_pct) +
              " < 50% — sub-economic; consider reagent / grind review");
    }

    if (in.retention_time_min > 120.0) {
        r.add("DFM-RETENTION-LONG", "info",
              "ore_flotation retention_time_min " +
              std::to_string(in.retention_time_min) +
              " > 120 min — bank likely oversized; reassess kinetics");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "ore_flotation DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const double total_bank_time_min = in.cell_count * in.retention_time_min;

    json params = {
        { "cell_count",         in.cell_count },
        { "retention_time_min", in.retention_time_min },
        { "recovery_pct",       in.recovery_pct },
    };

    json pattern = {
        { "kind",                kSkillId },
        { "cell_count",          in.cell_count },
        { "retention_time_min",  in.retention_time_min },
        { "recovery_pct",        in.recovery_pct },
        { "total_bank_time_min", total_bank_time_min },
        { "process_family",      "froth_flotation" },
        { "geometry_changed",    false },
        { "is_process_only",     true },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code",     f.code },
                            { "severity", f.severity },
                            { "message",  f.message }});

    ToolingMeta tooling;
    tooling.tool_type         = "flotation_cell_bank";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "rubber_lined_steel";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    tooling.est_cycle_time_s  = total_bank_time_min * 60.0;
    tooling.extra = {
        { "process",             "froth_flotation" },
        { "cell_count",          in.cell_count },
        { "retention_time_min",  in.retention_time_min },
        { "recovery_pct",        in.recovery_pct },
        { "total_bank_time_min", total_bank_time_min },
        { "process_category",    "mineral_processing" },
        { "dfm_findings",        findings },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::ore_flotation applied: cells={} t_ret={}min recovery={}%",
                  in.cell_count, in.retention_time_min, in.recovery_pct);

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

}  // namespace koocadcam::skill::ore_flotation
