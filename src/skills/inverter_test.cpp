// @lat: [[engine/skills#inverter_test]]

#include "inverter_test.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <string>

namespace koocadcam::skill::inverter_test {

using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.inverter_id.empty()) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": inverter_id must be non-empty");
    }

    if (in.peak_power_kw <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": peak_power_kw must be > 0 (got " +
              std::to_string(in.peak_power_kw) + ")");
    }

    if (in.efficiency_pct <= 0.0 || in.efficiency_pct > 100.0) {
        r.add("DFM-INV-EFF", "error",
              std::string(kSkillId) + ": efficiency_pct " +
              std::to_string(in.efficiency_pct) +
              " not in (0, 100]");
    } else if (in.efficiency_pct < 90.0) {
        r.add("DFM-INV-EFF", "info",
              std::string(kSkillId) + ": efficiency_pct " +
              std::to_string(in.efficiency_pct) +
              " < 90% — FAIL threshold, unit should be reworked");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "inverter_test DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const std::string result = (in.efficiency_pct >= 90.0) ? "PASS" : "FAIL";

    json params = {
        { "inverter_id",    in.inverter_id },
        { "peak_power_kw",  in.peak_power_kw },
        { "efficiency_pct", in.efficiency_pct },
    };
    json pattern = {
        { "kind",             kSkillId },
        { "inverter_id",      in.inverter_id },
        { "peak_power_kw",    in.peak_power_kw },
        { "efficiency_pct",   in.efficiency_pct },
        { "test_result",      result },
        { "loss_kw",
          in.peak_power_kw * (1.0 - in.efficiency_pct / 100.0) },
        { "geometry_changed", false },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "inverter_dyno_test_bench";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "instrumentation";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    // ~ 180 s warm-up + 120 s sweep + 60 s teardown.
    tooling.est_cycle_time_s  = 360.0;
    tooling.extra = {
        { "process",        "ev_inverter_eol_test" },
        { "inverter_id",    in.inverter_id },
        { "peak_power_kw",  in.peak_power_kw },
        { "efficiency_pct", in.efficiency_pct },
        { "test_result",    result },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::inverter_test applied: id={} peak={}kW eff={}% result={}",
                  in.inverter_id, in.peak_power_kw, in.efficiency_pct, result);

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

}  // namespace koocadcam::skill::inverter_test
