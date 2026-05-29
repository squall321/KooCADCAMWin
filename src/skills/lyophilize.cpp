// @lat: [[engine/skills#lyophilize]]

#include "lyophilize.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <string>

namespace koocadcam::skill::lyophilize {

using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.freeze_temp_c >= 0.0) {
        r.add("DFM-INPUT", "error",
              "lyophilize freeze_temp_c must be < 0 °C (got " +
              std::to_string(in.freeze_temp_c) + ")");
    } else if (in.freeze_temp_c < -200.0) {
        r.add("DFM-LYO-TEMP", "info",
              "lyophilize freeze_temp_c " + std::to_string(in.freeze_temp_c) +
              " °C below practical industrial freezer range (≥ -80 °C typical)");
    }

    if (in.primary_dry_pressure_mbar <= 0.0 ||
        in.primary_dry_pressure_mbar > 100.0) {
        r.add("DFM-LYO-PRESSURE", "error",
              "lyophilize primary_dry_pressure_mbar " +
              std::to_string(in.primary_dry_pressure_mbar) +
              " outside (0, 100] mbar range");
    }

    if (in.residual_moisture_pct < 0.0 || in.residual_moisture_pct > 20.0) {
        r.add("DFM-LYO-MOISTURE", "error",
              "lyophilize residual_moisture_pct " +
              std::to_string(in.residual_moisture_pct) +
              " outside [0, 20] %% range");
    } else if (in.residual_moisture_pct > 5.0) {
        r.add("DFM-LYO-MOISTURE-HIGH", "info",
              "lyophilize residual_moisture_pct " +
              std::to_string(in.residual_moisture_pct) +
              " > 5 %% — long-term protein stability risk");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "lyophilize DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    json params = {
        { "freeze_temp_c",             in.freeze_temp_c },
        { "primary_dry_pressure_mbar", in.primary_dry_pressure_mbar },
        { "residual_moisture_pct",     in.residual_moisture_pct },
    };

    json pattern = {
        { "kind",                       kSkillId },
        { "freeze_temp_c",              in.freeze_temp_c },
        { "primary_dry_pressure_mbar",  in.primary_dry_pressure_mbar },
        { "residual_moisture_pct",      in.residual_moisture_pct },
        { "process_family",             "lyophilization" },
        { "geometric_change",           false },
        { "is_process_only",            true },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code",     f.code },
                            { "severity", f.severity },
                            { "message",  f.message }});

    ToolingMeta tooling;
    tooling.tool_type         = "freeze_dryer";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "stainless_316l_chamber";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    tooling.est_cycle_time_s  = 0.0;
    tooling.extra = {
        { "process",                    "lyophilization" },
        { "freeze_temp_c",              in.freeze_temp_c },
        { "primary_dry_pressure_mbar",  in.primary_dry_pressure_mbar },
        { "residual_moisture_pct",      in.residual_moisture_pct },
        { "process_category",           "drying" },
        { "dfm_findings",               findings },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug("skill::lyophilize applied: freeze={}C P={}mbar moisture={}%%",
                  in.freeze_temp_c, in.primary_dry_pressure_mbar,
                  in.residual_moisture_pct);

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

}  // namespace koocadcam::skill::lyophilize
