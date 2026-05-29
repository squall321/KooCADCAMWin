// @lat: [[engine/skills#lyophilize_pharma]]

#include "lyophilize_pharma.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <string>

namespace koocadcam::skill::lyophilize_pharma {

using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.vacuum_mbar <= 0.0 || in.vacuum_mbar > 100.0) {
        r.add("DFM-INPUT", "error",
              "lyophilize_pharma vacuum_mbar " +
              std::to_string(in.vacuum_mbar) +
              " outside (0, 100] mbar range");
    }

    if (in.secondary_dry_temp_c <= in.primary_dry_temp_c) {
        r.add("DFM-INPUT", "error",
              "lyophilize_pharma secondary_dry_temp_c (" +
              std::to_string(in.secondary_dry_temp_c) +
              ") must be > primary_dry_temp_c (" +
              std::to_string(in.primary_dry_temp_c) + ")");
    }

    if (in.primary_dry_temp_c < -60.0 || in.primary_dry_temp_c > 0.0) {
        r.add("DFM-LYP-PRIM-TEMP", "info",
              "lyophilize_pharma primary_dry_temp_c " +
              std::to_string(in.primary_dry_temp_c) +
              " outside typical [-60, 0] °C — review collapse temperature");
    }

    if (in.secondary_dry_temp_c > 60.0) {
        r.add("DFM-LYP-SEC-TEMP", "info",
              "lyophilize_pharma secondary_dry_temp_c " +
              std::to_string(in.secondary_dry_temp_c) +
              " > 60 °C — protein/biologic denaturation risk");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "lyophilize_pharma DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    json params = {
        { "primary_dry_temp_c",   in.primary_dry_temp_c },
        { "secondary_dry_temp_c", in.secondary_dry_temp_c },
        { "vacuum_mbar",          in.vacuum_mbar },
    };

    json pattern = {
        { "kind",                 kSkillId },
        { "primary_dry_temp_c",   in.primary_dry_temp_c },
        { "secondary_dry_temp_c", in.secondary_dry_temp_c },
        { "vacuum_mbar",          in.vacuum_mbar },
        { "process_family",       "pharma_lyophilization" },
        { "geometric_change",     false },
        { "is_process_only",      true },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code",     f.code },
                            { "severity", f.severity },
                            { "message",  f.message }});

    ToolingMeta tooling;
    tooling.tool_type         = "pharma_freeze_dryer";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "stainless_316l_chamber";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    tooling.est_cycle_time_s  = 0.0;
    tooling.extra = {
        { "process",              "pharma_lyophilization" },
        { "primary_dry_temp_c",   in.primary_dry_temp_c },
        { "secondary_dry_temp_c", in.secondary_dry_temp_c },
        { "vacuum_mbar",          in.vacuum_mbar },
        { "process_category",     "pharma_parenteral" },
        { "dfm_findings",         findings },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug("skill::lyophilize_pharma applied: T1={}C T2={}C vac={}mbar",
                  in.primary_dry_temp_c, in.secondary_dry_temp_c,
                  in.vacuum_mbar);

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

}  // namespace koocadcam::skill::lyophilize_pharma
