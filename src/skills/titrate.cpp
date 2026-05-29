// @lat: [[engine/skills#titrate]]

#include "titrate.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <string>

namespace koocadcam::skill::titrate {

using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.analyte.empty()) {
        r.add("DFM-INPUT", "error",
              "titrate analyte must not be empty");
    }
    if (in.titrant.empty()) {
        r.add("DFM-INPUT", "error",
              "titrate titrant must not be empty");
    }

    if (in.target_endpoint_ph < 0.0 || in.target_endpoint_ph > 14.0) {
        r.add("DFM-TITR-PH", "error",
              "titrate target_endpoint_ph " +
              std::to_string(in.target_endpoint_ph) +
              " outside [0, 14]");
    }

    if (in.volume_ml <= 0.0) {
        r.add("DFM-TITR-VOL", "error",
              "titrate volume_ml must be > 0 (got " +
              std::to_string(in.volume_ml) + ")");
    } else if (in.volume_ml > 1000.0) {
        r.add("DFM-TITR-VOL-HIGH", "info",
              "titrate volume_ml " + std::to_string(in.volume_ml) +
              " mL exceeds typical bench burette range (≤1000 mL)");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "titrate DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    json params = {
        { "analyte",            in.analyte },
        { "titrant",            in.titrant },
        { "target_endpoint_ph", in.target_endpoint_ph },
        { "volume_ml",          in.volume_ml },
    };

    json pattern = {
        { "kind",               kSkillId },
        { "analyte",            in.analyte },
        { "titrant",            in.titrant },
        { "target_endpoint_ph", in.target_endpoint_ph },
        { "volume_ml",          in.volume_ml },
        { "process_family",     "volumetric_analysis" },
        { "geometry_changed",   false },
        { "is_process_only",    true },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code",     f.code },
                            { "severity", f.severity },
                            { "message",  f.message }});

    ToolingMeta tooling;
    // Endpoint-detection apparatus: pH meter for buffered endpoints near 7,
    // indicator-only otherwise.  Conductivity preferred for very low/high pH.
    if (in.target_endpoint_ph < 3.0 || in.target_endpoint_ph > 11.0)
        tooling.tool_type = "conductivity_probe";
    else
        tooling.tool_type = "ph_meter";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "burette_borosilicate";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    // Bench manual titration ≈ 5 minutes baseline + 1 s per mL delivered.
    tooling.est_cycle_time_s  = 300.0 + in.volume_ml;
    tooling.extra = {
        { "process",            "volumetric_titration" },
        { "analyte",            in.analyte },
        { "titrant",            in.titrant },
        { "target_endpoint_ph", in.target_endpoint_ph },
        { "volume_ml",          in.volume_ml },
        { "process_category",   "laboratory" },
        { "dfm_findings",       findings },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug("skill::titrate applied: analyte={} titrant={} ph={} V={}mL",
                  in.analyte, in.titrant, in.target_endpoint_ph, in.volume_ml);

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

}  // namespace koocadcam::skill::titrate
