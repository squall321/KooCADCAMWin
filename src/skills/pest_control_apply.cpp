// @lat: [[engine/skills#pest_control_apply]]

#include "pest_control_apply.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <memory>
#include <string>

namespace koocadcam::skill::pest_control_apply {

using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;
    (void)wp;

    if (in.chemical_name.empty()) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": chemical_name must be non-empty");
    }

    if (in.dose_ml_ha <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": dose_ml_ha must be > 0 (got " +
              std::to_string(in.dose_ml_ha) + ")");
    } else if (in.dose_ml_ha > 10000.0) {
        r.add("DFM-PEST-DOSE", "error",
              std::string(kSkillId) + ": dose_ml_ha " +
              std::to_string(in.dose_ml_ha) +
              " > 10000 — exceeds typical regulatory cap");
    }

    if (in.application_method != "spray"   &&
        in.application_method != "drench"  &&
        in.application_method != "granule") {
        r.add("DFM-PEST-METHOD", "info",
              std::string(kSkillId) + ": application_method '" +
              in.application_method +
              "' not in {spray, drench, granule} — treated as 'spray'");
    }

    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "pest_control_apply DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const std::string method =
        (in.application_method == "spray"   ||
         in.application_method == "drench"  ||
         in.application_method == "granule")
            ? in.application_method : "spray";

    json params = {
        { "chemical_name",      in.chemical_name },
        { "dose_ml_ha",         in.dose_ml_ha },
        { "application_method", method },
    };
    json pattern = {
        { "kind",               kSkillId },
        { "chemical_name",      in.chemical_name },
        { "dose_ml_ha",         in.dose_ml_ha },
        { "application_method", method },
        { "geometry_changed",   false },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code", f.code },
                            { "severity", f.severity },
                            { "message", f.message }});

    ToolingMeta tooling;
    if (method == "spray") {
        tooling.tool_type = "boom_sprayer";
    } else if (method == "drench") {
        tooling.tool_type = "soil_drench_tank";
    } else {
        tooling.tool_type = "granule_broadcaster";
    }
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "stainless_steel";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    tooling.est_cycle_time_s  = 1800.0;  // ~30 min per typical pass
    tooling.extra = {
        { "process",            "pest_control_apply" },
        { "chemical_name",      in.chemical_name },
        { "dose_ml_ha",         in.dose_ml_ha },
        { "application_method", method },
        { "process_category",   "agriculture" },
        { "geometric_no_op",    true },
        { "dfm_findings",       findings },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug(
        "skill::pest_control_apply applied: chem={} dose={}ml/ha method={}",
        in.chemical_name, in.dose_ml_ha, method);

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

}  // namespace koocadcam::skill::pest_control_apply
