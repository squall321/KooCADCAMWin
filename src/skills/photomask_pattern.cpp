// @lat: [[engine/skills#photomask_pattern]]

#include "photomask_pattern.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <string>

namespace koocadcam::skill::photomask_pattern {

using nlohmann::json;

namespace {

bool isKnownAlignment(const std::string& s)
{
    return s == "coarse" || s == "fine" || s == "critical";
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.mask_id.empty()) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": mask_id must be non-empty");
    }

    if (in.exposure_dose_mj_cm2 <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": exposure_dose_mj_cm2 must be > 0 (got " +
              std::to_string(in.exposure_dose_mj_cm2) + ")");
    } else if (in.exposure_dose_mj_cm2 < 1.0) {
        r.add("DFM-LITHO-DOSE", "info",
              std::string(kSkillId) + ": exposure_dose_mj_cm2 " +
              std::to_string(in.exposure_dose_mj_cm2) +
              " < 1 mJ/cm² — likely underexposed (typical 5–200)");
    } else if (in.exposure_dose_mj_cm2 > 1000.0) {
        r.add("DFM-LITHO-DOSE", "info",
              std::string(kSkillId) + ": exposure_dose_mj_cm2 " +
              std::to_string(in.exposure_dose_mj_cm2) +
              " > 1000 mJ/cm² — likely overexposed");
    }

    if (!isKnownAlignment(in.alignment_class)) {
        r.add("DFM-LITHO-ALIGN", "info",
              std::string(kSkillId) + ": alignment_class '" + in.alignment_class +
              "' unrecognised — expected coarse | fine | critical");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "photomask_pattern DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    json params = {
        { "mask_id",              in.mask_id },
        { "exposure_dose_mj_cm2", in.exposure_dose_mj_cm2 },
        { "alignment_class",      in.alignment_class },
    };
    json pattern = {
        { "kind",                 kSkillId },
        { "mask_id",              in.mask_id },
        { "exposure_dose_mj_cm2", in.exposure_dose_mj_cm2 },
        { "alignment_class",      in.alignment_class },
        { "is_metadata_only",     true },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code", f.code },
                            { "severity", f.severity },
                            { "message", f.message }});

    ToolingMeta tooling;
    tooling.tool_type         = "stepper_scanner";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "fused_silica_reticle";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    // Single field exposure ≈ 1 s; per-wafer cycle ~ 30 s.
    tooling.est_cycle_time_s  = 30.0;
    tooling.extra = {
        { "process",              "photolithography_exposure" },
        { "mask_id",              in.mask_id },
        { "alignment_class",      in.alignment_class },
        { "dfm_findings",         findings },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug("skill::photomask_pattern applied: mask={} dose={} align={}",
                  in.mask_id, in.exposure_dose_mj_cm2, in.alignment_class);

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

}  // namespace koocadcam::skill::photomask_pattern
