// @lat: [[engine/skills#sterile_fill]]

#include "sterile_fill.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <string>

namespace koocadcam::skill::sterile_fill {

using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.fill_volume_ml <= 0.0) {
        r.add("DFM-INPUT", "error",
              "sterile_fill fill_volume_ml must be > 0 (got " +
              std::to_string(in.fill_volume_ml) + ")");
    } else if (in.fill_volume_ml > 500.0) {
        r.add("DFM-SF-VOLUME-LARGE", "info",
              "sterile_fill fill_volume_ml " +
              std::to_string(in.fill_volume_ml) +
              " > 500 mL — atypical for unit-dose container");
    }

    if (in.env_class != "ISO_5" && in.env_class != "ISO_7") {
        r.add("DFM-SF-ENV", "error",
              "sterile_fill env_class '" + in.env_class +
              "' invalid — aseptic fill requires 'ISO_5' or 'ISO_7'");
    } else if (in.env_class == "ISO_7") {
        r.add("DFM-SF-ENV-CLASS", "info",
              "sterile_fill env_class 'ISO_7' — Annex 1 requires ISO 5 at "
              "point of fill for non-terminally-sterilized product");
    }

    if (in.container_type != "vial"     &&
        in.container_type != "syringe"  &&
        in.container_type != "ampoule"  &&
        in.container_type != "cartridge") {
        r.add("DFM-SF-CONTAINER", "info",
              "sterile_fill container_type '" + in.container_type +
              "' unrecognised — expected 'vial' | 'syringe' | 'ampoule' | "
              "'cartridge'");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "sterile_fill DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    json params = {
        { "fill_volume_ml", in.fill_volume_ml },
        { "container_type", in.container_type },
        { "env_class",      in.env_class },
    };

    json pattern = {
        { "kind",             kSkillId },
        { "fill_volume_ml",   in.fill_volume_ml },
        { "container_type",   in.container_type },
        { "env_class",        in.env_class },
        { "process_family",   "pharma_aseptic_fill" },
        { "geometric_change", false },
        { "is_process_only",  true },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code",     f.code },
                            { "severity", f.severity },
                            { "message",  f.message }});

    ToolingMeta tooling;
    tooling.tool_type         = "aseptic_filler";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "stainless_316l_filling_needle";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    tooling.est_cycle_time_s  = 0.0;
    tooling.extra = {
        { "process",          "aseptic_filling" },
        { "fill_volume_ml",   in.fill_volume_ml },
        { "container_type",   in.container_type },
        { "env_class",        in.env_class },
        { "process_category", "pharma_parenteral" },
        { "dfm_findings",     findings },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug("skill::sterile_fill applied: vol={}mL container={} env={}",
                  in.fill_volume_ml, in.container_type, in.env_class);

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

}  // namespace koocadcam::skill::sterile_fill
