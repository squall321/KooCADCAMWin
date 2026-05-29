// @lat: [[engine/skills#package_medical]]

#include "package_medical.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <string>
#include <unordered_set>

namespace koocadcam::skill::package_medical {

using nlohmann::json;

namespace {

const std::unordered_set<std::string>& knownBarriers()
{
    static const std::unordered_set<std::string> s = {
        "tyvek_pouch", "foil_blister", "rigid_tray", "double_pouch",
    };
    return s;
}

const std::unordered_set<std::string>& knownLabelStandards()
{
    static const std::unordered_set<std::string> s = {
        "ISO 11607", "ISO 11607-1", "ISO 11607-2",
        "FDA-21-CFR-820", "EU-MDR-2017-745",
    };
    return s;
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.barrier_class.empty() || knownBarriers().count(in.barrier_class) == 0) {
        r.add("DFM-INPUT", "error",
              "package_medical unknown barrier_class '" + in.barrier_class +
              "' (expected tyvek_pouch | foil_blister | rigid_tray | double_pouch)");
    }

    if (in.shelf_life_months <= 0.0 || in.shelf_life_months > 120.0) {
        r.add("DFM-PACKAGE-SHELF", "error",
              "package_medical shelf_life_months " +
              std::to_string(in.shelf_life_months) +
              " outside (0, 120] month range");
    }

    if (in.label_compliance.empty()) {
        r.add("DFM-INPUT", "error",
              "package_medical label_compliance must be specified");
    } else if (knownLabelStandards().count(in.label_compliance) == 0) {
        r.add("DFM-PACKAGE-LABEL", "info",
              "package_medical label_compliance '" + in.label_compliance +
              "' not in standard regulatory catalog");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "package_medical DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    json params = {
        { "barrier_class",     in.barrier_class },
        { "label_compliance",  in.label_compliance },
        { "shelf_life_months", in.shelf_life_months },
    };

    json pattern = {
        { "kind",                 kSkillId },
        { "barrier_class",        in.barrier_class },
        { "label_compliance",     in.label_compliance },
        { "shelf_life_months",    in.shelf_life_months },
        { "process_family",       "medical_packaging" },
        { "geometric_change",     false },
        { "is_process_only",      true },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code",     f.code },
                            { "severity", f.severity },
                            { "message",  f.message }});

    ToolingMeta tooling;
    if      (in.barrier_class == "tyvek_pouch")  tooling.tool_type = "heat_sealer";
    else if (in.barrier_class == "foil_blister") tooling.tool_type = "blister_sealer";
    else if (in.barrier_class == "rigid_tray")   tooling.tool_type = "tray_lidding_sealer";
    else if (in.barrier_class == "double_pouch") tooling.tool_type = "double_pouch_sealer";
    else                                          tooling.tool_type = "medical_packager";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "ptfe_coated_heater_bar";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    tooling.est_cycle_time_s  = 0.0;
    tooling.extra = {
        { "process",            "medical_barrier_packaging" },
        { "barrier_class",      in.barrier_class },
        { "label_compliance",   in.label_compliance },
        { "shelf_life_months",  in.shelf_life_months },
        { "process_category",   "packaging" },
        { "dfm_findings",       findings },
        { "regulatory_target",  "ISO 11607" },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug("skill::package_medical applied: barrier={} label={} shelf={}mo",
                  in.barrier_class, in.label_compliance, in.shelf_life_months);

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

}  // namespace koocadcam::skill::package_medical
