// @lat: [[engine/skills#synthesize_chemical]]

#include "synthesize_chemical.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <string>

namespace koocadcam::skill::synthesize_chemical {

using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.reaction_id.empty()) {
        r.add("DFM-INPUT", "error",
              "synthesize_chemical reaction_id must not be empty");
    }

    if (in.reactants_count < 1) {
        r.add("DFM-INPUT", "error",
              "synthesize_chemical reactants_count must be >= 1 (got " +
              std::to_string(in.reactants_count) + ")");
    }

    if (in.time_h <= 0.0) {
        r.add("DFM-RXN-TIME", "error",
              "synthesize_chemical time_h must be > 0 (got " +
              std::to_string(in.time_h) + ")");
    }

    if (in.solvent.empty()) {
        r.add("DFM-RXN-SOLVENT", "warning",
              "synthesize_chemical solvent empty — defaulting to 'neat'");
    }

    if (in.temp_c < -80.0 || in.temp_c > 400.0) {
        r.add("DFM-RXN-TEMP", "info",
              "synthesize_chemical temp_c " + std::to_string(in.temp_c) +
              " °C outside typical lab envelope [-80, 400]");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "synthesize_chemical DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const std::string solvent = in.solvent.empty() ? "neat" : in.solvent;

    json params = {
        { "reaction_id",     in.reaction_id },
        { "reactants_count", in.reactants_count },
        { "solvent",         solvent },
        { "temp_c",          in.temp_c },
        { "time_h",          in.time_h },
    };

    json pattern = {
        { "kind",             kSkillId },
        { "reaction_id",      in.reaction_id },
        { "reactants_count",  in.reactants_count },
        { "solvent",          solvent },
        { "temp_c",           in.temp_c },
        { "time_h",           in.time_h },
        { "process_family",   "wet_chem_synthesis" },
        { "geometry_changed", false },
        { "is_process_only",  true },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code",     f.code },
                            { "severity", f.severity },
                            { "message",  f.message }});

    ToolingMeta tooling;
    tooling.tool_type         = (in.temp_c >= 80.0)
                                   ? "reflux_apparatus"
                                   : "magnetic_stirrer";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "borosilicate_glass";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    tooling.est_cycle_time_s  = in.time_h * 3600.0;
    tooling.extra = {
        { "process",          "chemical_synthesis" },
        { "reaction_id",      in.reaction_id },
        { "reactants_count",  in.reactants_count },
        { "solvent",          solvent },
        { "temp_c",           in.temp_c },
        { "time_h",           in.time_h },
        { "process_category", "laboratory" },
        { "dfm_findings",     findings },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug("skill::synthesize_chemical applied: rxn={} solvent={} T={}°C t={}h",
                  in.reaction_id, solvent, in.temp_c, in.time_h);

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

}  // namespace koocadcam::skill::synthesize_chemical
