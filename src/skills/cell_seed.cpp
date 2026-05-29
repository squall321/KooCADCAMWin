// @lat: [[engine/skills#cell_seed]]

#include "cell_seed.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <string>
#include <unordered_set>

namespace koocadcam::skill::cell_seed {

using nlohmann::json;

namespace {

const std::unordered_set<std::string>& knownCellTypes()
{
    static const std::unordered_set<std::string> s = {
        "fibroblast", "hepatocyte", "msc", "hMSC",
        "chondrocyte", "osteoblast", "endothelial",
        "ipsc", "keratinocyte", "ipsc_derived_neuron",
    };
    return s;
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.cell_type.empty()) {
        r.add("DFM-INPUT", "error",
              "cell_seed cell_type must be specified");
    } else if (knownCellTypes().count(in.cell_type) == 0) {
        r.add("DFM-SEED-TYPE", "info",
              "cell_seed cell_type '" + in.cell_type +
              "' not in standard catalog — verify culture protocol");
    }

    if (in.density_cells_cm2 < 1.0e2 || in.density_cells_cm2 > 1.0e7) {
        r.add("DFM-SEED-DENSITY", "error",
              "cell_seed density_cells_cm2 " +
              std::to_string(in.density_cells_cm2) +
              " outside [1e2, 1e7] viable range");
    }

    if (in.viability_pct < 0.0 || in.viability_pct > 100.0) {
        r.add("DFM-SEED-VIABILITY", "error",
              "cell_seed viability_pct " +
              std::to_string(in.viability_pct) +
              " outside [0, 100] %% range");
    } else if (in.viability_pct < 70.0) {
        r.add("DFM-SEED-VIABILITY-LOW", "info",
              "cell_seed viability_pct " +
              std::to_string(in.viability_pct) +
              " < 70 %% — confluent culture rescue may fail");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "cell_seed DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    json params = {
        { "cell_type",         in.cell_type },
        { "density_cells_cm2", in.density_cells_cm2 },
        { "viability_pct",     in.viability_pct },
    };

    json pattern = {
        { "kind",                 kSkillId },
        { "cell_type",            in.cell_type },
        { "density_cells_cm2",    in.density_cells_cm2 },
        { "viability_pct",        in.viability_pct },
        { "process_family",       "bioprocessing" },
        { "geometric_change",     false },
        { "is_process_only",      true },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code",     f.code },
                            { "severity", f.severity },
                            { "message",  f.message }});

    ToolingMeta tooling;
    tooling.tool_type         = "automated_pipette";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "sterile_polypropylene";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    tooling.est_cycle_time_s  = 0.0;
    tooling.extra = {
        { "process",            "cell_seeding" },
        { "cell_type",          in.cell_type },
        { "density_cells_cm2",  in.density_cells_cm2 },
        { "viability_pct",      in.viability_pct },
        { "process_category",   "bioprocessing" },
        { "dfm_findings",       findings },
        { "biosafety_required", true },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug("skill::cell_seed applied: cell_type={} density={} viability={}%%",
                  in.cell_type, in.density_cells_cm2, in.viability_pct);

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

}  // namespace koocadcam::skill::cell_seed
