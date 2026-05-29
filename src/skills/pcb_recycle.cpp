// @lat: [[engine/skills#pcb_recycle]]

#include "pcb_recycle.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <memory>
#include <string>

namespace koocadcam::skill::pcb_recycle {

using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;
    (void)wp;

    if (in.pcb_mass_kg <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": pcb_mass_kg must be > 0 (got " +
              std::to_string(in.pcb_mass_kg) + ")");
    }

    if (in.copper_yield_pct <= 0.0 || in.copper_yield_pct > 100.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": copper_yield_pct must be in (0, 100] (got " +
              std::to_string(in.copper_yield_pct) + ")");
    } else if (in.copper_yield_pct > 30.0) {
        r.add("DFM-PCB-CU", "info",
              std::string(kSkillId) + ": copper_yield_pct " +
              std::to_string(in.copper_yield_pct) +
              " exceeds 30% — atypical for EOL PCB feedstock, verify assay");
    }

    if (in.precious_metal_yield_g_t < 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": precious_metal_yield_g_t must be ≥ 0 (got " +
              std::to_string(in.precious_metal_yield_g_t) + ")");
    } else if (in.precious_metal_yield_g_t > 1000.0) {
        r.add("DFM-PCB-PM", "info",
              std::string(kSkillId) + ": precious_metal_yield_g_t " +
              std::to_string(in.precious_metal_yield_g_t) +
              " exceeds 1000 g/t — exceptionally rich feedstock, verify");
    }

    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "pcb_recycle DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const double recoveredCuKg = in.pcb_mass_kg * in.copper_yield_pct / 100.0;
    // g/tonne × kg ÷ 1000kg/tonne = g recovered
    const double recoveredPmG  = in.pcb_mass_kg * in.precious_metal_yield_g_t / 1000.0;

    json params = {
        { "pcb_mass_kg",              in.pcb_mass_kg },
        { "copper_yield_pct",         in.copper_yield_pct },
        { "precious_metal_yield_g_t", in.precious_metal_yield_g_t },
    };
    json pattern = {
        { "kind",                     kSkillId },
        { "pcb_mass_kg",              in.pcb_mass_kg },
        { "copper_yield_pct",         in.copper_yield_pct },
        { "precious_metal_yield_g_t", in.precious_metal_yield_g_t },
        { "recovered_copper_kg",      recoveredCuKg },
        { "recovered_pm_g",           recoveredPmG },
        { "geometry_changed",         false },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code", f.code },
                            { "severity", f.severity },
                            { "message", f.message }});

    ToolingMeta tooling;
    tooling.tool_type         = "pcb_recycle_line";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "n/a";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    // Shred + smelt + leach: ~180 s/kg + 60 s setup.
    tooling.est_cycle_time_s  = 60.0 + 180.0 * in.pcb_mass_kg;
    tooling.extra = {
        { "process",              "pcb_recycle" },
        { "pcb_mass_kg",          in.pcb_mass_kg },
        { "recovered_copper_kg",  recoveredCuKg },
        { "recovered_pm_g",       recoveredPmG },
        { "process_category",     "circular_economy" },
        { "geometric_no_op",      true },
        { "dfm_findings",         findings },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug(
        "skill::pcb_recycle applied: mass={}kg Cu={}% PM={}g/t recovered Cu={}kg PM={}g",
        in.pcb_mass_kg, in.copper_yield_pct, in.precious_metal_yield_g_t,
        recoveredCuKg, recoveredPmG);

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

}  // namespace koocadcam::skill::pcb_recycle
