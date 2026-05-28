// @lat: [[engine/skills#metal_injection_mold]]

#include "metal_injection_mold.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <string>
#include <unordered_set>

namespace koocadcam::skill::metal_injection_mold {

using nlohmann::json;

namespace {

// Materials in common MIM feedstock catalogs (BASF Catamold + Höganäs +
// Sandvik datasheets).  Unknown materials produce an info finding only.
const std::unordered_set<std::string>& knownMimMaterials()
{
    static const std::unordered_set<std::string> s = {
        "stainless_316", "stainless_316l", "stainless_17_4ph", "stainless_420",
        "low_alloy_steel_4605", "low_alloy_steel_4140",
        "tool_steel_m2", "tool_steel_d2",
        "titanium_grade5", "titanium_grade2",
        "tungsten_heavy_alloy", "kovar", "invar",
        "cobalt_chrome_f75", "iron_2ni", "iron_8ni",
    };
    return s;
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.wall_thickness_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": wall_thickness_mm must be > 0 (got " +
              std::to_string(in.wall_thickness_mm) + ")");
    } else if (in.wall_thickness_mm < 0.5) {
        r.add("DFM-MIM-WALL", "error",
              std::string(kSkillId) + ": wall_thickness " +
              std::to_string(in.wall_thickness_mm) +
              " mm < 0.5 mm MIM minimum (sinter collapse)");
    } else if (in.wall_thickness_mm > 10.0) {
        r.add("DFM-MIM-WALL-HEAVY", "info",
              std::string(kSkillId) + ": wall_thickness " +
              std::to_string(in.wall_thickness_mm) +
              " mm > 10 mm — debinding cracking risk; MIM optimal for thin sections");
    }

    if (in.binder_pct < 30.0 || in.binder_pct > 50.0) {
        r.add("DFM-MIM-BINDER", "error",
              std::string(kSkillId) + ": binder_pct " +
              std::to_string(in.binder_pct) +
              " outside [30, 50] typical MIM feedstock range");
    }

    if (in.sinter_shrinkage_pct <= 0.0 || in.sinter_shrinkage_pct >= 100.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": sinter_shrinkage_pct must be in (0, 100), got " +
              std::to_string(in.sinter_shrinkage_pct));
    } else if (in.sinter_shrinkage_pct < 10.0 || in.sinter_shrinkage_pct > 25.0) {
        r.add("DFM-MIM-SHRINK", "info",
              std::string(kSkillId) + ": sinter_shrinkage_pct " +
              std::to_string(in.sinter_shrinkage_pct) +
              " outside typical MIM range [10, 25]");
    }

    if (in.material.empty()) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": material must be specified");
    } else if (knownMimMaterials().count(in.material) == 0) {
        r.add("DFM-MIM-MAT", "info",
              std::string(kSkillId) + ": material '" + in.material +
              "' not in MIM feedstock catalog — verify shrinkage factor");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = std::string(kSkillId) + " DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    // Cavity pre-scale needed by tool-builder downstream: dim_cavity
    // = dim_part / (1 - S/100).  Stored in pattern (not applied to shape).
    const double scale = 1.0 / (1.0 - in.sinter_shrinkage_pct / 100.0);

    json params = {
        { "material",              in.material },
        { "binder_pct",            in.binder_pct },
        { "sinter_shrinkage_pct",  in.sinter_shrinkage_pct },
        { "wall_thickness_mm",     in.wall_thickness_mm },
    };

    json pattern = {
        { "kind",                       kSkillId },
        { "is_process_only",            true },
        { "geometric_change",           false },
        { "process_family",             "powder_metallurgy" },
        { "process_subtype",            "mim" },
        { "material",                   in.material },
        { "binder_pct",                 in.binder_pct },
        { "sinter_shrinkage_pct",       in.sinter_shrinkage_pct },
        { "predicted_cavity_scale",     scale },
        { "wall_thickness_mm",          in.wall_thickness_mm },
        { "additive_workflow",          true },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code",     f.code },
                            { "severity", f.severity },
                            { "message",  f.message }});

    ToolingMeta tooling;
    tooling.tool_type         = "mim_press_furnace";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "hardened_tool_steel_cavity";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    // Injection (~ minutes) + solvent/thermal debind (hours) + sinter (hours).
    tooling.est_cycle_time_s  = 6.0 * 3600.0;
    tooling.extra = {
        { "process",                "metal_injection_mold" },
        { "process_family",         "powder_metallurgy" },
        { "material",               in.material },
        { "binder_pct",             in.binder_pct },
        { "sinter_shrinkage_pct",   in.sinter_shrinkage_pct },
        { "predicted_cavity_scale", scale },
        { "dfm_findings",           findings },
        { "machining_constraint",
          "no material removal — MIM produces final shape directly" },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug("skill::metal_injection_mold applied: material={} binder={} shrink={}",
                  in.material, in.binder_pct, in.sinter_shrinkage_pct);

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

}  // namespace koocadcam::skill::metal_injection_mold
