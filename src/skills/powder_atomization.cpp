// @lat: [[engine/skills#powder_atomization]]

#include "powder_atomization.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <string>
#include <unordered_set>

namespace koocadcam::skill::powder_atomization {

using nlohmann::json;

namespace {

const std::unordered_set<std::string>& knownAtomizationMaterials()
{
    static const std::unordered_set<std::string> s = {
        "stainless_316", "stainless_316l", "stainless_17_4ph", "stainless_420",
        "titanium_grade5", "titanium_grade23", "titanium_grade2",
        "inconel_718", "inconel_625",
        "aluminum_alsi10mg", "aluminum_6061",
        "cobalt_chrome_f75",
        "tool_steel_h13", "tool_steel_m2",
        "copper_pure", "iron_pure", "low_alloy_steel_4140",
        "kovar", "invar", "tungsten_pure", "nickel_pure",
    };
    return s;
}

const std::unordered_set<std::string>& knownAtomizationGases()
{
    static const std::unordered_set<std::string> s = {
        "argon", "nitrogen", "water", "plasma", "air",
    };
    return s;
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.median_particle_size_um <= 0.0 || in.median_particle_size_um > 500.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": median_particle_size_um must be in (0, 500], got " +
              std::to_string(in.median_particle_size_um));
    } else if (in.median_particle_size_um < 5.0 ||
               in.median_particle_size_um > 200.0) {
        r.add("DFM-PA-D50", "info",
              std::string(kSkillId) + ": median_particle_size_um " +
              std::to_string(in.median_particle_size_um) +
              " outside typical PM range [5, 200] µm");
    }

    if (in.gas.empty()) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": gas must be specified");
    } else if (knownAtomizationGases().count(in.gas) == 0) {
        r.add("DFM-PA-GAS", "info",
              std::string(kSkillId) + ": gas '" + in.gas +
              "' not in standard set {argon, nitrogen, water, plasma, air}");
    }

    if (in.yield_pct <= 0.0 || in.yield_pct > 100.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": yield_pct must be in (0, 100], got " +
              std::to_string(in.yield_pct));
    } else if (in.yield_pct < 50.0) {
        r.add("DFM-PA-YIELD", "info",
              std::string(kSkillId) + ": yield_pct " +
              std::to_string(in.yield_pct) +
              " < 50 — process likely upset (atomizer geometry / metal-jet stability)");
    }

    if (in.material.empty()) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": material must be specified");
    } else if (knownAtomizationMaterials().count(in.material) == 0) {
        r.add("DFM-PA-MAT", "info",
              std::string(kSkillId) + ": material '" + in.material +
              "' not in atomization catalog — verify melt-stream stability");
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

    json params = {
        { "material",                 in.material },
        { "median_particle_size_um",  in.median_particle_size_um },
        { "gas",                      in.gas },
        { "yield_pct",                in.yield_pct },
    };

    json pattern = {
        { "kind",                     kSkillId },
        { "is_process_only",          true },
        { "geometric_change",         false },
        { "process_family",           "powder_metallurgy" },
        { "process_subtype",          "atomization" },
        { "material",                 in.material },
        { "median_particle_size_um",  in.median_particle_size_um },
        { "gas",                      in.gas },
        { "yield_pct",                in.yield_pct },
        { "produces_powder_lot",      true },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code",     f.code },
                            { "severity", f.severity },
                            { "message",  f.message }});

    ToolingMeta tooling;
    tooling.tool_type         = "atomizer";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = (in.gas == "water") ? "water_jet_nozzle"
                                                    : "gas_atomization_nozzle";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    // Atomization run-time is melt-batch dominated; rough hour-scale value.
    tooling.est_cycle_time_s  = 4.0 * 3600.0;
    tooling.extra = {
        { "process",                  "powder_atomization" },
        { "process_family",           "powder_metallurgy" },
        { "material",                 in.material },
        { "median_particle_size_um",  in.median_particle_size_um },
        { "gas",                      in.gas },
        { "yield_pct",                in.yield_pct },
        { "dfm_findings",             findings },
        { "produces_powder_lot",      true },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug("skill::powder_atomization applied: material={} d50={} gas={} yield={}",
                  in.material, in.median_particle_size_um, in.gas, in.yield_pct);

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

}  // namespace koocadcam::skill::powder_atomization
