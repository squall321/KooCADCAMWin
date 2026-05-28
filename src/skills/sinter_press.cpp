// @lat: [[engine/skills#sinter_press]]

#include "sinter_press.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <string>
#include <unordered_set>

namespace koocadcam::skill::sinter_press {

using nlohmann::json;

namespace {

const std::unordered_set<std::string>& knownSinterMaterials()
{
    static const std::unordered_set<std::string> s = {
        "iron_fc_0205", "iron_fn_0205", "iron_fl_4205",
        "iron_2ni", "iron_8ni",
        "stainless_316l", "stainless_410", "stainless_434",
        "bronze_cf_1000", "bronze_cf_5000",
        "copper_pure", "brass_cz_1000",
        "tungsten_carbide_co_6", "tungsten_carbide_co_12",
        "low_alloy_steel_4605",
    };
    return s;
}

double expectedDensityPct(double pressure_mpa, double sinter_t, double hold_min)
{
    if (pressure_mpa <= 0.0 || sinter_t <= 0.0 || hold_min <= 0.0) return 0.0;
    // Saturating heuristic: higher P + T + t → density closer to ~ 96 %.
    const double e = pressure_mpa * sinter_t * hold_min;
    const double k = 5.0e-9;
    const double frac = 1.0 - std::exp(-k * e);
    return 80.0 + 16.0 * frac;
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.compaction_pressure_mpa <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": compaction_pressure_mpa must be > 0 (got " +
              std::to_string(in.compaction_pressure_mpa) + ")");
    } else if (in.compaction_pressure_mpa < 200.0 ||
               in.compaction_pressure_mpa > 800.0) {
        r.add("DFM-SP-PRESS", "info",
              std::string(kSkillId) + ": compaction_pressure_mpa " +
              std::to_string(in.compaction_pressure_mpa) +
              " outside typical press-and-sinter range [200, 800] MPa");
    }

    if (in.sinter_temp_c <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": sinter_temp_c must be > 0 (got " +
              std::to_string(in.sinter_temp_c) + ")");
    } else if (in.sinter_temp_c < 700.0 || in.sinter_temp_c > 1400.0) {
        r.add("DFM-SP-SINTER-T", "info",
              std::string(kSkillId) + ": sinter_temp_c " +
              std::to_string(in.sinter_temp_c) +
              " outside typical sintering range [700, 1400] °C");
    }

    if (in.sinter_time_min <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": sinter_time_min must be > 0 (got " +
              std::to_string(in.sinter_time_min) + ")");
    } else if (in.sinter_time_min < 15.0 || in.sinter_time_min > 240.0) {
        r.add("DFM-SP-SINTER-t", "info",
              std::string(kSkillId) + ": sinter_time_min " +
              std::to_string(in.sinter_time_min) +
              " outside typical sintering range [15, 240] min");
    }

    if (in.material.empty()) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": material must be specified");
    } else if (knownSinterMaterials().count(in.material) == 0) {
        r.add("DFM-SP-MAT", "info",
              std::string(kSkillId) + ": material '" + in.material +
              "' not in press-and-sinter catalog");
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

    const double densityPct = expectedDensityPct(
        in.compaction_pressure_mpa, in.sinter_temp_c, in.sinter_time_min);

    json params = {
        { "material",                 in.material },
        { "compaction_pressure_mpa",  in.compaction_pressure_mpa },
        { "sinter_temp_c",            in.sinter_temp_c },
        { "sinter_time_min",          in.sinter_time_min },
    };

    json pattern = {
        { "kind",                     kSkillId },
        { "is_process_only",          true },
        { "geometric_change",         false },
        { "process_family",           "powder_metallurgy" },
        { "process_subtype",          "press_and_sinter" },
        { "material",                 in.material },
        { "compaction_pressure_mpa",  in.compaction_pressure_mpa },
        { "sinter_temp_c",            in.sinter_temp_c },
        { "sinter_time_min",          in.sinter_time_min },
        { "expected_density_pct",     densityPct },
        { "additive_workflow",        true },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code",     f.code },
                            { "severity", f.severity },
                            { "message",  f.message }});

    ToolingMeta tooling;
    tooling.tool_type         = "uniaxial_press_furnace";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "tool_steel_die_carbide_insert";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    // Press (~ seconds) + dewax + sinter belt time.
    tooling.est_cycle_time_s  = (10.0 + in.sinter_time_min + 30.0) * 60.0;
    tooling.extra = {
        { "process",                  "sinter_press" },
        { "process_family",           "powder_metallurgy" },
        { "material",                 in.material },
        { "compaction_pressure_mpa",  in.compaction_pressure_mpa },
        { "sinter_temp_c",            in.sinter_temp_c },
        { "sinter_time_min",          in.sinter_time_min },
        { "expected_density_pct",     densityPct },
        { "dfm_findings",             findings },
        { "machining_constraint",
          "no material removal — die-press + sinter produces near-net shape" },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug("skill::sinter_press applied: material={} P={} T={} t={}",
                  in.material, in.compaction_pressure_mpa,
                  in.sinter_temp_c, in.sinter_time_min);

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

}  // namespace koocadcam::skill::sinter_press
