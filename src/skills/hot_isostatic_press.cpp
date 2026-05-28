// @lat: [[engine/skills#hot_isostatic_press]]

#include "hot_isostatic_press.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <cmath>
#include <string>
#include <unordered_set>

namespace koocadcam::skill::hot_isostatic_press {

using nlohmann::json;

namespace {

const std::unordered_set<std::string>& knownHipMaterials()
{
    static const std::unordered_set<std::string> s = {
        "stainless_316", "stainless_316l", "stainless_17_4ph",
        "titanium_grade5", "titanium_grade23",
        "inconel_718", "inconel_625", "hastelloy_x",
        "cobalt_chrome_f75", "tool_steel_m2", "tool_steel_h13",
        "aluminum_a356", "aluminum_a357",
        "tungsten_heavy_alloy",
    };
    return s;
}

// Empirical post-HIP density (% theoretical).  Very crude — real value
// depends on starting porosity.  Used only as a downstream hint.
double expectedDensityPct(double pressure_mpa, double temp_c, double hold_min)
{
    if (pressure_mpa <= 0.0 || hold_min <= 0.0 || temp_c <= 0.0) return 0.0;
    // Closer to 100 % as pressure × temperature × time grows; saturating
    // approximation 99.0 + 1.0 · (1 − exp(−k · P · T · t)).
    const double e = pressure_mpa * temp_c * hold_min;
    const double k = 1.0e-8;
    const double frac = 1.0 - std::exp(-k * e);
    return 99.0 + 1.0 * frac;
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.temp_c <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": temp_c must be > 0 (got " +
              std::to_string(in.temp_c) + ")");
    } else if (in.temp_c < 800.0 || in.temp_c > 2000.0) {
        r.add("DFM-HIP-TEMP", "info",
              std::string(kSkillId) + ": temp_c " +
              std::to_string(in.temp_c) +
              " outside typical HIP range [800, 2000] °C");
    }

    if (in.pressure_mpa <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": pressure_mpa must be > 0 (got " +
              std::to_string(in.pressure_mpa) + ")");
    } else if (in.pressure_mpa < 50.0 || in.pressure_mpa > 300.0) {
        r.add("DFM-HIP-PRESS", "info",
              std::string(kSkillId) + ": pressure_mpa " +
              std::to_string(in.pressure_mpa) +
              " outside typical HIP range [50, 300] MPa");
    }

    if (in.hold_min <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": hold_min must be > 0 (got " +
              std::to_string(in.hold_min) + ")");
    } else if (in.hold_min < 30.0) {
        r.add("DFM-HIP-HOLD", "info",
              std::string(kSkillId) + ": hold_min " +
              std::to_string(in.hold_min) +
              " < 30 — pore closure may be incomplete");
    } else if (in.hold_min > 480.0) {
        r.add("DFM-HIP-HOLD", "info",
              std::string(kSkillId) + ": hold_min " +
              std::to_string(in.hold_min) +
              " > 480 — excessive cycle cost / grain growth");
    }

    if (in.material.empty()) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": material must be specified");
    } else if (knownHipMaterials().count(in.material) == 0) {
        r.add("DFM-HIP-MAT", "info",
              std::string(kSkillId) + ": material '" + in.material +
              "' not in HIP catalog — verify cycle parameters");
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
        in.pressure_mpa, in.temp_c, in.hold_min);

    json params = {
        { "material",      in.material },
        { "temp_c",        in.temp_c },
        { "pressure_mpa",  in.pressure_mpa },
        { "hold_min",      in.hold_min },
    };

    json pattern = {
        { "kind",                   kSkillId },
        { "is_process_only",        true },
        { "geometric_change",       false },
        { "process_family",         "powder_metallurgy" },
        { "process_subtype",        "hip" },
        { "material",               in.material },
        { "temp_c",                 in.temp_c },
        { "pressure_mpa",           in.pressure_mpa },
        { "hold_min",               in.hold_min },
        { "expected_density_pct",   densityPct },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code",     f.code },
                            { "severity", f.severity },
                            { "message",  f.message }});

    ToolingMeta tooling;
    tooling.tool_type         = "hip_vessel";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "molybdenum_lined_pressure_vessel";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    // Ramp + hold + cool, all under pressure.
    tooling.est_cycle_time_s  = (60.0 + in.hold_min + 120.0) * 60.0;
    tooling.extra = {
        { "process",              "hot_isostatic_press" },
        { "process_family",       "powder_metallurgy" },
        { "material",             in.material },
        { "temp_c",               in.temp_c },
        { "pressure_mpa",         in.pressure_mpa },
        { "hold_min",             in.hold_min },
        { "expected_density_pct", densityPct },
        { "dfm_findings",         findings },
        { "machining_constraint",
          "no material removal — pore closure only, outer envelope unchanged" },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug("skill::hot_isostatic_press applied: T={} P={} t={}",
                  in.temp_c, in.pressure_mpa, in.hold_min);

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

}  // namespace koocadcam::skill::hot_isostatic_press
