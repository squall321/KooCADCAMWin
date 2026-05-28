// @lat: [[engine/skills#composite_cure]]

#include "composite_cure.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <string>

namespace koocadcam::skill::composite_cure {

using nlohmann::json;

namespace {

// Threshold above which we classify the cure as "autoclave"; below = "oven".
constexpr double kAutoclavePressureThresholdKpa = 100.0;

const char* inferMethod(double pressure_kpa)
{
    return (pressure_kpa > kAutoclavePressureThresholdKpa) ? "autoclave" : "oven";
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.temp_c < 80.0 || in.temp_c > 220.0) {
        r.add("DFM-CURE-TEMP", "error",
              std::string(kSkillId) + ": temp_c " +
              std::to_string(in.temp_c) +
              " outside epoxy cure window [80, 220] °C");
    }
    if (in.time_min < 5.0) {
        r.add("DFM-CURE-TIME", "error",
              std::string(kSkillId) + ": time_min " +
              std::to_string(in.time_min) + " < 5 — under-cure risk");
    }
    if (in.pressure_kpa < 0.0 || in.pressure_kpa > 1500.0) {
        r.add("DFM-CURE-PRESS", "error",
              std::string(kSkillId) + ": pressure_kpa " +
              std::to_string(in.pressure_kpa) +
              " outside autoclave range [0, 1500] kPa");
    }
    r.add("DFM-CURE-METHOD", "info",
          std::string(kSkillId) + ": inferred cure method = " +
          inferMethod(in.pressure_kpa) +
          " (pressure_kpa " + std::to_string(in.pressure_kpa) + ")");

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "composite_cure DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const std::string method = inferMethod(in.pressure_kpa);

    json params = {
        { "temp_c",       in.temp_c },
        { "time_min",     in.time_min },
        { "pressure_kpa", in.pressure_kpa },
        { "vacuum",       in.vacuum },
    };

    json pattern = {
        { "kind",              kSkillId },
        { "temp_c",            in.temp_c },
        { "time_min",          in.time_min },
        { "pressure_kpa",      in.pressure_kpa },
        { "vacuum",            in.vacuum },
        { "cure_method",       method },
        { "geometric_change",  false },
        { "process_category",  "composite" },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code", f.code },
                            { "severity", f.severity },
                            { "message", f.message }});

    ToolingMeta tooling;
    tooling.tool_type         = method;   // "autoclave" or "oven"
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "refractory_lining";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    // Cycle ≈ ramp (~ 30 min) + hold + cool (~ 30 min).
    tooling.est_cycle_time_s  = (30.0 + in.time_min + 30.0) * 60.0;
    tooling.extra = {
        { "temp_c",            in.temp_c },
        { "time_min",          in.time_min },
        { "pressure_kpa",      in.pressure_kpa },
        { "vacuum",            in.vacuum },
        { "cure_method",       method },
        { "dfm_findings",      findings },
        { "process_category",  "composite_cure" },
        { "geometric_no_op",   true },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug("skill::composite_cure applied: T={}°C t={}min p={}kPa method={}",
                  in.temp_c, in.time_min, in.pressure_kpa, method);

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

}  // namespace koocadcam::skill::composite_cure
