// @lat: [[engine/skills#sterilize]]

#include "sterilize.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <string>
#include <unordered_set>

namespace koocadcam::skill::sterilize {

using nlohmann::json;

namespace {

const std::unordered_set<std::string>& knownMethods()
{
    static const std::unordered_set<std::string> s = {
        "autoclave", "eto", "gamma", "ebeam", "h2o2_plasma",
    };
    return s;
}

// Per-method reasonable dose_or_temp envelope (advisory).
//   autoclave : 100 … 140 °C
//   eto       : 100 … 1200 mg/L
//   gamma     : 5   … 60   kGy
//   ebeam     : 5   … 50   kGy
//   h2o2_plasma: 1  … 10   (mg/L vapor concentration nominal)
bool doseInBand(const std::string& method, double d)
{
    if (method == "autoclave")   return d >= 100.0 && d <= 140.0;
    if (method == "eto")         return d >= 100.0 && d <= 1200.0;
    if (method == "gamma")       return d >= 5.0   && d <= 60.0;
    if (method == "ebeam")       return d >= 5.0   && d <= 50.0;
    if (method == "h2o2_plasma") return d >= 1.0   && d <= 10.0;
    return true;
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.method.empty() || knownMethods().count(in.method) == 0) {
        r.add("DFM-INPUT", "error",
              "sterilize unknown method '" + in.method +
              "' (expected autoclave | eto | gamma | ebeam | h2o2_plasma)");
    }

    if (in.exposure_min <= 0.0 || in.exposure_min > 1440.0) {
        r.add("DFM-STERILIZE-EXPO", "error",
              "sterilize exposure_min " + std::to_string(in.exposure_min) +
              " outside (0, 1440] minute range");
    }

    if (in.dose_or_temp <= 0.0) {
        r.add("DFM-STERILIZE-DOSE", "error",
              "sterilize dose_or_temp must be > 0 (got " +
              std::to_string(in.dose_or_temp) + ")");
    } else if (knownMethods().count(in.method) != 0 &&
               !doseInBand(in.method, in.dose_or_temp)) {
        r.add("DFM-STERILIZE-DOSE", "info",
              "sterilize dose_or_temp " + std::to_string(in.dose_or_temp) +
              " outside typical band for method '" + in.method + "'");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "sterilize DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    json params = {
        { "method",       in.method },
        { "dose_or_temp", in.dose_or_temp },
        { "exposure_min", in.exposure_min },
    };

    json pattern = {
        { "kind",                 kSkillId },
        { "method",               in.method },
        { "dose_or_temp",         in.dose_or_temp },
        { "exposure_min",         in.exposure_min },
        { "sal_target",           "10e-6" },
        { "process_family",       "sterilization" },
        { "geometric_change",     false },
        { "is_process_only",      true },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code",     f.code },
                            { "severity", f.severity },
                            { "message",  f.message }});

    ToolingMeta tooling;
    if      (in.method == "autoclave")   tooling.tool_type = "autoclave_chamber";
    else if (in.method == "eto")         tooling.tool_type = "eto_gas_chamber";
    else if (in.method == "gamma")       tooling.tool_type = "gamma_irradiator";
    else if (in.method == "ebeam")       tooling.tool_type = "ebeam_irradiator";
    else if (in.method == "h2o2_plasma") tooling.tool_type = "h2o2_plasma_sterilizer";
    else                                  tooling.tool_type = "sterilizer";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "stainless_316l";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    tooling.est_cycle_time_s  = in.exposure_min * 60.0;
    tooling.extra = {
        { "process",          "terminal_sterilization" },
        { "method",           in.method },
        { "dose_or_temp",     in.dose_or_temp },
        { "exposure_min",     in.exposure_min },
        { "sal_target",       "10e-6" },
        { "process_category", "sterilization" },
        { "dfm_findings",     findings },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug("skill::sterilize applied: method={} dose={} expo={}min",
                  in.method, in.dose_or_temp, in.exposure_min);

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

}  // namespace koocadcam::skill::sterilize
