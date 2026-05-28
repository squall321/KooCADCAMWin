// @lat: [[engine/skills#ceramic_sinter]]

#include "ceramic_sinter.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <string>
#include <unordered_set>

namespace koocadcam::skill::ceramic_sinter {

using nlohmann::json;

namespace {

const std::unordered_set<std::string>& knownAtmospheres()
{
    static const std::unordered_set<std::string> s = {
        "air", "argon", "nitrogen", "vacuum", "hydrogen",
    };
    return s;
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.temp_peak_c < 800.0 || in.temp_peak_c > 2000.0) {
        r.add("DFM-SINTER-TEMP", "error",
              std::string(kSkillId) + ": temp_peak_c " +
              std::to_string(in.temp_peak_c) +
              " outside typical ceramic sinter range [800, 2000] °C");
    }
    if (in.dwell_min < 5.0) {
        r.add("DFM-SINTER-DWELL", "error",
              std::string(kSkillId) + ": dwell_min " +
              std::to_string(in.dwell_min) + " < 5 — incomplete densification");
    }
    if (knownAtmospheres().count(in.atmosphere) == 0) {
        r.add("DFM-SINTER-ATMOS", "error",
              std::string(kSkillId) + ": atmosphere '" + in.atmosphere +
              "' unrecognised (expected air | argon | nitrogen | vacuum | hydrogen)");
    }
    if (in.shrinkage_pct < 0.0 || in.shrinkage_pct > 30.0) {
        r.add("DFM-SINTER-SHRINK", "error",
              std::string(kSkillId) + ": shrinkage_pct " +
              std::to_string(in.shrinkage_pct) +
              " outside typical [0, 30] %% range");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "ceramic_sinter DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    // Pre-sinter (green) linear scale = 1 / (1 - shrinkage/100).
    const double linShrinkFactor = 1.0 / (1.0 - in.shrinkage_pct / 100.0);

    json params = {
        { "temp_peak_c",   in.temp_peak_c },
        { "dwell_min",     in.dwell_min },
        { "atmosphere",    in.atmosphere },
        { "shrinkage_pct", in.shrinkage_pct },
    };

    json pattern = {
        { "kind",                       kSkillId },
        { "temp_peak_c",                in.temp_peak_c },
        { "dwell_min",                  in.dwell_min },
        { "atmosphere",                 in.atmosphere },
        { "shrinkage_pct",              in.shrinkage_pct },
        { "green_to_fired_scale_factor", linShrinkFactor },
        { "geometric_change",           false },
        { "process_category",           "ceramic" },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code", f.code },
                            { "severity", f.severity },
                            { "message", f.message }});

    ToolingMeta tooling;
    tooling.tool_type         = "kiln";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "refractory_lining";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    // Ramp (slow, ~ 2 h to high temp) + dwell + slow cool (~ 4 h).
    tooling.est_cycle_time_s  = (120.0 + in.dwell_min + 240.0) * 60.0;
    tooling.extra = {
        { "temp_peak_c",                in.temp_peak_c },
        { "dwell_min",                  in.dwell_min },
        { "atmosphere",                 in.atmosphere },
        { "shrinkage_pct",              in.shrinkage_pct },
        { "green_to_fired_scale_factor", linShrinkFactor },
        { "dfm_findings",               findings },
        { "process_category",           "ceramic_sinter" },
        { "geometric_no_op",            true },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug("skill::ceramic_sinter applied: T={}°C dwell={}min atm={} shrink={}%%",
                  in.temp_peak_c, in.dwell_min, in.atmosphere, in.shrinkage_pct);

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

}  // namespace koocadcam::skill::ceramic_sinter
