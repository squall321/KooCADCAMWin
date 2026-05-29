// @lat: [[engine/skills#cryogenic_anneal]]

#include "cryogenic_anneal.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <string>

namespace koocadcam::skill::cryogenic_anneal {

using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.cold_temp_k <= 0.0) {
        r.add("DFM-CRYO-TEMP", "error",
              std::string(kSkillId) +
              " cold_temp_k must be > 0 K (got " +
              std::to_string(in.cold_temp_k) + ")");
    } else if (in.cold_temp_k > 200.0) {
        r.add("DFM-CRYO-TEMP", "error",
              std::string(kSkillId) +
              " cold_temp_k " + std::to_string(in.cold_temp_k) +
              " K > 200 K — outside cryogenic regime, use cold_treat instead");
    }

    if (in.hold_h <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) +
              " hold_h must be > 0 (got " + std::to_string(in.hold_h) + ")");
    }

    if (in.ramp_k_min <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) +
              " ramp_k_min must be > 0");
    } else if (in.ramp_k_min > 5.0) {
        r.add("DFM-CRYO-RAMP", "warning",
              std::string(kSkillId) +
              " ramp_k_min " + std::to_string(in.ramp_k_min) +
              " > 5 K/min — risk of thermal shock / microcracking");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "cryogenic_anneal DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    json params = {
        { "cold_temp_k", in.cold_temp_k },
        { "hold_h",      in.hold_h },
        { "ramp_k_min",  in.ramp_k_min },
    };
    json pattern = {
        { "kind",             kSkillId },
        { "cold_temp_k",      in.cold_temp_k },
        { "hold_h",           in.hold_h },
        { "ramp_k_min",       in.ramp_k_min },
        { "thermal_cycle",    "ambient -> cold -> ambient" },
        { "geometry_changed", false },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "cryogenic_chamber";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "stainless_steel_dewar";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    // Two ramps (down + up) plus the hold.
    {
        const double rampDownMin = (293.0 - in.cold_temp_k) / in.ramp_k_min;
        const double rampUpMin   = rampDownMin;
        const double holdMin     = in.hold_h * 60.0;
        tooling.est_cycle_time_s = (rampDownMin + holdMin + rampUpMin) * 60.0;
    }
    tooling.extra = {
        { "process",     "cryogenic_anneal" },
        { "cold_temp_k", in.cold_temp_k },
        { "hold_h",      in.hold_h },
        { "ramp_k_min",  in.ramp_k_min },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::cryogenic_anneal applied: T={}K hold={}h ramp={}K/min",
                  in.cold_temp_k, in.hold_h, in.ramp_k_min);

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
        rf.matched_geometry = { { "source", "metadata" } };
        out.push_back(rf);
    }
    return out;
}

}  // namespace koocadcam::skill::cryogenic_anneal
