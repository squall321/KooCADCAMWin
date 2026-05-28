// @lat: [[engine/skills#resin_transfer_mold]]

#include "resin_transfer_mold.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <string>

namespace koocadcam::skill::resin_transfer_mold {

using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.injection_pressure_kpa < 100.0 || in.injection_pressure_kpa > 3000.0) {
        r.add("DFM-RTM-PRESS", "error",
              std::string(kSkillId) + ": injection_pressure_kpa " +
              std::to_string(in.injection_pressure_kpa) +
              " outside RTM practical range [100, 3000] kPa");
    }
    if (in.gate_count < 1) {
        r.add("DFM-RTM-GATES", "error",
              std::string(kSkillId) + ": gate_count must be ≥ 1 (got " +
              std::to_string(in.gate_count) + ")");
    }
    if (in.vent_count < 1) {
        r.add("DFM-RTM-VENTS", "error",
              std::string(kSkillId) + ": vent_count must be ≥ 1 (got " +
              std::to_string(in.vent_count) + ")");
    }
    if (in.vent_count >= 1 && in.gate_count >= 1) {
        const double ratio = static_cast<double>(in.gate_count) /
                             static_cast<double>(in.vent_count);
        if (ratio > 5.0) {
            r.add("DFM-RTM-RATIO", "info",
                  std::string(kSkillId) + ": gates/vents ratio " +
                  std::to_string(ratio) +
                  " > 5:1 — risk of dry spots / trapped air");
        }
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "resin_transfer_mold DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    json params = {
        { "injection_pressure_kpa", in.injection_pressure_kpa },
        { "gate_count",             in.gate_count },
        { "vent_count",             in.vent_count },
    };

    json pattern = {
        { "kind",                    kSkillId },
        { "injection_pressure_kpa",  in.injection_pressure_kpa },
        { "gate_count",              in.gate_count },
        { "vent_count",              in.vent_count },
        { "geometric_change",        false },
        { "process_category",        "composite" },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code", f.code },
                            { "severity", f.severity },
                            { "message", f.message }});

    ToolingMeta tooling;
    tooling.tool_type         = "rtm_mold";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "steel_p20";   // typical RTM mold body
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    // Fill-time loosely scales with sqrt(area / pressure).  Default 5 min.
    tooling.est_cycle_time_s  = 300.0;
    tooling.extra = {
        { "injection_pressure_kpa", in.injection_pressure_kpa },
        { "gate_count",             in.gate_count },
        { "vent_count",             in.vent_count },
        { "dfm_findings",           findings },
        { "process_category",       "resin_transfer_mold" },
        { "geometric_no_op",        true },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug("skill::resin_transfer_mold applied: p={}kPa gates={} vents={}",
                  in.injection_pressure_kpa, in.gate_count, in.vent_count);

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

}  // namespace koocadcam::skill::resin_transfer_mold
