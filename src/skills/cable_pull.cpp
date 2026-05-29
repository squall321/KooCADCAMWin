// @lat: [[engine/skills#cable_pull]]

#include "cable_pull.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <memory>
#include <string>

namespace koocadcam::skill::cable_pull {

using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;
    (void)wp;

    if (in.cable_type.empty()) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": cable_type must be non-empty");
    }
    if (in.length_m <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": length_m must be > 0 (got " +
              std::to_string(in.length_m) + ")");
    }
    if (in.pull_force_n <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": pull_force_n must be > 0 (got " +
              std::to_string(in.pull_force_n) + ")");
    } else if (in.pull_force_n > 600.0) {
        r.add("DFM-CABLE-FORCE", "warning",
              std::string(kSkillId) + ": pull_force_n " +
              std::to_string(in.pull_force_n) +
              " exceeds 600 N typical telecom tension limit");
    }

    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "cable_pull DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    json params = {
        { "cable_type",   in.cable_type },
        { "length_m",     in.length_m },
        { "pull_force_n", in.pull_force_n },
    };
    json pattern = {
        { "kind",             kSkillId },
        { "cable_type",       in.cable_type },
        { "length_m",         in.length_m },
        { "pull_force_n",     in.pull_force_n },
        { "geometry_changed", false },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code", f.code },
                            { "severity", f.severity },
                            { "message", f.message }});

    ToolingMeta tooling;
    tooling.tool_type         = "cable_puller";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "n/a";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    // Pull speed ~ 0.5 m/s on grease/lube + 30 s setup.
    tooling.est_cycle_time_s  = 30.0 + in.length_m / 0.5;
    tooling.extra = {
        { "process",          "telecom_cable_pull" },
        { "cable_type",       in.cable_type },
        { "length_m",         in.length_m },
        { "pull_force_n",     in.pull_force_n },
        { "process_category", "telecom_install" },
        { "geometric_no_op",  true },
        { "dfm_findings",     findings },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug(
        "skill::cable_pull applied: type={} len={}m force={}N",
        in.cable_type, in.length_m, in.pull_force_n);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// Metadata-only: leaves no geometric trace on the workpiece.

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

}  // namespace koocadcam::skill::cable_pull
