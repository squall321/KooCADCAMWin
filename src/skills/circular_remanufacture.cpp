// @lat: [[engine/skills#circular_remanufacture]]

#include "circular_remanufacture.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <memory>
#include <string>

namespace koocadcam::skill::circular_remanufacture {

using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;
    (void)wp;

    if (in.core_id.empty()) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": core_id must be non-empty");
    }

    if (in.refurbish_steps.empty()) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": refurbish_steps must be non-empty");
    } else if (in.refurbish_steps.size() < 3u) {
        r.add("DFM-REMAN-STEPS", "info",
              std::string(kSkillId) + ": refurbish_steps count " +
              std::to_string(in.refurbish_steps.size()) +
              " below industry minimum of 3 (clean/inspect/replace)");
    }

    if (in.warranty_months <= 0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": warranty_months must be > 0 (got " +
              std::to_string(in.warranty_months) + ")");
    } else if (in.warranty_months > 60) {
        r.add("DFM-REMAN-WARRANTY", "info",
              std::string(kSkillId) + ": warranty_months " +
              std::to_string(in.warranty_months) +
              " exceeds 60 — verify against remanufacturing program policy");
    }

    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "circular_remanufacture DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    json stepsArr = json::array();
    for (const auto& s : in.refurbish_steps) stepsArr.push_back(s);

    json params = {
        { "core_id",         in.core_id },
        { "refurbish_steps", stepsArr },
        { "warranty_months", in.warranty_months },
    };
    json pattern = {
        { "kind",               kSkillId },
        { "core_id",            in.core_id },
        { "refurbish_steps",    stepsArr },
        { "warranty_months",    in.warranty_months },
        { "step_count",         static_cast<int>(in.refurbish_steps.size()) },
        { "geometry_changed",   false },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code", f.code },
                            { "severity", f.severity },
                            { "message", f.message }});

    ToolingMeta tooling;
    tooling.tool_type         = "remanufacture_cell";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "n/a";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    // ~600 s per refurbish step + 300 s intake/outbound.
    tooling.est_cycle_time_s  = 300.0 + 600.0 *
                                static_cast<double>(in.refurbish_steps.size());
    tooling.extra = {
        { "process",          "circular_remanufacture" },
        { "core_id",          in.core_id },
        { "step_count",       static_cast<int>(in.refurbish_steps.size()) },
        { "warranty_months",  in.warranty_months },
        { "process_category", "circular_economy" },
        { "geometric_no_op",  true },
        { "dfm_findings",     findings },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug(
        "skill::circular_remanufacture applied: core={} steps={} warranty={}mo",
        in.core_id, in.refurbish_steps.size(), in.warranty_months);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────

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

}  // namespace koocadcam::skill::circular_remanufacture
