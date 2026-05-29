// @lat: [[engine/skills#tool_grind]]

#include "tool_grind.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <string>

namespace koocadcam::skill::tool_grind {

using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;
    (void)wp;

    if (in.tool_id.empty()) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) +
              ": tool_id must not be empty (tool traceability)");
    }
    if (in.target_edge_radius_um <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) +
              ": target_edge_radius_um must be > 0 (got " +
              std::to_string(in.target_edge_radius_um) + ")");
    }

    if (in.rake_angle_deg < -30.0 || in.rake_angle_deg > 30.0) {
        r.add("DFM-TG-RAKE", "warning",
              std::string(kSkillId) + ": rake_angle_deg " +
              std::to_string(in.rake_angle_deg) +
              " outside typical [-30, +30] range");
    }
    if (in.relief_angle_deg < 0.0 || in.relief_angle_deg > 25.0) {
        r.add("DFM-TG-RELIEF", "warning",
              std::string(kSkillId) + ": relief_angle_deg " +
              std::to_string(in.relief_angle_deg) +
              " outside typical [0, 25] range");
    }
    if (in.target_edge_radius_um > 0.0 &&
        (in.target_edge_radius_um < 2.0 || in.target_edge_radius_um > 50.0)) {
        r.add("DFM-TG-RADIUS", "info",
              std::string(kSkillId) + ": target_edge_radius_um " +
              std::to_string(in.target_edge_radius_um) +
              " outside typical [2, 50] um range");
    }

    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "tool_grind DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    json params = {
        { "tool_id",               in.tool_id },
        { "rake_angle_deg",        in.rake_angle_deg },
        { "relief_angle_deg",      in.relief_angle_deg },
        { "target_edge_radius_um", in.target_edge_radius_um },
    };
    json pattern = {
        { "kind",                  kSkillId },
        { "tool_id",               in.tool_id },
        { "rake_angle_deg",        in.rake_angle_deg },
        { "relief_angle_deg",      in.relief_angle_deg },
        { "target_edge_radius_um", in.target_edge_radius_um },
        { "process",               "cutting_tool_preparation" },
        { "geometry_changed",      false },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code", f.code },
                            { "severity", f.severity },
                            { "message", f.message }});

    ToolingMeta tooling;
    tooling.tool_type         = "tool_cutter_grinder";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "n/a";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    // Typical cycle: setup 5 min + grind 5-15 min depending on edge complexity.
    tooling.est_cycle_time_s  = 720.0;
    tooling.extra["process"]               = "tool_grind";
    tooling.extra["tool_id"]               = in.tool_id;
    tooling.extra["rake_angle_deg"]        = in.rake_angle_deg;
    tooling.extra["relief_angle_deg"]      = in.relief_angle_deg;
    tooling.extra["target_edge_radius_um"] = in.target_edge_radius_um;
    tooling.extra["dfm_findings"]          = findings;

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug("skill::tool_grind applied: tool={} rake={} relief={} edge_r={}um",
                  in.tool_id, in.rake_angle_deg,
                  in.relief_angle_deg, in.target_edge_radius_um);

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

}  // namespace koocadcam::skill::tool_grind
