// @lat: [[engine/skills#edge_prep_hone]]

#include "edge_prep_hone.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <string>

namespace koocadcam::skill::edge_prep_hone {

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
    if (in.target_radius_um <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) +
              ": target_radius_um must be > 0 (got " +
              std::to_string(in.target_radius_um) + ")");
    }

    if (in.target_radius_um > 0.0 &&
        (in.target_radius_um < 3.0 || in.target_radius_um > 40.0)) {
        r.add("DFM-EP-RADIUS", "info",
              std::string(kSkillId) + ": target_radius_um " +
              std::to_string(in.target_radius_um) +
              " outside typical [3, 40] um range");
    }
    if (in.asymmetry_factor < 0.5 || in.asymmetry_factor > 3.0) {
        r.add("DFM-EP-ASYM", "warning",
              std::string(kSkillId) + ": asymmetry_factor " +
              std::to_string(in.asymmetry_factor) +
              " outside typical [0.5, 3.0] range");
    }

    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "edge_prep_hone DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const std::string edge_profile =
        (in.asymmetry_factor > 1.05) ? "waterfall" :
        (in.asymmetry_factor < 0.95) ? "trumpet" : "symmetric";

    json params = {
        { "tool_id",          in.tool_id },
        { "target_radius_um", in.target_radius_um },
        { "asymmetry_factor", in.asymmetry_factor },
    };
    json pattern = {
        { "kind",              kSkillId },
        { "tool_id",           in.tool_id },
        { "target_radius_um",  in.target_radius_um },
        { "asymmetry_factor",  in.asymmetry_factor },
        { "edge_profile",      edge_profile },
        { "process",           "micro_hone" },
        { "geometry_changed",  false },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code", f.code },
                            { "severity", f.severity },
                            { "message", f.message }});

    ToolingMeta tooling;
    tooling.tool_type         = "drag_finisher";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "n/a";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    // Drag / brush hone cycle ~ 5-15 min per tool.
    tooling.est_cycle_time_s  = 600.0;
    tooling.extra["process"]          = "edge_prep_hone";
    tooling.extra["tool_id"]          = in.tool_id;
    tooling.extra["target_radius_um"] = in.target_radius_um;
    tooling.extra["asymmetry_factor"] = in.asymmetry_factor;
    tooling.extra["edge_profile"]     = edge_profile;
    tooling.extra["dfm_findings"]     = findings;

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug("skill::edge_prep_hone applied: tool={} r={}um asym={}",
                  in.tool_id, in.target_radius_um, in.asymmetry_factor);

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

}  // namespace koocadcam::skill::edge_prep_hone
