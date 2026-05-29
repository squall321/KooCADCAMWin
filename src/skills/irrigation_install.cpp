// @lat: [[engine/skills#irrigation_install]]

#include "irrigation_install.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <memory>
#include <string>

namespace koocadcam::skill::irrigation_install {

using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;
    (void)wp;

    if (in.system_type != "drip" &&
        in.system_type != "sprinkler" &&
        in.system_type != "flood") {
        r.add("DFM-IRRIG-TYPE", "error",
              std::string(kSkillId) + ": system_type '" + in.system_type +
              "' not in {drip, sprinkler, flood}");
    }

    if (in.area_ha <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": area_ha must be > 0 (got " +
              std::to_string(in.area_ha) + ")");
    }

    if (in.flow_l_h <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": flow_l_h must be > 0 (got " +
              std::to_string(in.flow_l_h) + ")");
    }

    if (in.system_type == "flood") {
        r.add("DFM-IRRIG-FLOOD", "info",
              std::string(kSkillId) +
              ": flood system — flow_l_h applies per inlet, not per emitter");
    }

    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "irrigation_install DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    json params = {
        { "system_type", in.system_type },
        { "area_ha",     in.area_ha },
        { "flow_l_h",    in.flow_l_h },
    };
    json pattern = {
        { "kind",             kSkillId },
        { "system_type",      in.system_type },
        { "area_ha",          in.area_ha },
        { "flow_l_h",         in.flow_l_h },
        { "geometry_changed", false },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code", f.code },
                            { "severity", f.severity },
                            { "message", f.message }});

    ToolingMeta tooling;
    if (in.system_type == "drip") {
        tooling.tool_type = "drip_emitter_kit";
    } else if (in.system_type == "sprinkler") {
        tooling.tool_type = "sprinkler_head_kit";
    } else {
        tooling.tool_type = "flood_inlet_kit";
    }
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "polyethylene";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    // Install ~ 8 h/ha drip, 4 h/ha sprinkler, 2 h/ha flood inlet.
    {
        double hPerHa = (in.system_type == "drip")      ? 8.0
                      : (in.system_type == "sprinkler") ? 4.0
                                                        : 2.0;
        tooling.est_cycle_time_s = hPerHa * in.area_ha * 3600.0;
    }
    tooling.extra = {
        { "process",          "irrigation_install" },
        { "system_type",      in.system_type },
        { "area_ha",          in.area_ha },
        { "flow_l_h",         in.flow_l_h },
        { "process_category", "agriculture" },
        { "geometric_no_op",  true },
        { "dfm_findings",     findings },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug(
        "skill::irrigation_install applied: type={} area={}ha flow={}l/h",
        in.system_type, in.area_ha, in.flow_l_h);

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

}  // namespace koocadcam::skill::irrigation_install
