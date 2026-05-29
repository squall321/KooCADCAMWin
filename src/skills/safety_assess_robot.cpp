// @lat: [[engine/skills#safety_assess_robot]]

#include "safety_assess_robot.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <set>
#include <string>

namespace koocadcam::skill::safety_assess_robot {

using nlohmann::json;

namespace {

bool isValidPL(const std::string& c)
{
    static const std::set<std::string> all = { "PLa", "PLb", "PLc", "PLd", "PLe" };
    return all.count(c) > 0;
}

bool isCollabPL(const std::string& c)
{
    return c == "PLd" || c == "PLe";
}

bool isValidRisk(const std::string& c)
{
    static const std::set<std::string> all = { "low", "medium", "high" };
    return all.count(c) > 0;
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (!isValidPL(in.safety_category)) {
        r.add("DFM-INPUT", "error",
              "safety_assess_robot safety_category '" + in.safety_category +
              "' invalid — expected one of {PLa, PLb, PLc, PLd, PLe}");
    } else if (!isCollabPL(in.safety_category)) {
        r.add("DFM-SAFETY-CAT", "error",
              "safety_assess_robot safety_category '" + in.safety_category +
              "' below PLd — collaborative robot cells require ISO 13849 PLd or PLe");
    }

    if (!isValidRisk(in.risk_class)) {
        r.add("DFM-INPUT", "error",
              "safety_assess_robot risk_class '" + in.risk_class +
              "' invalid — expected one of {low, medium, high}");
    }

    if (in.risk_class == "high" && in.safety_category == "PLd") {
        r.add("DFM-SAFETY-MISMATCH", "info",
              "safety_assess_robot: risk_class=high with PLd — prefer PLe "
              "to provide additional redundancy for high-risk tasks");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "safety_assess_robot DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    json params = {
        { "safety_category", in.safety_category },
        { "risk_class",      in.risk_class },
    };
    json pattern = {
        { "kind",             kSkillId },
        { "safety_category",  in.safety_category },
        { "risk_class",       in.risk_class },
        { "standard",         "ISO_13849-1" },
        { "geometry_changed", false },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "safety_audit";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "n/a";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    tooling.est_cycle_time_s  = 0.0;   // audit task, no production cycle contribution
    tooling.extra = {
        { "process",         "robot_safety_assessment" },
        { "standard",        "ISO_13849-1" },
        { "safety_category", in.safety_category },
        { "risk_class",      in.risk_class },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::safety_assess_robot applied: cat={} risk={}",
                  in.safety_category, in.risk_class);

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

}  // namespace koocadcam::skill::safety_assess_robot
