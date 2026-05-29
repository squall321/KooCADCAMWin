// @lat: [[engine/skills#tool_balancing]]

#include "tool_balancing.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <string>

namespace koocadcam::skill::tool_balancing {

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
    if (in.grade_g_mm_kg <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) +
              ": grade_g_mm_kg must be > 0 (got " +
              std::to_string(in.grade_g_mm_kg) + ")");
    }
    if (in.target_rpm <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) +
              ": target_rpm must be > 0 (got " +
              std::to_string(in.target_rpm) + ")");
    }

    if (in.grade_g_mm_kg > 0.0 &&
        (in.grade_g_mm_kg < 0.4 || in.grade_g_mm_kg > 16.0)) {
        r.add("DFM-TB-GRADE", "info",
              std::string(kSkillId) + ": grade_g_mm_kg " +
              std::to_string(in.grade_g_mm_kg) +
              " outside typical [0.4, 16] g·mm/kg range");
    }
    if (in.target_rpm > 0.0 &&
        (in.target_rpm < 3000.0 || in.target_rpm > 100000.0)) {
        r.add("DFM-TB-RPM", "info",
              std::string(kSkillId) + ": target_rpm " +
              std::to_string(in.target_rpm) +
              " outside typical [3000, 100000] rpm range");
    }

    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "tool_balancing DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    // Grade classification (informative).
    const std::string grade_class =
        (in.grade_g_mm_kg <= 1.0)  ? "precision" :
        (in.grade_g_mm_kg <= 2.5)  ? "high"      :
        (in.grade_g_mm_kg <= 6.3)  ? "standard"  : "coarse";

    json params = {
        { "tool_id",       in.tool_id },
        { "grade_g_mm_kg", in.grade_g_mm_kg },
        { "target_rpm",    in.target_rpm },
    };
    json pattern = {
        { "kind",              kSkillId },
        { "tool_id",           in.tool_id },
        { "grade_g_mm_kg",     in.grade_g_mm_kg },
        { "target_rpm",        in.target_rpm },
        { "grade_class",       grade_class },
        { "process",           "tool_dynamic_balancing" },
        { "geometry_changed",  false },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code", f.code },
                            { "severity", f.severity },
                            { "message", f.message }});

    ToolingMeta tooling;
    tooling.tool_type         = "tool_balancing_machine";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "n/a";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    // Setup + 2 measurement runs + 1 correction ~ 8-12 min per tool.
    tooling.est_cycle_time_s  = 600.0;
    tooling.extra["process"]        = "tool_balancing";
    tooling.extra["tool_id"]        = in.tool_id;
    tooling.extra["grade_g_mm_kg"]  = in.grade_g_mm_kg;
    tooling.extra["target_rpm"]     = in.target_rpm;
    tooling.extra["grade_class"]    = grade_class;
    tooling.extra["dfm_findings"]   = findings;

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug("skill::tool_balancing applied: tool={} grade={} g·mm/kg rpm={}",
                  in.tool_id, in.grade_g_mm_kg, in.target_rpm);

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

}  // namespace koocadcam::skill::tool_balancing
