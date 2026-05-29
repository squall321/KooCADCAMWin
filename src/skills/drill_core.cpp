// @lat: [[engine/skills#drill_core]]

#include "drill_core.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <string>
#include <unordered_set>

namespace koocadcam::skill::drill_core {

using nlohmann::json;

namespace {

const std::unordered_set<std::string>& knownMethods()
{
    static const std::unordered_set<std::string> s = {
        "rotary", "percussive", "diamond",
    };
    return s;
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.hole_dia_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "drill_core hole_dia_mm must be > 0 (got " +
              std::to_string(in.hole_dia_mm) + ")");
    }

    if (in.depth_m <= 0.0) {
        r.add("DFM-INPUT", "error",
              "drill_core depth_m must be > 0 (got " +
              std::to_string(in.depth_m) + ")");
    }

    if (in.drill_method.empty() || knownMethods().count(in.drill_method) == 0) {
        r.add("DFM-DRILL-METHOD", "error",
              "drill_core unknown drill_method '" + in.drill_method +
              "' (expected rotary | percussive | diamond)");
    }

    if (in.hole_dia_mm > 200.0) {
        r.add("DFM-CORE-DIA-HIGH", "info",
              "drill_core hole_dia_mm " + std::to_string(in.hole_dia_mm) +
              " > 200 mm — outsize industrial bore");
    }

    if (in.depth_m > 2000.0) {
        r.add("DFM-CORE-DEPTH-HIGH", "info",
              "drill_core depth_m " + std::to_string(in.depth_m) +
              " > 2000 m — deep exploration; verify rod string + mud plan");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "drill_core DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    json params = {
        { "hole_dia_mm",  in.hole_dia_mm },
        { "depth_m",      in.depth_m },
        { "drill_method", in.drill_method },
    };

    json pattern = {
        { "kind",             kSkillId },
        { "hole_dia_mm",      in.hole_dia_mm },
        { "depth_m",          in.depth_m },
        { "drill_method",     in.drill_method },
        { "process_family",   "exploration_drilling" },
        { "geometry_changed", false },
        { "is_process_only",  true },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code",     f.code },
                            { "severity", f.severity },
                            { "message",  f.message }});

    ToolingMeta tooling;
    if      (in.drill_method == "rotary")     tooling.tool_type = "rotary_tricone_bit";
    else if (in.drill_method == "percussive") tooling.tool_type = "down_the_hole_hammer";
    else                                       tooling.tool_type = "diamond_core_bit";
    tooling.tool_dia_mm       = in.hole_dia_mm;
    tooling.tool_length_mm    = in.depth_m * 1000.0;
    tooling.tool_material     = (in.drill_method == "diamond") ? "impregnated_diamond"
                                                               : "tungsten_carbide";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    // Rough rate-of-penetration: diamond 5 m/h, rotary 20 m/h, percussive 30 m/h.
    double rop_m_per_h = 20.0;
    if      (in.drill_method == "diamond")    rop_m_per_h =  5.0;
    else if (in.drill_method == "percussive") rop_m_per_h = 30.0;
    tooling.est_cycle_time_s  = (in.depth_m / rop_m_per_h) * 3600.0;
    tooling.extra = {
        { "process",          "core_drilling" },
        { "hole_dia_mm",      in.hole_dia_mm },
        { "depth_m",          in.depth_m },
        { "drill_method",     in.drill_method },
        { "process_category", "mining_exploration" },
        { "dfm_findings",     findings },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::drill_core applied: dia={}mm depth={}m method={}",
                  in.hole_dia_mm, in.depth_m, in.drill_method);

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

}  // namespace koocadcam::skill::drill_core
