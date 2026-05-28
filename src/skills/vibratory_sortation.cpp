// @lat: [[engine/skills#vibratory_sortation]]

#include "vibratory_sortation.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <memory>
#include <string>

namespace koocadcam::skill::vibratory_sortation {

using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;
    (void)wp;

    if (in.feed_rate_per_min <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": feed_rate_per_min must be > 0 (got " +
              std::to_string(in.feed_rate_per_min) + ")");
    } else if (in.feed_rate_per_min > 600.0) {
        r.add("DFM-VIB-RATE", "info",
              std::string(kSkillId) + ": feed_rate_per_min " +
              std::to_string(in.feed_rate_per_min) +
              " exceeds typical industrial cap (600/min)");
    }

    if (in.orientation_quality_pct < 0.0 || in.orientation_quality_pct > 100.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": orientation_quality_pct " +
              std::to_string(in.orientation_quality_pct) +
              " outside [0, 100]");
    } else if (in.orientation_quality_pct < 50.0) {
        r.add("DFM-VIB-YIELD", "error",
              std::string(kSkillId) + ": orientation_quality_pct " +
              std::to_string(in.orientation_quality_pct) +
              " < 50 — bowl tooling needs re-design");
    } else if (in.orientation_quality_pct < 80.0) {
        r.add("DFM-VIB-YIELD", "warning",
              std::string(kSkillId) + ": orientation_quality_pct " +
              std::to_string(in.orientation_quality_pct) +
              " < 80 — track / kicker tuning required");
    }

    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "vibratory_sortation DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    // Cycle time per oriented part.  feed_rate_per_min > 0 guaranteed.
    const double per_part_s = 60.0 / in.feed_rate_per_min;

    json params = {
        { "feed_rate_per_min",       in.feed_rate_per_min },
        { "orientation_quality_pct", in.orientation_quality_pct },
    };
    json pattern = {
        { "kind",                     kSkillId },
        { "feed_rate_per_min",        in.feed_rate_per_min },
        { "orientation_quality_pct",  in.orientation_quality_pct },
        { "cycle_time_per_part_s",    per_part_s },
        { "geometry_changed",         false },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code", f.code },
                            { "severity", f.severity },
                            { "message", f.message }});

    ToolingMeta tooling;
    tooling.tool_type         = "vibratory_bowl_feeder";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "spring_steel_bowl";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    tooling.est_cycle_time_s  = per_part_s;
    tooling.extra = {
        { "process",                    "vibratory_feeding_and_sortation" },
        { "feed_rate_per_min",          in.feed_rate_per_min },
        { "orientation_quality_pct",    in.orientation_quality_pct },
        { "process_category",           "material_handling" },
        { "geometric_no_op",            true },
        { "dfm_findings",               findings },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug(
        "skill::vibratory_sortation applied: rate={}/min quality={}%",
        in.feed_rate_per_min, in.orientation_quality_pct);

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

}  // namespace koocadcam::skill::vibratory_sortation
