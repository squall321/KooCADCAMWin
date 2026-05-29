// @lat: [[engine/skills#knit_apparel]]

#include "knit_apparel.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

namespace koocadcam::skill::knit_apparel {

using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.pattern_id.empty()) {
        r.add("DFM-INPUT", "error",
              "knit_apparel pattern_id must be non-empty");
    }
    if (in.gauge <= 0) {
        r.add("DFM-INPUT", "error",
              "knit_apparel gauge must be > 0 (got " +
              std::to_string(in.gauge) + ")");
    } else {
        if (in.gauge < 3) {
            r.add("DFM-KNIT-GAUGE", "warning",
                  "knit_apparel gauge " + std::to_string(in.gauge) +
                  " < 3 NPI — extremely coarse, fabric will be loose");
        }
        if (in.gauge > 36) {
            r.add("DFM-KNIT-GAUGE", "warning",
                  "knit_apparel gauge " + std::to_string(in.gauge) +
                  " > 36 NPI — exceeds typical flat-knit machinery");
        }
    }
    if (in.yarn_color.empty()) {
        r.add("DFM-INPUT", "warning",
              "knit_apparel yarn_color empty — defaulting to natural");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "knit_apparel DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const std::string color =
        in.yarn_color.empty() ? "natural" : in.yarn_color;

    json params = {
        { "gauge",      in.gauge },
        { "yarn_color", color },
        { "pattern_id", in.pattern_id },
    };
    json pattern = {
        { "kind",             kSkillId },
        { "gauge",            in.gauge },
        { "yarn_color",       color },
        { "pattern_id",       in.pattern_id },
        { "geometry_changed", false },
    };

    ToolingMeta tooling;
    // High gauge (>=18) → fine flat-knit; otherwise circular knit machine.
    tooling.tool_type         = (in.gauge >= 18) ? "flat_knit_machine"
                                                 : "circular_knit_machine";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "yarn";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    // Finer gauge runs slower; rough estimate per panel.
    tooling.est_cycle_time_s  = 30.0 * in.gauge;
    tooling.extra = {
        { "process",    "apparel_knit" },
        { "gauge",      in.gauge },
        { "yarn_color", color },
        { "pattern_id", in.pattern_id },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::knit_apparel applied: gauge={} color={} pattern={}",
                  in.gauge, color, in.pattern_id);

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

}  // namespace koocadcam::skill::knit_apparel
