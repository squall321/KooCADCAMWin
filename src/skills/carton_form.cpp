// @lat: [[engine/skills#carton_form]]

#include "carton_form.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <string>
#include <unordered_set>

namespace koocadcam::skill::carton_form {

using nlohmann::json;

namespace {

const std::unordered_set<std::string>& knownFoldPatterns()
{
    static const std::unordered_set<std::string> s = {
        "rte", "ste", "alb",
    };
    return s;
}

const std::unordered_set<std::string>& knownGlueTypes()
{
    static const std::unordered_set<std::string> s = {
        "hot_melt", "cold_glue", "tab_lock",
    };
    return s;
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.carton_size_mm[0] <= 0.0 ||
        in.carton_size_mm[1] <= 0.0 ||
        in.carton_size_mm[2] <= 0.0)
    {
        r.add("DFM-INPUT", "error",
              "carton_form carton_size_mm must have all positive dimensions "
              "(got W=" + std::to_string(in.carton_size_mm[0]) +
              " H=" + std::to_string(in.carton_size_mm[1]) +
              " D=" + std::to_string(in.carton_size_mm[2]) + ")");
    }

    if (in.fold_pattern.empty()) {
        r.add("DFM-INPUT", "error",
              "carton_form fold_pattern must be specified");
    } else if (knownFoldPatterns().count(in.fold_pattern) == 0) {
        r.add("DFM-CARTON-PATTERN", "info",
              "carton_form fold_pattern '" + in.fold_pattern +
              "' not in known catalog (rte | ste | alb)");
    }

    if (in.glue_type.empty()) {
        r.add("DFM-INPUT", "error",
              "carton_form glue_type must be specified");
    } else if (knownGlueTypes().count(in.glue_type) == 0) {
        r.add("DFM-CARTON-GLUE", "info",
              "carton_form glue_type '" + in.glue_type +
              "' not in known catalog (hot_melt | cold_glue | tab_lock)");
    }

    for (int i = 0; i < 3; ++i) {
        if (in.carton_size_mm[i] > 600.0) {
            r.add("DFM-CARTON-SIZE", "info",
                  "carton_form dimension " + std::to_string(in.carton_size_mm[i]) +
                  " mm > 600 — non-standard large carton, custom tooling required");
            break;
        }
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "carton_form DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    json size = json::array(
        { in.carton_size_mm[0], in.carton_size_mm[1], in.carton_size_mm[2] });

    json params = {
        { "carton_size_mm", size },
        { "fold_pattern",   in.fold_pattern },
        { "glue_type",      in.glue_type },
    };

    json pattern = {
        { "kind",             kSkillId },
        { "carton_size_mm",   size },
        { "fold_pattern",     in.fold_pattern },
        { "glue_type",        in.glue_type },
        { "process_family",   "carton_erecting" },
        { "geometry_changed", false },
        { "is_process_only",  true },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code",     f.code },
                            { "severity", f.severity },
                            { "message",  f.message }});

    ToolingMeta tooling;
    tooling.tool_type         = "carton_erector";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "stainless_jaws";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    // Auto-lock-bottom is fastest; hot-melt requires glue dwell.
    tooling.est_cycle_time_s  = (in.glue_type == "tab_lock") ? 1.5 : 3.0;
    tooling.extra = {
        { "process",          "carton_form" },
        { "carton_size_mm",   size },
        { "fold_pattern",     in.fold_pattern },
        { "glue_type",        in.glue_type },
        { "process_category", "packaging" },
        { "dfm_findings",     findings },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug("skill::carton_form applied: size=({},{},{}) fold={} glue={}",
                  in.carton_size_mm[0], in.carton_size_mm[1], in.carton_size_mm[2],
                  in.fold_pattern, in.glue_type);

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

}  // namespace koocadcam::skill::carton_form
