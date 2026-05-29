// @lat: [[engine/skills#shrink_wrap]]

#include "shrink_wrap.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <string>
#include <unordered_set>

namespace koocadcam::skill::shrink_wrap {

using nlohmann::json;

namespace {

const std::unordered_set<std::string>& knownFilms()
{
    static const std::unordered_set<std::string> s = {
        "pvc", "pof", "pe",
    };
    return s;
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.film_type.empty() || knownFilms().count(in.film_type) == 0) {
        r.add("DFM-INPUT", "error",
              "shrink_wrap unknown film_type '" + in.film_type +
              "' (expected pvc | pof | pe)");
    }

    if (in.oven_temp_c <= 0.0) {
        r.add("DFM-INPUT", "error",
              "shrink_wrap oven_temp_c must be > 0 (got " +
              std::to_string(in.oven_temp_c) + ")");
    }

    if (in.dwell_time_s <= 0.0) {
        r.add("DFM-INPUT", "error",
              "shrink_wrap dwell_time_s must be > 0 (got " +
              std::to_string(in.dwell_time_s) + ")");
    }

    if (in.oven_temp_c > 0.0 && in.oven_temp_c < 100.0) {
        r.add("DFM-SHRINK-TEMP-LOW", "info",
              "shrink_wrap oven_temp_c " + std::to_string(in.oven_temp_c) +
              " °C < 100 — film may not fully activate (incomplete shrink)");
    } else if (in.oven_temp_c > 220.0) {
        r.add("DFM-SHRINK-TEMP-HIGH", "info",
              "shrink_wrap oven_temp_c " + std::to_string(in.oven_temp_c) +
              " °C > 220 — film burn / hole risk");
    }

    if (in.dwell_time_s > 30.0) {
        r.add("DFM-SHRINK-DWELL", "info",
              "shrink_wrap dwell_time_s " + std::to_string(in.dwell_time_s) +
              " > 30 s — risk of contents heat damage");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "shrink_wrap DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    json params = {
        { "film_type",    in.film_type },
        { "oven_temp_c",  in.oven_temp_c },
        { "dwell_time_s", in.dwell_time_s },
    };

    json pattern = {
        { "kind",             kSkillId },
        { "film_type",        in.film_type },
        { "oven_temp_c",      in.oven_temp_c },
        { "dwell_time_s",     in.dwell_time_s },
        { "process_family",   "shrink_wrap" },
        { "geometry_changed", false },
        { "is_process_only",  true },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code",     f.code },
                            { "severity", f.severity },
                            { "message",  f.message }});

    ToolingMeta tooling;
    tooling.tool_type         = "shrink_tunnel";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "stainless_oven";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    tooling.est_cycle_time_s  = in.dwell_time_s + 2.0;
    tooling.extra = {
        { "process",          "shrink_wrap" },
        { "film_type",        in.film_type },
        { "oven_temp_c",      in.oven_temp_c },
        { "dwell_time_s",     in.dwell_time_s },
        { "process_category", "packaging" },
        { "dfm_findings",     findings },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug("skill::shrink_wrap applied: film={} T={}°C dwell={}s",
                  in.film_type, in.oven_temp_c, in.dwell_time_s);

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

}  // namespace koocadcam::skill::shrink_wrap
