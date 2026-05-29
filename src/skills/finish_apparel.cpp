// @lat: [[engine/skills#finish_apparel]]

#include "finish_apparel.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

namespace koocadcam::skill::finish_apparel {

using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.technique != "washing" && in.technique != "pressing" &&
        in.technique != "labeling") {
        r.add("DFM-INPUT", "error",
              "finish_apparel technique '" + in.technique +
              "' not in {washing, pressing, labeling}");
    }

    if (in.technique == "washing") {
        if (in.bath_temp_c < 0.0 || in.bath_temp_c > 95.0) {
            r.add("DFM-FIN-TEMP", "warning",
                  "finish_apparel bath_temp_c " +
                  std::to_string(in.bath_temp_c) +
                  " outside [0, 95] — typical commercial wash range");
        }
    } else if (in.bath_temp_c != 0.0) {
        r.add("DFM-FIN-TEMP", "info",
              "finish_apparel bath_temp_c=" + std::to_string(in.bath_temp_c) +
              " ignored for technique '" + in.technique + "'");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "finish_apparel DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const double tempC = (in.technique == "washing") ? in.bath_temp_c : 0.0;

    json params = {
        { "technique",   in.technique },
        { "bath_temp_c", tempC },
    };
    json pattern = {
        { "kind",             kSkillId },
        { "technique",        in.technique },
        { "bath_temp_c",      tempC },
        { "geometry_changed", false },
    };

    ToolingMeta tooling;
    tooling.tool_type =
        (in.technique == "washing")  ? "industrial_washer" :
        (in.technique == "pressing") ? "steam_press" :
                                       "label_applicator";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "n/a";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    // Wash cycle ~45 min; press ~30 s; labeling ~5 s.
    tooling.est_cycle_time_s  =
        (in.technique == "washing")  ? 2700.0 :
        (in.technique == "pressing") ?   30.0 :
                                          5.0;
    tooling.extra = {
        { "process",     "apparel_finish" },
        { "technique",   in.technique },
        { "bath_temp_c", tempC },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::finish_apparel applied: tech={} temp={}°C",
                  in.technique, tempC);

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

}  // namespace koocadcam::skill::finish_apparel
