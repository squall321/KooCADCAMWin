// @lat: [[engine/skills#penetrant_weld]]

#include "penetrant_weld.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <string>

namespace koocadcam::skill::penetrant_weld {

using nlohmann::json;

namespace {

bool isAllowedSensitivity(const std::string& s)
{
    return s == "level_1" || s == "level_2" || s == "level_3" ||
           s == "level_4" || s == "level_5";
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.weld_id.empty()) {
        r.add("DFM-INPUT", "error",
              "penetrant_weld weld_id must be non-empty");
    }
    if (!isAllowedSensitivity(in.sensitivity_class)) {
        r.add("DFM-NDT-SENS", "error",
              "penetrant_weld sensitivity_class '" + in.sensitivity_class +
              "' not in {level_1..level_5}");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "penetrant_weld DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    json params = {
        { "weld_id",           in.weld_id },
        { "sensitivity_class", in.sensitivity_class },
        { "accept",            in.accept },
    };
    json pattern = {
        { "kind",               kSkillId },
        { "weld_id",            in.weld_id },
        { "sensitivity_class",  in.sensitivity_class },
        { "accept",             in.accept },
        { "code_reference",     "AWS_D1" },
        { "geometry_changed",   false },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "dye_penetrant_kit";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "fluorescent_dye";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    tooling.est_cycle_time_s  = 900.0;  // ~15 min: clean+dwell+develop
    tooling.extra = {
        { "process",            "ndt_liquid_penetrant" },
        { "weld_id",            in.weld_id },
        { "sensitivity_class",  in.sensitivity_class },
        { "accept",             in.accept },
        { "code",               "AWS_D1" },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::penetrant_weld applied: weld={} sens={} accept={}",
                  in.weld_id, in.sensitivity_class, in.accept);

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

}  // namespace koocadcam::skill::penetrant_weld
