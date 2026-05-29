// @lat: [[engine/skills#cut_apparel]]

#include "cut_apparel.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

namespace koocadcam::skill::cut_apparel {

using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.pattern_id.empty()) {
        r.add("DFM-INPUT", "error",
              "cut_apparel pattern_id must be non-empty");
    }
    if (in.cut_layers < 1) {
        r.add("DFM-INPUT", "error",
              "cut_apparel cut_layers must be >= 1 (got " +
              std::to_string(in.cut_layers) + ")");
    }
    if (in.cut_layers > 250) {
        r.add("DFM-CUT-LAYERS", "warning",
              "cut_apparel cut_layers " + std::to_string(in.cut_layers) +
              " > 250 — may exceed cutter Z capacity");
    }
    if (in.cut_method != "manual" && in.cut_method != "laser" &&
        in.cut_method != "die") {
        r.add("DFM-CUT-METHOD", "info",
              "cut_apparel cut_method '" + in.cut_method +
              "' not in {manual, laser, die} — treated as manual");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "cut_apparel DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const std::string method = (in.cut_method == "manual" ||
                                in.cut_method == "laser" ||
                                in.cut_method == "die")
                                   ? in.cut_method : "manual";

    json params = {
        { "pattern_id",  in.pattern_id },
        { "cut_layers",  in.cut_layers },
        { "cut_method",  method },
    };
    json pattern = {
        { "kind",             kSkillId },
        { "pattern_id",       in.pattern_id },
        { "cut_layers",       in.cut_layers },
        { "cut_method",       method },
        { "geometry_changed", false },
    };

    ToolingMeta tooling;
    tooling.tool_type =
        (method == "laser") ? "co2_laser_cutter" :
        (method == "die")   ? "steel_rule_die_press" :
                              "manual_shears";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = (method == "die") ? "tool_steel" : "n/a";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    // Laser ~ 30 s/layer; die press ~ 5 s/cycle; manual ~ 90 s/layer.
    tooling.est_cycle_time_s  =
        (method == "laser") ? 30.0 * in.cut_layers :
        (method == "die")   ?  5.0 :
                              90.0 * in.cut_layers;
    tooling.extra = {
        { "process",    "apparel_cut" },
        { "pattern_id", in.pattern_id },
        { "cut_layers", in.cut_layers },
        { "cut_method", method },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::cut_apparel applied: pattern={} layers={} method={}",
                  in.pattern_id, in.cut_layers, method);

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

}  // namespace koocadcam::skill::cut_apparel
