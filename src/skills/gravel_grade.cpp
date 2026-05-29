// @lat: [[engine/skills#gravel_grade]]

#include "gravel_grade.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <string>

namespace koocadcam::skill::gravel_grade {

using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.throughput_kg_h <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": throughput_kg_h must be > 0 (got " +
              std::to_string(in.throughput_kg_h) + ")");
    }
    if (in.size_distribution_classes.empty()) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": size_distribution_classes must not be empty");
    }

    bool targetFound = std::find(in.size_distribution_classes.begin(),
                                 in.size_distribution_classes.end(),
                                 in.target_class)
                       != in.size_distribution_classes.end();
    if (!in.size_distribution_classes.empty() && !targetFound) {
        r.add("DFM-GRAVEL-TARGET", "warning",
              std::string(kSkillId) + ": target_class '" + in.target_class +
              "' not present in size_distribution_classes");
    }

    if (in.size_distribution_classes.size() > 6) {
        r.add("DFM-GRAVEL-CLASS-COUNT", "info",
              std::string(kSkillId) + ": class_count " +
              std::to_string(in.size_distribution_classes.size()) +
              " > 6 — multi-deck screen complexity rises sharply");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "gravel_grade DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    json classesJson = json::array();
    for (const auto& c : in.size_distribution_classes) classesJson.push_back(c);

    json params = {
        { "size_distribution_classes", classesJson },
        { "throughput_kg_h",           in.throughput_kg_h },
        { "target_class",              in.target_class },
    };
    json pattern = {
        { "kind",                       kSkillId },
        { "size_distribution_classes",  classesJson },
        { "throughput_kg_h",            in.throughput_kg_h },
        { "target_class",               in.target_class },
        { "class_count",                in.size_distribution_classes.size() },
        { "geometry_changed",           false },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "vibrating_screen";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "manganese_screen_mesh";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    tooling.est_cycle_time_s  =
        (in.throughput_kg_h > 0.0) ? 3600.0 / in.throughput_kg_h : 0.0;
    tooling.extra = {
        { "process",                    "aggregate_classification" },
        { "throughput_kg_h",            in.throughput_kg_h },
        { "target_class",               in.target_class },
        { "class_count",                in.size_distribution_classes.size() },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::gravel_grade applied: throughput={}kg/h target={} classes={}",
                  in.throughput_kg_h, in.target_class,
                  in.size_distribution_classes.size());

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

}  // namespace koocadcam::skill::gravel_grade
