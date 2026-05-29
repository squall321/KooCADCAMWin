// @lat: [[engine/skills#filter_change]]

#include "filter_change.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <string>

namespace koocadcam::skill::filter_change {

using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.filter_id.empty()) {
        r.add("DFM-INPUT", "error",
              "filter_change filter_id must be non-empty");
    }

    if (in.filter_type != "oil" && in.filter_type != "air" &&
        in.filter_type != "fuel" && in.filter_type != "hydraulic") {
        r.add("DFM-FILTER-TYPE", "info",
              "filter_change filter_type '" + in.filter_type +
              "' not in {oil, air, fuel, hydraulic}");
    }

    if (in.hours_in_service < 0.0) {
        r.add("DFM-INPUT", "error",
              "filter_change hours_in_service must be ≥ 0 (got " +
              std::to_string(in.hours_in_service) + ")");
    } else if (in.hours_in_service > 8760.0) {
        r.add("DFM-FILTER-LIFE", "info",
              "filter_change hours_in_service " +
              std::to_string(in.hours_in_service) +
              " > 8760 h (≈ 1 year) — likely overdue");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "filter_change DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const std::string canonicalType =
        (in.filter_type == "oil" || in.filter_type == "air" ||
         in.filter_type == "fuel" || in.filter_type == "hydraulic")
            ? in.filter_type : "generic";

    json params = {
        { "filter_type",      in.filter_type },
        { "filter_id",        in.filter_id },
        { "hours_in_service", in.hours_in_service },
    };
    json pattern = {
        { "kind",              kSkillId },
        { "filter_type",       canonicalType },
        { "filter_id",         in.filter_id },
        { "hours_in_service",  in.hours_in_service },
        { "geometry_changed",  false },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code", f.code },
                            { "severity", f.severity },
                            { "message", f.message }});

    ToolingMeta tooling;
    tooling.tool_type         = (canonicalType == "oil" || canonicalType == "fuel")
                                  ? "filter_wrench"
                                  : "filter_housing_tool";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "steel";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    tooling.est_cycle_time_s  = 600.0;     // ~10 min cartridge swap
    tooling.extra = {
        { "process",          "filter_swap" },
        { "filter_type",      canonicalType },
        { "filter_id",        in.filter_id },
        { "hours_in_service", in.hours_in_service },
        { "dfm_findings",     findings },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::filter_change applied: type={} id={} hours={}",
                  canonicalType, in.filter_id, in.hours_in_service);

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

}  // namespace koocadcam::skill::filter_change
