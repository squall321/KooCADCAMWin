// @lat: [[engine/skills#harvest_crop]]

#include "harvest_crop.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <memory>
#include <string>

namespace koocadcam::skill::harvest_crop {

using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;
    (void)wp;

    if (in.crop_type.empty()) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": crop_type must be non-empty");
    }

    if (in.yield_kg_ha <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": yield_kg_ha must be > 0 (got " +
              std::to_string(in.yield_kg_ha) + ")");
    } else if (in.yield_kg_ha > 15000.0) {
        r.add("DFM-HARVEST-YIELD", "info",
              std::string(kSkillId) + ": yield_kg_ha " +
              std::to_string(in.yield_kg_ha) +
              " exceeds 15000 — extreme yield, verify assumption");
    }

    if (in.area_ha <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": area_ha must be > 0 (got " +
              std::to_string(in.area_ha) + ")");
    }

    if (in.harvest_method != "combine" &&
        in.harvest_method != "manual"  &&
        in.harvest_method != "mechanical") {
        r.add("DFM-HARVEST-METHOD", "info",
              std::string(kSkillId) + ": harvest_method '" + in.harvest_method +
              "' not in {combine, manual, mechanical} — treated as 'mechanical'");
    }

    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "harvest_crop DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const std::string method =
        (in.harvest_method == "combine"  ||
         in.harvest_method == "manual"   ||
         in.harvest_method == "mechanical")
            ? in.harvest_method : "mechanical";

    const double totalYieldKg = in.yield_kg_ha * in.area_ha;

    json params = {
        { "crop_type",      in.crop_type },
        { "harvest_method", method },
        { "yield_kg_ha",    in.yield_kg_ha },
        { "area_ha",        in.area_ha },
    };
    json pattern = {
        { "kind",             kSkillId },
        { "crop_type",        in.crop_type },
        { "harvest_method",   method },
        { "yield_kg_ha",      in.yield_kg_ha },
        { "area_ha",          in.area_ha },
        { "total_yield_kg",   totalYieldKg },
        { "geometry_changed", false },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code", f.code },
                            { "severity", f.severity },
                            { "message", f.message }});

    ToolingMeta tooling;
    tooling.tool_type         = (method == "manual") ? "manual_harvest"
                                                     : (method == "combine"
                                                            ? "combine_harvester"
                                                            : "mechanical_harvester");
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "n/a";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    // Manual ~ 40 h/ha; mechanical ~ 1 h/ha; combine ~ 0.5 h/ha.
    {
        double hPerHa = (method == "manual")  ? 40.0
                      : (method == "combine") ? 0.5
                                              : 1.0;
        tooling.est_cycle_time_s = hPerHa * in.area_ha * 3600.0;
    }
    tooling.extra = {
        { "process",          "crop_harvest" },
        { "crop_type",        in.crop_type },
        { "harvest_method",   method },
        { "total_yield_kg",   totalYieldKg },
        { "process_category", "agriculture" },
        { "geometric_no_op",  true },
        { "dfm_findings",     findings },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug(
        "skill::harvest_crop applied: crop={} method={} yield={}kg/ha area={}ha",
        in.crop_type, method, in.yield_kg_ha, in.area_ha);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// Metadata-only: leaves no geometric trace.

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

}  // namespace koocadcam::skill::harvest_crop
