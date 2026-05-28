// @lat: [[engine/skills#multi_material_3d_print]]

#include "multi_material_3d_print.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <string>
#include <unordered_set>
#include <utility>

namespace koocadcam::skill::mmp3d_print {

using nlohmann::json;

namespace {

const std::unordered_set<std::string>& knownProcesses()
{
    static const std::unordered_set<std::string> s = {
        "fdm", "sla", "sls", "metal_laser", "polyjet",
    };
    return s;
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    // Process check (lighter than plain 3d_print — polyjet is allowed here).
    if (knownProcesses().count(in.process_type) == 0) {
        r.add("DFM-INPUT", "error",
              "multi_material_3d_print unknown process_type '" +
              in.process_type +
              "' (expected fdm | sla | sls | metal_laser | polyjet)");
    }

    // Layer-height check.
    if (in.layer_height_mm < 0.05 || in.layer_height_mm > 0.5) {
        r.add("DFM-MMP3D-LAYER", "error",
              "multi_material_3d_print layer_height_mm " +
              std::to_string(in.layer_height_mm) +
              " outside [0.05, 0.5] mm range");
    }

    // Region map check.
    const size_t n = in.region_material_map.size();
    if (n < 2) {
        r.add("DFM-MMP3D-REGIONS", "error",
              "multi_material_3d_print requires ≥ 2 regions (got " +
              std::to_string(n) +
              ") — single-region case should use plain 3d_print");
    }

    double pctSum = 0.0;
    bool   anyBadMat = false;
    for (const auto& rg : in.region_material_map) {
        if (rg.material.empty()) anyBadMat = true;
        if (rg.layer_pct > 0.0)  pctSum += rg.layer_pct;
    }
    if (anyBadMat) {
        r.add("DFM-MMP3D-MATERIAL", "error",
              "multi_material_3d_print: every region must have a non-empty "
              "material id");
    }

    // Percentage sum check: > 100 is an error, < 50 is an info-level warning
    // (region map may be intentionally partial when modeling only the
    // critical regions).
    if (pctSum > 100.001) {
        r.add("DFM-MMP3D-PCTSUM", "error",
              "multi_material_3d_print: layer_pct total " +
              std::to_string(pctSum) + " > 100 — over-specified region map");
    } else if (pctSum > 0.0 && pctSum < 50.0) {
        r.add("DFM-MMP3D-PCTSUM", "info",
              "multi_material_3d_print: layer_pct total " +
              std::to_string(pctSum) + " < 50 — region map may be incomplete");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "multi_material_3d_print DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    double pctSum = 0.0;
    json   regionsJ = json::array();
    for (const auto& rg : in.region_material_map) {
        regionsJ.push_back({
            { "region_id", rg.region_id },
            { "material",  rg.material },
            { "layer_pct", rg.layer_pct },
        });
        if (rg.layer_pct > 0.0) pctSum += rg.layer_pct;
    }

    json params = {
        { "process_type",         in.process_type },
        { "layer_height_mm",      in.layer_height_mm },
        { "region_material_map",  regionsJ },
    };

    json pattern = {
        { "kind",                  kSkillId },
        { "process_type",          in.process_type },
        { "layer_height_mm",       in.layer_height_mm },
        { "region_count",          static_cast<int>(in.region_material_map.size()) },
        { "region_material_map",   regionsJ },
        { "region_pct_sum",        pctSum },
        { "geometric_change",      false },
        { "process_family",        "additive" },
        { "additive_workflow",     true },
        { "is_multi_material",     true },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code",     f.code },
                            { "severity", f.severity },
                            { "message",  f.message }});

    ToolingMeta tooling;
    tooling.tool_type         = (in.process_type == "polyjet")
                                  ? "polyjet_head"
                                  : "multi_extruder";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "n/a";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    tooling.est_cycle_time_s  = 0.0;
    tooling.extra = {
        { "process",               in.process_type },
        { "process_family",        "additive" },
        { "layer_height_mm",       in.layer_height_mm },
        { "region_count",          static_cast<int>(in.region_material_map.size()) },
        { "region_material_map",   regionsJ },
        { "dfm_findings",          findings },
        { "machining_constraint",
          "additive multi-material — per-region tool change per layer" },
    };

    FeatureSignature sig{ kSkillId, std::move(params), std::move(pattern),
                          std::move(tooling) };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug("skill::multi_material_3d_print applied: process={} layer={} regions={}",
                  in.process_type, in.layer_height_mm,
                  in.region_material_map.size());

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

}  // namespace koocadcam::skill::mmp3d_print
