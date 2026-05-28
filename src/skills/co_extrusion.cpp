// @lat: [[engine/skills#co_extrusion]]

#include "co_extrusion.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <string>
#include <utility>

namespace koocadcam::skill::co_extrusion {

using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    const size_t n = in.layers.size();
    if (n < 2) {
        r.add("DFM-COEX-COUNT", "error",
              "co_extrusion requires ≥ 2 layers (got " + std::to_string(n) +
              ") — single-layer profiles should use plain extrusion");
    } else if (n > 7) {
        r.add("DFM-COEX-COUNT", "error",
              "co_extrusion " + std::to_string(n) + " layers > 7 — "
              "die-flow balance becomes impractical");
    }

    double totalT = 0.0;
    bool   anyBadMat = false;
    bool   anyBadThk = false;
    for (size_t i = 0; i < in.layers.size(); ++i) {
        const auto& L = in.layers[i];
        if (L.material.empty()) {
            anyBadMat = true;
        }
        if (L.thickness_mm <= 0.0) {
            anyBadThk = true;
        } else {
            totalT += L.thickness_mm;
        }
    }
    if (anyBadMat) {
        r.add("DFM-COEX-MATERIAL", "error",
              "co_extrusion: every layer must have a non-empty material id");
    }
    if (anyBadThk) {
        r.add("DFM-COEX-THICKNESS", "error",
              "co_extrusion: every layer thickness must be > 0");
    }
    if (!anyBadThk && totalT <= 0.0) {
        r.add("DFM-COEX-THICKNESS", "error",
              "co_extrusion: total thickness must be > 0");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "co_extrusion DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    double totalT = 0.0;
    json layersJ  = json::array();
    for (const auto& L : in.layers) {
        layersJ.push_back({
            { "material",     L.material },
            { "thickness_mm", L.thickness_mm },
        });
        totalT += L.thickness_mm;
    }

    json params = {
        { "layers", layersJ },
    };

    json pattern = {
        { "kind",                kSkillId },
        { "layer_count",         static_cast<int>(in.layers.size()) },
        { "layers",              layersJ },
        { "total_thickness_mm",  totalT },
        { "geometric_change",    false },
        { "process_family",      "extrusion" },
        { "is_multilayer",       true },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code",     f.code },
                            { "severity", f.severity },
                            { "message",  f.message }});

    ToolingMeta tooling;
    tooling.tool_type         = "co_extrusion_die";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "tool_steel";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    tooling.est_cycle_time_s  = 0.0;
    tooling.extra = {
        { "process",            "co_extrusion" },
        { "process_family",     "extrusion" },
        { "layer_count",        static_cast<int>(in.layers.size()) },
        { "layers",             layersJ },
        { "total_thickness_mm", totalT },
        { "dfm_findings",       findings },
    };

    FeatureSignature sig{ kSkillId, std::move(params), std::move(pattern),
                          std::move(tooling) };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug("skill::co_extrusion applied: layers={} total={}mm",
                  in.layers.size(), totalT);

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

}  // namespace koocadcam::skill::co_extrusion
