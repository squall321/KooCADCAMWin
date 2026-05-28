// @lat: [[engine/skills#3d_print]]

#include "3d_print.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <string>
#include <unordered_map>
#include <unordered_set>

namespace koocadcam::skill::p3d_print {

using nlohmann::json;

namespace {

// Supported AM processes.
const std::unordered_set<std::string>& knownProcesses()
{
    static const std::unordered_set<std::string> s = {
        "fdm", "sla", "sls", "metal_laser",
    };
    return s;
}

// Material → process compatibility map (info-only DFM).
// Each entry lists processes that commonly print the material.
const std::unordered_map<std::string, std::unordered_set<std::string>>&
materialProcessCompat()
{
    static const std::unordered_map<std::string, std::unordered_set<std::string>> m = {
        { "pla",            { "fdm" } },
        { "abs",            { "fdm" } },
        { "petg",           { "fdm" } },
        { "tpu",            { "fdm" } },
        { "nylon",          { "fdm", "sls" } },
        { "polycarbonate",  { "fdm" } },
        { "resin",          { "sla" } },
        { "uv_resin",       { "sla" } },
        { "tough_resin",    { "sla" } },
        { "nylon_pa12",     { "sls" } },
        { "tpu_powder",     { "sls" } },
        { "stainless_316l", { "metal_laser" } },
        { "titanium_64",    { "metal_laser" } },
        { "aluminum_alsi10mg", { "metal_laser" } },
        { "inconel_718",    { "metal_laser" } },
        { "cobalt_chrome",  { "metal_laser" } },
    };
    return m;
}

// FDM is the only process that uses traditional infill.  SLA/SLS/metal_laser
// produce dense parts by default.
bool processSupportsInfill(const std::string& processType)
{
    return processType == "fdm";
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    // Process check.
    if (knownProcesses().count(in.process_type) == 0) {
        r.add("DFM-INPUT", "error",
              "3d_print unknown process_type '" + in.process_type +
              "' (expected fdm | sla | sls | metal_laser)");
    }

    // Layer-height check.
    if (in.layer_height_mm < 0.05 || in.layer_height_mm > 0.5) {
        r.add("DFM-PRINT-LAYER", "error",
              "3d_print layer_height_mm " +
              std::to_string(in.layer_height_mm) +
              " outside [0.05, 0.5] mm range");
    }

    // Infill check — only enforced when the process supports it.
    if (processSupportsInfill(in.process_type)) {
        if (in.infill_pct < 10.0 || in.infill_pct > 100.0) {
            r.add("DFM-PRINT-INFILL", "error",
                  "3d_print (FDM) infill_pct " +
                  std::to_string(in.infill_pct) +
                  " outside [10, 100] %% range");
        }
    } else {
        if (in.infill_pct > 0.0) {
            r.add("DFM-PRINT-INFILL", "info",
                  "3d_print infill_pct ignored for non-FDM process '" +
                  in.process_type + "'");
        }
    }

    if (in.material.empty()) {
        r.add("DFM-INPUT", "error", "3d_print material must be specified");
    } else {
        const auto& compat = materialProcessCompat();
        auto it = compat.find(in.material);
        if (it == compat.end()) {
            r.add("DFM-PRINT-COMPAT", "info",
                  "3d_print material '" + in.material +
                  "' not in compatibility catalog — verify with vendor");
        } else if (it->second.count(in.process_type) == 0) {
            r.add("DFM-PRINT-COMPAT", "info",
                  "3d_print material '" + in.material +
                  "' is not typically paired with process '" + in.process_type +
                  "' — verify material/process selection");
        }
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────
//
// NO-OP geometric.  The workpiece shape is treated as the FINAL printed
// part; we attach the process intent and slicer-relevant parameters to the
// FeatureSignature for downstream consumers.

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "3d_print DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const bool usesInfill = processSupportsInfill(in.process_type);
    const double effectiveInfill = usesInfill ? in.infill_pct : 0.0;

    json params = {
        { "process_type",     in.process_type },
        { "layer_height_mm",  in.layer_height_mm },
        { "material",         in.material },
        { "infill_pct",       effectiveInfill },
    };

    json pattern = {
        { "kind",                       kSkillId },
        { "is_process_only",            true },
        { "geometric_change",           false },
        { "process_family",             "additive" },
        { "process_subtype",            in.process_type },
        { "layer_height_mm",            in.layer_height_mm },
        { "material",                   in.material },
        { "infill_pct",                 effectiveInfill },
        { "process_supports_infill",    usesInfill },
        { "additive_workflow",          true },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code",     f.code },
                            { "severity", f.severity },
                            { "message",  f.message }});

    ToolingMeta tooling;
    // The "tool" of a 3D printer is the print head / laser / projector.
    if (in.process_type == "fdm")              tooling.tool_type = "fdm_extruder";
    else if (in.process_type == "sla")         tooling.tool_type = "sla_uv_projector";
    else if (in.process_type == "sls")         tooling.tool_type = "sls_laser";
    else if (in.process_type == "metal_laser") tooling.tool_type = "metal_laser_fusion";
    else                                        tooling.tool_type = "additive_head";
    tooling.tool_dia_mm       = 0.0;   // unknown without nozzle/beam spec
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "n/a";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    // Print cycle time depends on layer count × area — we record 0 here
    // and let the slicer fill it in during process planning.
    tooling.est_cycle_time_s  = 0.0;
    tooling.extra = {
        { "process",            in.process_type },
        { "process_family",     "additive" },
        { "layer_height_mm",    in.layer_height_mm },
        { "material",           in.material },
        { "infill_pct",         effectiveInfill },
        { "dfm_findings",       findings },
        { "machining_constraint",
          "additive — final shape produced by layer deposition, no removal" },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug("skill::3d_print applied: process={} layer={} material={} infill={}%%",
                  in.process_type, in.layer_height_mm, in.material, effectiveInfill);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// Metadata-only.  An ideal B-Rep shape gives no signal that says "I was
// printed."  Slice-stepping artifacts are not represented at the analytic
// modeling layer.

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

}  // namespace koocadcam::skill::p3d_print
