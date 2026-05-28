// @lat: [[engine/skills#fused_dep_model]]

#include "fused_dep_model.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <string>

namespace koocadcam::skill::fused_dep_model {

using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    // Nozzle diameter band.
    if (in.nozzle_dia_mm < 0.2 || in.nozzle_dia_mm > 1.0) {
        r.add("DFM-FDM-NOZ", "error",
              "fused_dep_model nozzle_dia_mm " +
              std::to_string(in.nozzle_dia_mm) +
              " outside [0.2, 1.0] mm range");
    }

    // Layer height — must be ≤ nozzle × 0.8 for stable inter-layer bond.
    if (in.layer_height_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "fused_dep_model layer_height_mm must be > 0 (got " +
              std::to_string(in.layer_height_mm) + ")");
    } else if (in.nozzle_dia_mm > 0.0 &&
               in.layer_height_mm > in.nozzle_dia_mm * 0.8 + 1e-9) {
        r.add("DFM-FDM-LAYER", "error",
              "fused_dep_model layer_height " +
              std::to_string(in.layer_height_mm) +
              " > 0.8 × nozzle_dia " + std::to_string(in.nozzle_dia_mm) +
              " — inter-layer bond will be weak, part will delaminate");
    }

    if (in.print_speed_mm_per_s <= 0.0) {
        r.add("DFM-FDM-SPEED", "error",
              "fused_dep_model print_speed_mm_per_s must be > 0 (got " +
              std::to_string(in.print_speed_mm_per_s) + ")");
    } else if (in.print_speed_mm_per_s > 300.0) {
        r.add("DFM-FDM-SPEED", "warning",
              "fused_dep_model print_speed_mm_per_s " +
              std::to_string(in.print_speed_mm_per_s) +
              " > 300 — unusually fast, verify printer kinematics");
    }

    if (in.material.empty()) {
        r.add("DFM-INPUT", "error",
              "fused_dep_model material must be specified");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────
//
// NO-OP geometric: the workpiece IS the printed part.  We record the
// nozzle / layer / speed parameters into the FeatureSignature so the
// downstream slicer/process planner has everything it needs to emit a
// G-code program.

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "fused_dep_model DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    // Resolve the entry face (build-plate reference) if specified.  This is
    // not strictly required by the process, but if it resolves we record
    // its id so a slicer knows which face was nominated as layer 1.
    auto entryId = wp.resolve(in.entry_face);

    json params = {
        { "nozzle_dia_mm",         in.nozzle_dia_mm },
        { "layer_height_mm",       in.layer_height_mm },
        { "material",              in.material },
        { "print_speed_mm_per_s",  in.print_speed_mm_per_s },
    };
    if (entryId.has_value()) {
        params["entry_face_id"] = *entryId;
    } else {
        params["entry_face_id"] = nullptr;
    }

    // Compute the layer/nozzle ratio for downstream inspection.
    const double layerToNozzle = (in.nozzle_dia_mm > 0.0)
        ? in.layer_height_mm / in.nozzle_dia_mm
        : 0.0;

    json pattern = {
        { "kind",                       kSkillId },
        { "is_process_only",            true },
        { "geometric_change",           false },
        { "process_family",             "additive" },
        { "process_subtype",            "fdm" },
        { "process_alias",              "fused_deposition_modeling" },
        { "nozzle_dia_mm",              in.nozzle_dia_mm },
        { "layer_height_mm",            in.layer_height_mm },
        { "layer_to_nozzle_ratio",      layerToNozzle },
        { "material",                   in.material },
        { "print_speed_mm_per_s",       in.print_speed_mm_per_s },
        { "additive_workflow",          true },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code",     f.code },
                            { "severity", f.severity },
                            { "message",  f.message }});

    ToolingMeta tooling;
    tooling.tool_type         = "fdm_nozzle";
    tooling.tool_dia_mm       = in.nozzle_dia_mm;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "brass_steel_hardened";  // typical FDM hot-end nozzle
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    // Print cycle time depends on part volume + travel — leave 0 for the
    // slicer to fill in.
    tooling.est_cycle_time_s  = 0.0;
    tooling.extra = {
        { "process",                "fused_dep_model" },
        { "process_family",         "additive" },
        { "nozzle_dia_mm",          in.nozzle_dia_mm },
        { "layer_height_mm",        in.layer_height_mm },
        { "layer_to_nozzle_ratio",  layerToNozzle },
        { "material",               in.material },
        { "print_speed_mm_per_s",   in.print_speed_mm_per_s },
        { "dfm_findings",           findings },
        { "machining_constraint",
          "additive — bead-by-bead deposition, no material removal" },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug("skill::fused_dep_model applied: nozzle={} layer={} mat={} speed={}",
                  in.nozzle_dia_mm, in.layer_height_mm, in.material,
                  in.print_speed_mm_per_s);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// Metadata-only.

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

}  // namespace koocadcam::skill::fused_dep_model
