// @lat: [[engine/skills#glued_joint]]

#include "glued_joint.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <cmath>

namespace koocadcam::skill::glued_joint {

using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    // Resolve to check bond face area, but only if resolution succeeds.
    auto bondId = wp.resolve(in.bond_face);
    if (bondId) {
        const double area = wp.faceArea(*bondId);
        if (area < 25.0) {
            r.add("DFM-GLUE-AREA", "error",
                  "glued_joint bond face area " + std::to_string(area) +
                  " mm² < 25 mm² minimum (joint too small to be reliable)");
        }
    } else {
        r.add("DFM-INPUT", "error",
              "glued_joint bond_face datum could not be resolved");
    }

    if (in.bead_width_mm < 0.1 || in.bead_width_mm > 5.0) {
        r.add("DFM-GLUE-WIDTH", "error",
              "glued_joint bead_width_mm " + std::to_string(in.bead_width_mm) +
              " outside [0.1, 5.0] mm");
    }
    if (in.adhesive_type.empty()) {
        r.add("DFM-GLUE-ADHESIVE", "error",
              "glued_joint adhesive_type must not be empty");
    }
    if (in.cure_time_min <= 0.0) {
        r.add("DFM-GLUE-CURE", "error",
              "glued_joint cure_time_min must be > 0");
    }
    if (in.shear_strength_MPa <= 0.0) {
        r.add("DFM-GLUE-STRENGTH", "warning",
              "glued_joint shear_strength_MPa is 0 — using adhesive default");
    }
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "glued_joint DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    auto bondId = wp.resolve(in.bond_face);
    if (!bondId) throw SkillError("glued_joint: bond_face datum unresolved");

    // Metadata-only — no geometric change.  Compute bond area from the face
    // we found so the planner knows adhesive bead length / coverage area.
    const double bondArea = wp.faceArea(*bondId);

    json params = {
        { "bond_face_id",        *bondId },
        { "partner_part",        in.partner_part },
        { "partner_face_name",   in.partner_face_name },
        { "adhesive_type",       in.adhesive_type },
        { "bead_width_mm",       in.bead_width_mm },
        { "cure_time_min",       in.cure_time_min },
        { "shear_strength_MPa",  in.shear_strength_MPa },
        { "bond_area_mm2",       bondArea },
    };
    json pattern = {
        { "kind",              kSkillId },
        { "geometric_change",  false },
        { "bond_face_id",      *bondId },
        { "bond_area_mm2",     bondArea },
        { "adhesive_type",     in.adhesive_type },
        { "additive",          false },
    };
    ToolingMeta tooling;
    tooling.tool_type         = "adhesive_dispenser";
    tooling.tool_dia_mm       = in.bead_width_mm;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "polymer_nozzle";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;     // adhesive volume is small; record below
    tooling.est_cycle_time_s  = in.cure_time_min * 60.0;
    // Rough adhesive volume estimate (bead_width × bead_width × √area)
    const double beadVol = in.bead_width_mm * in.bead_width_mm * std::sqrt(bondArea);
    tooling.extra = {
        { "process",             "adhesive_bond" },
        { "adhesive_type",       in.adhesive_type },
        { "bead_width_mm",       in.bead_width_mm },
        { "cure_time_min",       in.cure_time_min },
        { "shear_strength_MPa",  in.shear_strength_MPa },
        { "adhesive_volume_mm3", beadVol },
        { "partner_part",        in.partner_part },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    // Shape is UNCHANGED.  Clone workpiece carrying the same shape so
    // downstream callers always receive a fresh shared_ptr.
    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::glued_joint applied: face_id={} area={} adhesive={}",
                  *bondId, bondArea, in.adhesive_type);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// No B-rep evidence remains after applying a glued_joint, so recognition is
// purely metadata replay.

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;
    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        json rec = {
            { "bond_face_id",       f.params.value("bond_face_id",       -1) },
            { "partner_part",       f.params.value("partner_part",       std::string()) },
            { "partner_face_name",  f.params.value("partner_face_name",  std::string()) },
            { "adhesive_type",      f.params.value("adhesive_type",      std::string("epoxy")) },
            { "bead_width_mm",      f.params.value("bead_width_mm",      0.0) },
            { "cure_time_min",      f.params.value("cure_time_min",      0.0) },
            { "shear_strength_MPa", f.params.value("shear_strength_MPa", 0.0) },
        };
        out.push_back(RecognizedFeature{
            kSkillId, rec, 1.0, json{ { "source", "feature_history" } }
        });
    }
    return out;
}

}  // namespace koocadcam::skill::glued_joint
