// @lat: [[engine/skills#friction_stir_weld_aero]]

#include "friction_stir_weld_aero.hpp"

#include "Workpiece.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <string>

namespace koocadcam::skill::friction_stir_weld_aero {

using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.tool_dia_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "friction_stir_weld_aero tool_dia_mm must be > 0");
    } else {
        if (in.tool_dia_mm < 8.0) {
            r.add("DFM-FSW-TOOL", "error",
                  "friction_stir_weld_aero tool_dia_mm " +
                  std::to_string(in.tool_dia_mm) +
                  " < 8 mm — pin/shoulder insufficient for aerospace skin");
        }
        if (in.tool_dia_mm > 25.0) {
            r.add("DFM-FSW-TOOL", "error",
                  "friction_stir_weld_aero tool_dia_mm " +
                  std::to_string(in.tool_dia_mm) +
                  " > 25 mm — exceeds typical aerospace FSW shoulder envelope");
        }
    }

    if (in.traverse_speed_mm_min <= 0.0) {
        r.add("DFM-INPUT", "error",
              "friction_stir_weld_aero traverse_speed_mm_min must be > 0");
    } else {
        if (in.traverse_speed_mm_min < 50.0) {
            r.add("DFM-FSW-SPEED", "error",
                  "friction_stir_weld_aero traverse_speed_mm_min " +
                  std::to_string(in.traverse_speed_mm_min) +
                  " < 50 — heat-input excessive (HAZ softening)");
        }
        if (in.traverse_speed_mm_min > 1500.0) {
            r.add("DFM-FSW-SPEED", "error",
                  "friction_stir_weld_aero traverse_speed_mm_min " +
                  std::to_string(in.traverse_speed_mm_min) +
                  " > 1500 — cold-weld defect (insufficient plasticisation)");
        }
    }

    if (in.seam_length_mm < 50.0) {
        r.add("DFM-FSW-LEN", "error",
              "friction_stir_weld_aero seam_length_mm " +
              std::to_string(in.seam_length_mm) +
              " < 50 mm — short seams better served by tig_weld/laser");
    }

    if (in.alloy_pair.empty()) {
        r.add("DFM-INPUT", "warning",
              "friction_stir_weld_aero alloy_pair empty — recipe optimisation skipped");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "friction_stir_weld_aero DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    auto entryId = wp.resolve(in.entry_face);
    if (!entryId)
        throw SkillError("friction_stir_weld_aero: entry_face datum unresolved");

    const std::string alloyPair = in.alloy_pair.empty() ? "unspecified"
                                                        : in.alloy_pair;

    GProp_GProps props;
    BRepGProp::SurfaceProperties(wp.face(*entryId), props);
    const double faceArea = props.Mass();

    json params = {
        { "entry_face_id",         *entryId },
        { "tool_dia_mm",           in.tool_dia_mm },
        { "traverse_speed_mm_min", in.traverse_speed_mm_min },
        { "seam_length_mm",        in.seam_length_mm },
        { "alloy_pair",            alloyPair },
    };
    json pattern = {
        { "kind",                  kSkillId },
        { "target_face_id",        *entryId },
        { "tool_dia_mm",           in.tool_dia_mm },
        { "traverse_speed_mm_min", in.traverse_speed_mm_min },
        { "seam_length_mm",        in.seam_length_mm },
        { "alloy_pair",            alloyPair },
        { "target_face_area_mm2",  faceArea },
        { "joint_type",            "butt_or_lap_aero" },
        { "geometry_changed",      false },
    };
    ToolingMeta tooling;
    tooling.tool_type         = "fsw_shoulder_pin";
    tooling.tool_dia_mm       = in.tool_dia_mm;
    tooling.tool_length_mm    = in.tool_dia_mm * 0.5;   // pin ~ shoulder/2
    tooling.tool_material     = "tool_steel_h13";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;                    // solid-state — no removal
    // Cycle time = seam_length / traverse_speed (mm / (mm/min)) → minutes.
    const double traverse_min = (in.traverse_speed_mm_min > 0.0)
        ? in.seam_length_mm / in.traverse_speed_mm_min
        : 0.0;
    tooling.est_cycle_time_s  = std::max(5.0, traverse_min * 60.0);
    tooling.extra = {
        { "process",                "friction_stir_weld_aerospace" },
        { "tool_dia_mm",            in.tool_dia_mm },
        { "traverse_speed_mm_min",  in.traverse_speed_mm_min },
        { "seam_length_mm",         in.seam_length_mm },
        { "alloy_pair",             alloyPair },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    // No geometric change — clone verbatim.
    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::friction_stir_weld_aero applied: face={} tool={} v={} L={} pair={}",
                  *entryId, in.tool_dia_mm, in.traverse_speed_mm_min,
                  in.seam_length_mm, alloyPair);

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

}  // namespace koocadcam::skill::friction_stir_weld_aero
