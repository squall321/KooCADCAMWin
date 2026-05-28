// @lat: [[engine/skills#progressive_die_stage]]

#include "progressive_die_stage.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

namespace koocadcam::skill::progressive_die_stage {

using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.stage_index < 1) {
        r.add("DFM-INPUT", "error",
              "progressive_die_stage: stage_index must be >= 1 (got " +
              std::to_string(in.stage_index) + ")");
    }

    if (in.material_strip_pitch_mm < 0.0) {
        r.add("DFM-INPUT", "error",
              "progressive_die_stage: material_strip_pitch_mm must be >= 0");
    }

    if (in.material_strip_pitch_mm > 200.0) {
        r.add("DFM-PDS-PITCH", "info",
              "progressive_die_stage: material_strip_pitch_mm " +
              std::to_string(in.material_strip_pitch_mm) +
              " > 200 mm — unusually large stamping pitch");
    } else if (in.material_strip_pitch_mm > 0.0 && in.material_strip_pitch_mm < 5.0) {
        r.add("DFM-PDS-PITCH", "info",
              "progressive_die_stage: material_strip_pitch_mm " +
              std::to_string(in.material_strip_pitch_mm) +
              " < 5 mm — unusually small stamping pitch");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────
//
// Geometry: NO-OP.  Clone the workpiece (shape + material) carrying its
// prior features and stamp the stage signature.

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "progressive_die_stage DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    json params = {
        { "stage_index",             in.stage_index },
        { "station_position",        in.station_position },
        { "material_strip_pitch_mm", in.material_strip_pitch_mm },
    };
    json pattern = {
        { "kind",                    kSkillId },
        { "stage_index",             in.stage_index },
        { "station_position",        in.station_position },
        { "material_strip_pitch_mm", in.material_strip_pitch_mm },
        { "geometry_changed",        false },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code", f.code },
                            { "severity", f.severity },
                            { "message", f.message }});

    ToolingMeta tooling;
    tooling.tool_type         = "progressive_die_station";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "tool_steel";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    tooling.est_cycle_time_s  = 0.0;        // marker only
    tooling.extra = {
        { "process_category",       "stamping_layout" },
        { "geometric_no_op",        true },
        { "stage_index",            in.stage_index },
        { "station_position",       in.station_position },
        { "material_strip_pitch_mm", in.material_strip_pitch_mm },
        { "dfm_findings",           findings },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug("skill::progressive_die_stage applied: stage={} station={} pitch={}",
                  in.stage_index, in.station_position, in.material_strip_pitch_mm);

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

}  // namespace koocadcam::skill::progressive_die_stage
