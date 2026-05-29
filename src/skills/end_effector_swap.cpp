// @lat: [[engine/skills#end_effector_swap]]

#include "end_effector_swap.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <cmath>
#include <string>

namespace koocadcam::skill::end_effector_swap {

using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.old_effector_id.empty()) {
        r.add("DFM-INPUT", "error",
              "end_effector_swap old_effector_id must be non-empty");
    }
    if (in.new_effector_id.empty()) {
        r.add("DFM-INPUT", "error",
              "end_effector_swap new_effector_id must be non-empty");
    }

    if (!in.old_effector_id.empty() && !in.new_effector_id.empty() &&
        in.old_effector_id == in.new_effector_id) {
        r.add("DFM-EOAT-SAME", "info",
              "end_effector_swap: old_effector_id == new_effector_id '" +
              in.old_effector_id + "' — no-op swap, only recalibration");
    }

    if (std::fabs(in.calibration_offset_mm) > 10.0) {
        r.add("DFM-EOAT-OFFSET", "info",
              "end_effector_swap calibration_offset_mm " +
              std::to_string(in.calibration_offset_mm) +
              " exceeds ±10 mm — investigate mounting drift before production");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "end_effector_swap DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    json params = {
        { "old_effector_id",       in.old_effector_id },
        { "new_effector_id",       in.new_effector_id },
        { "calibration_offset_mm", in.calibration_offset_mm },
    };
    json pattern = {
        { "kind",                  kSkillId },
        { "old_effector_id",       in.old_effector_id },
        { "new_effector_id",       in.new_effector_id },
        { "calibration_offset_mm", in.calibration_offset_mm },
        { "geometry_changed",      false },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "tool_changer";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "n/a";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    tooling.est_cycle_time_s  = 30.0;   // automated swap + recalibration
    tooling.extra = {
        { "process",               "end_effector_swap" },
        { "old_effector_id",       in.old_effector_id },
        { "new_effector_id",       in.new_effector_id },
        { "calibration_offset_mm", in.calibration_offset_mm },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::end_effector_swap applied: {} -> {} offset={}",
                  in.old_effector_id, in.new_effector_id, in.calibration_offset_mm);

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

}  // namespace koocadcam::skill::end_effector_swap
