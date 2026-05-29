// @lat: [[engine/skills#robot_calibrate]]

#include "robot_calibrate.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <string>

namespace koocadcam::skill::robot_calibrate {

using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.robot_id.empty()) {
        r.add("DFM-INPUT", "error",
              "robot_calibrate robot_id must be non-empty");
    }

    if (in.accuracy_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "robot_calibrate accuracy_mm must be > 0 (got " +
              std::to_string(in.accuracy_mm) + ")");
    }
    if (in.repeatability_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "robot_calibrate repeatability_mm must be > 0 (got " +
              std::to_string(in.repeatability_mm) + ")");
    }

    if (in.accuracy_mm > 0.0 && in.repeatability_mm > 0.0 &&
        in.repeatability_mm > in.accuracy_mm) {
        r.add("DFM-ROBOT-RANGE", "info",
              "robot_calibrate: repeatability_mm (" +
              std::to_string(in.repeatability_mm) +
              ") > accuracy_mm (" + std::to_string(in.accuracy_mm) +
              ") — unusual, expect repeatability ≤ accuracy");
    }

    if (in.accuracy_mm > 1.0) {
        r.add("DFM-ROBOT-ACCURACY", "info",
              "robot_calibrate accuracy_mm " + std::to_string(in.accuracy_mm) +
              " > 1.0 — coarse for precision assembly tasks");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "robot_calibrate DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    json params = {
        { "robot_id",         in.robot_id },
        { "accuracy_mm",      in.accuracy_mm },
        { "repeatability_mm", in.repeatability_mm },
    };
    json pattern = {
        { "kind",             kSkillId },
        { "robot_id",         in.robot_id },
        { "accuracy_mm",      in.accuracy_mm },
        { "repeatability_mm", in.repeatability_mm },
        { "geometry_changed", false },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "robot_calibration_kit";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "n/a";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    tooling.est_cycle_time_s  = 600.0;   // ~10 min typical calibration session
    tooling.extra = {
        { "process",          "robot_arm_calibration" },
        { "robot_id",         in.robot_id },
        { "accuracy_mm",      in.accuracy_mm },
        { "repeatability_mm", in.repeatability_mm },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::robot_calibrate applied: robot={} acc={} rep={}",
                  in.robot_id, in.accuracy_mm, in.repeatability_mm);

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

}  // namespace koocadcam::skill::robot_calibrate
