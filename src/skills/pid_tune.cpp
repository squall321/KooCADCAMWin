// @lat: [[engine/skills#pid_tune]]

#include "pid_tune.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

namespace koocadcam::skill::pid_tune {

using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.kp < 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": kp must be >= 0 (got " +
              std::to_string(in.kp) + ")");
    }
    if (in.ki < 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": ki must be >= 0 (got " +
              std::to_string(in.ki) + ")");
    }
    if (in.kd < 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": kd must be >= 0 (got " +
              std::to_string(in.kd) + ")");
    }
    if (in.target_response_ms <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": target_response_ms must be > 0 (got " +
              std::to_string(in.target_response_ms) + ")");
    }
    if (in.overshoot_pct < 0.0 || in.overshoot_pct > 50.0) {
        r.add("DFM-PID-OVER", "error",
              std::string(kSkillId) + ": overshoot_pct " +
              std::to_string(in.overshoot_pct) +
              " not in [0, 50] — out of normal control-loop band");
    }
    if (in.kp == 0.0 && r.passed) {
        r.add("DFM-PID-TUNE", "info",
              std::string(kSkillId) +
              ": kp == 0 — loop is effectively open, verify intent");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "pid_tune DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    json params = {
        { "kp",                 in.kp },
        { "ki",                 in.ki },
        { "kd",                 in.kd },
        { "target_response_ms", in.target_response_ms },
        { "overshoot_pct",      in.overshoot_pct },
    };
    json pattern = {
        { "kind",                 kSkillId },
        { "kp",                   in.kp },
        { "ki",                   in.ki },
        { "kd",                   in.kd },
        { "target_response_ms",   in.target_response_ms },
        { "overshoot_pct",        in.overshoot_pct },
        { "geometry_changed",     false },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "process_controller";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "n/a";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    // Controller commissioning ≈ target settle + 10x safety margin.
    tooling.est_cycle_time_s  = (in.target_response_ms / 1000.0) * 10.0;
    tooling.extra = {
        { "process",            "pid_loop_tune" },
        { "kp",                 in.kp },
        { "ki",                 in.ki },
        { "kd",                 in.kd },
        { "target_response_ms", in.target_response_ms },
        { "overshoot_pct",      in.overshoot_pct },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    // No geometric change — clone shape + material verbatim.
    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::pid_tune applied: kp={} ki={} kd={} resp={}ms over={}%",
                  in.kp, in.ki, in.kd, in.target_response_ms, in.overshoot_pct);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// Controller gains are sub-OCCT — recognition is metadata-only.

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

}  // namespace koocadcam::skill::pid_tune
