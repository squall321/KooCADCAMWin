// @lat: [[engine/skills#structured_light_scan]]

#include "structured_light_scan.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <string>
#include <vector>

namespace koocadcam::skill::structured_light_scan {

using nlohmann::json;

namespace {

// Map projector pattern → captured frame count (drives cycle time).
int patternFrameCount(const std::string& p)
{
    if (p == "gray")        return 10;   // 10-bit Gray code
    if (p == "phase_shift") return 4;    // 4-step phase shift
    if (p == "hybrid")      return 14;   // Gray + phase
    return 0;
}

bool isKnownPattern(const std::string& p)
{
    return p == "gray" || p == "phase_shift" || p == "hybrid";
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;
    (void)wp;

    if (in.camera_count < 1) {
        r.add("DFM-INPUT", "error",
              "structured_light_scan camera_count must be >= 1 (got " +
              std::to_string(in.camera_count) + ")");
    } else if (in.camera_count == 1) {
        r.add("DFM-SL-CAMS", "info",
              "structured_light_scan camera_count == 1 — no stereo "
              "cross-check, accept-only mode");
    }

    if (in.accuracy_um <= 0.0) {
        r.add("DFM-INPUT", "error",
              "structured_light_scan accuracy_um must be > 0 (got " +
              std::to_string(in.accuracy_um) + ")");
    } else if (in.accuracy_um > 100.0) {
        r.add("DFM-SL-ACC", "warning",
              "structured_light_scan accuracy_um " +
              std::to_string(in.accuracy_um) +
              " > 100 µm — loose for structured light (typical ≤ 50 µm)");
    }

    if (!isKnownPattern(in.projector_pattern)) {
        r.add("DFM-SL-PATTERN", "info",
              "structured_light_scan projector_pattern '" + in.projector_pattern +
              "' not in known set (gray|phase_shift|hybrid)");
    }

    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "structured_light_scan DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const int frames = patternFrameCount(in.projector_pattern);

    json params = {
        { "projector_pattern", in.projector_pattern },
        { "camera_count",      in.camera_count },
        { "accuracy_um",       in.accuracy_um },
    };
    json pattern = {
        { "kind",                kSkillId },
        { "is_inspection_only",  true },
        { "geometric_change",    false },
        { "projector_pattern",   in.projector_pattern },
        { "camera_count",        in.camera_count },
        { "accuracy_um",         in.accuracy_um },
        { "frames_captured",     frames },
        { "stereo_enabled",      in.camera_count >= 2 },
        { "inspection_method",   "structured_light_triangulation" },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code", f.code },
                            { "severity", f.severity },
                            { "message", f.message }});

    ToolingMeta tooling;
    tooling.tool_type         = "structured_light_scanner";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "dlp_projector";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    // Cycle: setup (5 s) + frame capture (~ 0.5 s/frame) + reconstruction (5 s).
    const double captureS = std::max(1, frames) * 0.5;
    tooling.est_cycle_time_s  = 5.0 + captureS + 5.0;
    tooling.extra["dfm_findings"]         = findings;
    tooling.extra["machining_constraint"] =
        "inspection — structured-light scan, no material removal";
    tooling.extra["frames_captured"]      = frames;

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug("skill::structured_light_scan applied: pattern={} cams={} acc={} µm",
                  in.projector_pattern, in.camera_count, in.accuracy_um);

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

}  // namespace koocadcam::skill::structured_light_scan
