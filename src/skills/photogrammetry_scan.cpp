// @lat: [[engine/skills#photogrammetry_scan]]

#include "photogrammetry_scan.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <string>
#include <vector>

namespace koocadcam::skill::photogrammetry_scan {

using nlohmann::json;

namespace {

// Map accuracy class → target accuracy in mm.  Returns -1 for unknown.
double accuracyClassToMm(const std::string& cls)
{
    if (cls == "industrial") return 0.100;
    if (cls == "precision")  return 0.025;
    if (cls == "metrology")  return 0.005;
    return -1.0;
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;
    (void)wp;

    if (in.camera_count < 2) {
        r.add("DFM-INPUT", "error",
              "photogrammetry_scan camera_count must be >= 2 (got " +
              std::to_string(in.camera_count) +
              ") — triangulation requires at least 2 viewpoints");
    } else if (in.camera_count < 4) {
        r.add("DFM-PHOTO-CAMERAS", "info",
              "photogrammetry_scan camera_count " +
              std::to_string(in.camera_count) +
              " < 4 — low overlap, expect degraded reconstruction RMS");
    }

    const double targetAcc = accuracyClassToMm(in.accuracy_class);
    if (targetAcc < 0.0) {
        r.add("DFM-PHOTO-CLASS", "info",
              "photogrammetry_scan accuracy_class '" + in.accuracy_class +
              "' not in known set (industrial|precision|metrology)");
    }

    if (in.coverage_pct < 0.0 || in.coverage_pct > 100.0) {
        r.add("DFM-INPUT", "error",
              "photogrammetry_scan coverage_pct must be in [0, 100] (got " +
              std::to_string(in.coverage_pct) + ")");
    } else if (in.coverage_pct < 60.0) {
        r.add("DFM-PHOTO-COVER", "warning",
              "photogrammetry_scan coverage_pct " +
              std::to_string(in.coverage_pct) +
              " % < 60 % — shadows / occlusions will degrade mesh");
    }

    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "photogrammetry_scan DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const double targetAccMm = accuracyClassToMm(in.accuracy_class);
    const double accForSig   = (targetAccMm < 0.0) ? 0.100 : targetAccMm;

    json params = {
        { "camera_count",    in.camera_count },
        { "accuracy_class",  in.accuracy_class },
        { "coverage_pct",    in.coverage_pct },
    };
    json pattern = {
        { "kind",                   kSkillId },
        { "is_inspection_only",     true },
        { "geometric_change",       false },
        { "camera_count",           in.camera_count },
        { "accuracy_class",         in.accuracy_class },
        { "target_accuracy_mm",     accForSig },
        { "coverage_pct",           in.coverage_pct },
        { "inspection_method",      "photogrammetry_multi_view" },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code", f.code },
                            { "severity", f.severity },
                            { "message", f.message }});

    ToolingMeta tooling;
    tooling.tool_type         = "camera_array";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "optical_glass";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    // Capture (~ 1 s/camera) + dense MVS (~ 30 s constant).
    tooling.est_cycle_time_s  = 30.0 + static_cast<double>(in.camera_count) * 1.0;
    tooling.extra["dfm_findings"]         = findings;
    tooling.extra["machining_constraint"] =
        "inspection — multi-camera photogrammetry, no material removal";
    tooling.extra["target_accuracy_mm"]   = accForSig;

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug("skill::photogrammetry_scan applied: cameras={} class={} cov={}%",
                  in.camera_count, in.accuracy_class, in.coverage_pct);

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

}  // namespace koocadcam::skill::photogrammetry_scan
