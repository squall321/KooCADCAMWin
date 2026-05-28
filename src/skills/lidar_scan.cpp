// @lat: [[engine/skills#lidar_scan]]

#include "lidar_scan.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <string>
#include <vector>

namespace koocadcam::skill::lidar_scan {

using nlohmann::json;

namespace {

bool isKnownClass(const std::string& c)
{
    return c == "ToF" || c == "AMCW" || c == "FMCW";
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;
    (void)wp;

    if (in.range_m <= 0.0 || in.range_m > 1000.0) {
        r.add("DFM-INPUT", "error",
              "lidar_scan range_m must be in (0, 1000] m (got " +
              std::to_string(in.range_m) + ")");
    } else if (in.range_m < 0.3) {
        r.add("DFM-LIDAR-RANGE", "info",
              "lidar_scan range_m " + std::to_string(in.range_m) +
              " < 0.3 m — LIDAR overkill, consider SL / interferometry");
    }

    if (in.points_per_sec <= 0.0) {
        r.add("DFM-INPUT", "error",
              "lidar_scan points_per_sec must be > 0 (got " +
              std::to_string(in.points_per_sec) + ")");
    } else if (in.points_per_sec < 10000.0) {
        r.add("DFM-LIDAR-RATE", "warning",
              "lidar_scan points_per_sec " +
              std::to_string(in.points_per_sec) +
              " < 10 000 — too sparse for industrial QA");
    }

    if (!isKnownClass(in.ranging_class)) {
        r.add("DFM-LIDAR-CLASS", "info",
              "lidar_scan ranging_class '" + in.ranging_class +
              "' not in known set (ToF|AMCW|FMCW)");
    }

    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "lidar_scan DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    json params = {
        { "range_m",        in.range_m },
        { "points_per_sec", in.points_per_sec },
        { "ranging_class",  in.ranging_class },
    };
    json pattern = {
        { "kind",                kSkillId },
        { "is_inspection_only",  true },
        { "geometric_change",    false },
        { "range_m",             in.range_m },
        { "points_per_sec",      in.points_per_sec },
        { "ranging_class",       in.ranging_class },
        { "long_range",          in.range_m >= 10.0 },
        { "inspection_method",   "lidar_pointcloud_sweep" },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code", f.code },
                            { "severity", f.severity },
                            { "message", f.message }});

    ToolingMeta tooling;
    tooling.tool_type         = "lidar_scanner";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "laser_diode";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    // Cycle: 1M-point typical scan → time ≈ 1e6 / points_per_sec, ≥ 5 s.
    const double targetPoints = 1.0e6;
    tooling.est_cycle_time_s  =
        std::max(5.0, targetPoints / std::max(1.0, in.points_per_sec));
    tooling.extra["dfm_findings"]         = findings;
    tooling.extra["machining_constraint"] =
        "inspection — LIDAR sweep, no material removal";

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug("skill::lidar_scan applied: range={} m rate={} pt/s class={}",
                  in.range_m, in.points_per_sec, in.ranging_class);

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

}  // namespace koocadcam::skill::lidar_scan
