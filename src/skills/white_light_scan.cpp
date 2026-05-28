// @lat: [[engine/skills#white_light_scan]]

#include "white_light_scan.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

namespace koocadcam::skill::white_light_scan {

using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.point_density_per_mm2 <= 0.0) {
        r.add("DFM-INPUT", "error",
              "white_light_scan point_density_per_mm2 must be > 0 (got " +
              std::to_string(in.point_density_per_mm2) + ")");
    }
    if (in.rms_tolerance_nm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "white_light_scan rms_tolerance_nm must be > 0 (got " +
              std::to_string(in.rms_tolerance_nm) + ")");
    }
    if (in.measured_rms_nm < 0.0) {
        r.add("DFM-INPUT", "error",
              "white_light_scan measured_rms_nm cannot be negative (got " +
              std::to_string(in.measured_rms_nm) + ")");
    }

    auto faceId = wp.resolve(in.target_face_datum);
    if (!faceId) {
        r.add("DFM-PROBE-FACE", "error",
              "white_light_scan target_face_datum did not resolve");
    } else {
        const double area = wp.faceArea(*faceId);
        if (area <= 0.0) {
            r.add("DFM-PROBE-AREA", "error",
                  "white_light_scan: target face has non-positive area");
        }
    }

    if (in.measured_rms_nm > 0.0 && in.measured_rms_nm > in.rms_tolerance_nm) {
        r.add("DFM-WLS-OUTSPEC", "warning",
              "white_light_scan: measured RMS " +
              std::to_string(in.measured_rms_nm) + " nm exceeds tolerance " +
              std::to_string(in.rms_tolerance_nm) + " nm");
    }

    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "white_light_scan DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    auto faceId = wp.resolve(in.target_face_datum);
    if (!faceId) throw SkillError("white_light_scan: face datum unresolved");

    const double area_mm2  = wp.faceArea(*faceId);
    const double points    = area_mm2 * in.point_density_per_mm2;
    const bool   hasMeas   = in.measured_rms_nm > 0.0;
    const bool   conforms  = hasMeas && in.measured_rms_nm <= in.rms_tolerance_nm;

    json params = {
        { "target_face_id",         *faceId },
        { "point_density_per_mm2",  in.point_density_per_mm2 },
        { "measured_rms_nm",        in.measured_rms_nm },
        { "rms_tolerance_nm",       in.rms_tolerance_nm },
    };
    json pattern = {
        { "kind",                   kSkillId },
        { "is_inspection_only",     true },
        { "geometric_change",       false },
        { "target_face_id",         *faceId },
        { "area_mm2",               area_mm2 },
        { "point_density_per_mm2",  in.point_density_per_mm2 },
        { "estimated_point_count",  static_cast<long long>(points) },
        { "measured_rms_nm",        in.measured_rms_nm },
        { "rms_tolerance_nm",       in.rms_tolerance_nm },
        { "has_measurement",        hasMeas },
        { "conforms",               conforms },
        { "inspection_method",      "white_light_interferometry" },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code", f.code },
                            { "severity", f.severity },
                            { "message", f.message }});

    ToolingMeta tooling;
    tooling.tool_type         = "white_light_interferometer";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "optical_glass";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    // Scan time ≈ ramp + acquisition (rough: 1 s per 10 000 points).
    tooling.est_cycle_time_s  = std::max(30.0, points * 1.0e-4);
    tooling.extra["dfm_findings"]         = findings;
    tooling.extra["machining_constraint"] =
        "inspection — non-contact optical scan, no material removal";

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug("skill::white_light_scan applied: face={} area={} pts={}",
                  *faceId, area_mm2, static_cast<long long>(points));

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

}  // namespace koocadcam::skill::white_light_scan
