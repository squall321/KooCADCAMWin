// @lat: [[engine/skills#cmm_scan]]

#include "cmm_scan.hpp"

#include "Workpiece.hpp"

#include <gp_Pnt.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

namespace koocadcam::skill::cmm_scan {

using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.nominal_points_3d.empty()) {
        r.add("DFM-INPUT", "error",
              "cmm_scan nominal_points_3d must contain at least 1 waypoint");
    }
    if (in.target_tolerance_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "cmm_scan target_tolerance_mm must be > 0 (got " +
              std::to_string(in.target_tolerance_mm) + ")");
    }
    if (in.stylus_diameter_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "cmm_scan stylus_diameter_mm must be > 0 (got " +
              std::to_string(in.stylus_diameter_mm) + ")");
    }

    if (!in.measured_mm.empty() &&
        in.measured_mm.size() != in.nominal_points_3d.size())
    {
        r.add("DFM-INPUT", "error",
              "cmm_scan measured_mm size (" +
              std::to_string(in.measured_mm.size()) +
              ") must equal nominal_points_3d size (" +
              std::to_string(in.nominal_points_3d.size()) + ")");
    }

    if (in.target_tolerance_mm > 5.0) {
        r.add("DFM-PROBE-TOL", "warning",
              "cmm_scan target_tolerance_mm " +
              std::to_string(in.target_tolerance_mm) +
              " > 5 mm — unusually loose for CMM");
    }

    // Per-point out-of-spec → warning.  Only when measured data present.
    if (!in.measured_mm.empty() &&
        in.measured_mm.size() == in.nominal_points_3d.size())
    {
        const double tol = in.target_tolerance_mm;
        int outCount = 0;
        for (double d : in.measured_mm) {
            if (std::abs(d) > tol) ++outCount;
        }
        if (outCount > 0) {
            r.add("DFM-CMM-OUTSPEC", "warning",
                  "cmm_scan: " + std::to_string(outCount) + " of " +
                  std::to_string(in.measured_mm.size()) +
                  " waypoints exceed target_tolerance ±" +
                  std::to_string(tol) + " mm");
        }
    }

    // Light bbox sanity for nominal points (warning, not error).
    if (!wp.shape().IsNull() && !in.nominal_points_3d.empty()) {
        double xMin, yMin, zMin, xMax, yMax, zMax;
        wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
        const double tol = std::max(in.target_tolerance_mm, 1.0);
        int outside = 0;
        for (const auto& p : in.nominal_points_3d) {
            const bool inBbox =
                p.X() >= xMin - tol && p.X() <= xMax + tol &&
                p.Y() >= yMin - tol && p.Y() <= yMax + tol &&
                p.Z() >= zMin - tol && p.Z() <= zMax + tol;
            if (!inBbox) ++outside;
        }
        if (outside > 0) {
            r.add("DFM-PROBE-LOC", "warning",
                  "cmm_scan: " + std::to_string(outside) +
                  " nominal waypoint(s) outside workpiece bbox");
        }
    }

    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "cmm_scan DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    json nominalJson = json::array();
    for (const auto& p : in.nominal_points_3d)
        nominalJson.push_back({ p.X(), p.Y(), p.Z() });

    json measuredJson = json::array();
    for (double d : in.measured_mm) measuredJson.push_back(d);

    // Per-point deltas: signed measured deviation (same as measured_mm
    // when caller has supplied them).  When absent, an empty array.
    const bool hasMeasured =
        !in.measured_mm.empty() &&
        in.measured_mm.size() == in.nominal_points_3d.size();
    json deltasJson = json::array();
    bool allConform = hasMeasured;   // false if no measurement
    if (hasMeasured) {
        for (double d : in.measured_mm) {
            deltasJson.push_back(d);
            if (std::abs(d) > in.target_tolerance_mm) allConform = false;
        }
    }

    json params = {
        { "nominal_points_3d",   nominalJson },
        { "measured_mm",         measuredJson },
        { "target_tolerance_mm", in.target_tolerance_mm },
        { "stylus_diameter_mm",  in.stylus_diameter_mm },
    };
    json pattern = {
        { "kind",                   kSkillId },
        { "is_inspection_only",     true },
        { "geometric_change",       false },
        { "waypoint_count",         static_cast<int>(in.nominal_points_3d.size()) },
        { "nominal_points_3d",      nominalJson },
        { "measured_mm",            measuredJson },
        { "deltas_mm",              deltasJson },
        { "target_tolerance_mm",    in.target_tolerance_mm },
        { "stylus_diameter_mm",     in.stylus_diameter_mm },
        { "has_measurements",       hasMeasured },
        { "all_conform",            allConform },
        { "inspection_method",      "cmm_waypoint_scan" },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code", f.code },
                            { "severity", f.severity },
                            { "message", f.message }});

    ToolingMeta tooling;
    tooling.tool_type         = "touch_probe";
    tooling.tool_dia_mm       = in.stylus_diameter_mm;
    tooling.tool_length_mm    = 50.0;
    tooling.tool_material     = "ruby";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    // ~ 3 s per waypoint (touch + reposition + safe move).
    const double N = static_cast<double>(in.nominal_points_3d.size());
    tooling.est_cycle_time_s  = std::max(10.0, 3.0 * N);
    tooling.extra["dfm_findings"]         = findings;
    tooling.extra["machining_constraint"] =
        "inspection — CMM waypoint scan, no material removal";

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug("skill::cmm_scan applied: N={} tol={} stylus={} measured={}",
                  in.nominal_points_3d.size(), in.target_tolerance_mm,
                  in.stylus_diameter_mm, hasMeasured ? "yes" : "no");

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// Metadata-only: leaves no geometric fingerprint.

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

}  // namespace koocadcam::skill::cmm_scan
