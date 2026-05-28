// @lat: [[engine/skills#probe_point]]

#include "probe_point.hpp"

#include "Workpiece.hpp"

#include <gp_Pnt.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <vector>

namespace koocadcam::skill::probe_point {

using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;
    if (in.tolerance_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "probe_point tolerance_mm must be > 0 (got " +
              std::to_string(in.tolerance_mm) + ")");
    }
    if (in.tolerance_mm > 5.0) {
        r.add("DFM-PROBE-TOL", "warning",
              "probe_point tolerance_mm " + std::to_string(in.tolerance_mm) +
              " > 5 mm — unusually loose for inspection");
    }

    // Slice-1 verification: probe point must be within workpiece bbox +
    // tolerance.  A stricter check would use BRepExtrema_DistShapeShape
    // against the actual workpiece faces.
    if (!wp.shape().IsNull()) {
        double xMin, yMin, zMin, xMax, yMax, zMax;
        wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
        const double tol = std::max(in.tolerance_mm, 1e-6);
        const gp_Pnt& p = in.probe_point_3d;
        const bool inBbox =
            p.X() >= xMin - tol && p.X() <= xMax + tol &&
            p.Y() >= yMin - tol && p.Y() <= yMax + tol &&
            p.Z() >= zMin - tol && p.Z() <= zMax + tol;
        if (!inBbox) {
            r.add("DFM-PROBE-LOC", "warning",
                  "probe_point (" + std::to_string(p.X()) + ", " +
                  std::to_string(p.Y()) + ", " +
                  std::to_string(p.Z()) +
                  ") lies outside workpiece bbox (+ tolerance) — probe will miss part");
        }
    }
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "probe_point DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    // No geometric change.  Build a fresh Workpiece around the SAME shape so
    // the feature history is appended without mutating the source.
    const gp_Pnt& p = in.probe_point_3d;

    json params = {
        { "probe_point_3d", { p.X(), p.Y(), p.Z() } },
        { "tolerance_mm",   in.tolerance_mm },
    };
    json pattern = {
        { "kind",                   kSkillId },
        { "is_inspection_only",     true },
        { "geometric_change",       false },
        { "probe_point_3d",         { p.X(), p.Y(), p.Z() } },
        { "tolerance_mm",           in.tolerance_mm },
        { "inspection_method",      "touch_probe_single_point" },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code", f.code },
                            { "severity", f.severity },
                            { "message", f.message }});

    ToolingMeta tooling;
    tooling.tool_type         = "touch_probe";
    tooling.tool_dia_mm       = 2.0;     // typical Renishaw stylus ball
    tooling.tool_length_mm    = 50.0;
    tooling.tool_material     = "ruby";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    tooling.est_cycle_time_s  = 3.0;   // single touch + reposition
    tooling.extra["dfm_findings"]         = findings;
    tooling.extra["machining_constraint"] =
        "inspection — single-point touch probe, no material removal";

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug("skill::probe_point applied: ({}, {}, {}) tol={}",
                  p.X(), p.Y(), p.Z(), in.tolerance_mm);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// Metadata-only: leaves no geometric fingerprint, so there is nothing for
// a topology recognizer to match against.

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    (void)wp;
    return {};
}

}  // namespace koocadcam::skill::probe_point
