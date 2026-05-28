// @lat: [[engine/skills#scan_pattern]]

#include "scan_pattern.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Frames.hpp"   // for M_PI fallback on MSVC

#include <BRepAdaptor_Surface.hxx>
#include <gp_Pnt.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <string>
#include <utility>
#include <vector>

namespace koocadcam::skill::scan_pattern {

using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;
    const bool patternOk =
        in.pattern == "grid" ||
        in.pattern == "spiral" ||
        in.pattern == "diagonal";
    if (!patternOk) {
        r.add("DFM-INPUT", "error",
              "scan_pattern unknown pattern '" + in.pattern +
              "' (expected grid | spiral | diagonal)");
    }
    if (in.point_count < 4) {
        r.add("DFM-INPUT", "error",
              "scan_pattern point_count " + std::to_string(in.point_count) +
              " < 4 (need at least 4 points for a meaningful scan)");
    }
    if (in.tolerance_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "scan_pattern tolerance_mm must be > 0");
    }

    // Verify the entry face resolves and has area >= 100 mm².
    auto entryId = wp.resolve(in.entry_face_datum);
    if (!entryId) {
        r.add("DFM-PROBE-FACE", "error",
              "scan_pattern entry_face_datum did not resolve");
    } else {
        if (!wp.isFacePlanar(*entryId)) {
            r.add("DFM-PROBE-FACE", "error",
                  "scan_pattern entry face must be planar");
        }
        const double area = wp.faceArea(*entryId);
        if (area < 100.0) {
            r.add("DFM-PROBE-AREA", "error",
                  "scan_pattern entry face area " + std::to_string(area) +
                  " mm^2 < 100 mm^2 — pattern doesn't fit");
        }
    }
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

namespace {

// Generate a list of (u, v) probe points inside [0, 1]^2 for the given
// pattern.  We then map (u, v) → world XYZ via the face's UV
// parameterisation.
//
// For "grid", arrange a ⌈sqrt(N)⌉ × ⌈sqrt(N)⌉ grid (clamped to ≤ N).
// For "spiral", emit Archimedean spiral points centred on (0.5, 0.5).
// For "diagonal", line N points along the (0,0)→(1,1) diagonal.
std::vector<std::pair<double, double>>
generateUV(const std::string& pattern, int N)
{
    std::vector<std::pair<double, double>> uv;
    uv.reserve(static_cast<size_t>(N));
    if (pattern == "grid") {
        // ⌈sqrt(N)⌉ steps per side; clamp count to ≤ N.
        const int side = std::max(2, static_cast<int>(std::ceil(std::sqrt(static_cast<double>(N)))));
        for (int j = 0; j < side; ++j) {
            for (int i = 0; i < side; ++i) {
                if (static_cast<int>(uv.size()) >= N) break;
                const double u = (side == 1) ? 0.5 :
                                 static_cast<double>(i) / static_cast<double>(side - 1);
                const double v = (side == 1) ? 0.5 :
                                 static_cast<double>(j) / static_cast<double>(side - 1);
                uv.emplace_back(u, v);
            }
        }
    } else if (pattern == "spiral") {
        // Archimedean: r(t) = a t, θ(t) = b t, t ∈ [0, 1].
        // Fit so the spiral fills the UV square (max radius = 0.5).
        const double turns = 3.0;
        for (int i = 0; i < N; ++i) {
            const double t = (N <= 1) ? 0.0 :
                             static_cast<double>(i) / static_cast<double>(N - 1);
            const double r = 0.45 * t;          // max ~ 0.45 to stay inside square
            const double th = turns * 2.0 * M_PI * t;
            const double u = 0.5 + r * std::cos(th);
            const double v = 0.5 + r * std::sin(th);
            uv.emplace_back(std::clamp(u, 0.0, 1.0),
                            std::clamp(v, 0.0, 1.0));
        }
    } else {
        // "diagonal" — N points along (0,0)→(1,1).
        for (int i = 0; i < N; ++i) {
            const double t = (N <= 1) ? 0.5 :
                             static_cast<double>(i) / static_cast<double>(N - 1);
            uv.emplace_back(t, t);
        }
    }
    return uv;
}

}  // namespace

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "scan_pattern DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    auto entryId = wp.resolve(in.entry_face_datum);
    if (!entryId) throw SkillError("scan_pattern: entry_face datum unresolved");

    BRepAdaptor_Surface surf(wp.face(*entryId));
    const double uMin = surf.FirstUParameter();
    const double uMax = surf.LastUParameter();
    const double vMin = surf.FirstVParameter();
    const double vMax = surf.LastVParameter();

    // Generate UV in [0, 1]^2, map to actual face UV range, then to XYZ.
    const auto uvs = generateUV(in.pattern, in.point_count);
    json pointsJson = json::array();
    std::vector<gp_Pnt> points;
    points.reserve(uvs.size());
    for (const auto& [u01, v01] : uvs) {
        const double u = uMin + u01 * (uMax - uMin);
        const double v = vMin + v01 * (vMax - vMin);
        const gp_Pnt p = surf.Value(u, v);
        points.push_back(p);
        pointsJson.push_back({ p.X(), p.Y(), p.Z() });
    }

    // No geometric change.
    json params = {
        { "entry_face_id", *entryId },
        { "pattern",       in.pattern },
        { "point_count",   in.point_count },
        { "tolerance_mm",  in.tolerance_mm },
    };
    json pattern = {
        { "kind",                 kSkillId },
        { "is_inspection_only",   true },
        { "geometric_change",     false },
        { "pattern_type",         in.pattern },
        { "actual_point_count",   static_cast<int>(points.size()) },
        { "points",               pointsJson },
        { "tolerance_mm",         in.tolerance_mm },
        { "entry_face_id",        *entryId },
        { "inspection_method",    "touch_probe_planar_scan" },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code", f.code },
                            { "severity", f.severity },
                            { "message", f.message }});

    ToolingMeta tooling;
    tooling.tool_type         = "touch_probe";
    tooling.tool_dia_mm       = 2.0;
    tooling.tool_length_mm    = 50.0;
    tooling.tool_material     = "ruby";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    // ~2.5 s per probed point including reposition.
    tooling.est_cycle_time_s  = std::max(5.0, 2.5 * static_cast<double>(points.size()));
    tooling.extra["dfm_findings"]         = findings;
    tooling.extra["machining_constraint"] =
        "inspection — multi-point planar scan, no material removal";

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug("skill::scan_pattern applied: pattern={} N={} actual={} face={}",
                  in.pattern, in.point_count, points.size(), *entryId);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// Metadata-only: leaves no geometric fingerprint.

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    (void)wp;
    return {};
}

}  // namespace koocadcam::skill::scan_pattern
