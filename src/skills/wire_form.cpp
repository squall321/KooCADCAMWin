// @lat: [[engine/skills#wire_form]]

#include "wire_form.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <limits>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::skill::wire_form {

using nlohmann::json;

// ── helpers ──────────────────────────────────────────────────────────────

namespace {

double segmentLen(const Waypoint& a, const Waypoint& b)
{
    const double dx = b.x_mm - a.x_mm;
    const double dy = b.y_mm - a.y_mm;
    const double dz = b.z_mm - a.z_mm;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

double polylineLength(const std::vector<Waypoint>& w)
{
    double L = 0.0;
    for (size_t i = 1; i < w.size(); ++i) L += segmentLen(w[i - 1], w[i]);
    return L;
}

// Approximate the local turning radius at an interior waypoint using the
// formula r ≈ (a + b) · sin(α/2) / 2 where α is the included angle and
// (a, b) are the half-segment lengths.  For a sharp 90° turn between unit
// segments this yields ≈ 0.354 ; for a 180° (no turn) it yields ∞.
// We use a simpler heuristic that matches the "two-pin" bend on a wire-
// forming machine: r ≈ min(a, b) / tan(α / 2) (chord-to-tangent distance).
double minBendRadius(const std::vector<Waypoint>& w)
{
    if (w.size() < 3) return std::numeric_limits<double>::infinity();
    double minR = std::numeric_limits<double>::infinity();
    for (size_t i = 1; i + 1 < w.size(); ++i) {
        const Waypoint& a = w[i - 1];
        const Waypoint& b = w[i];
        const Waypoint& c = w[i + 1];
        const double ux = a.x_mm - b.x_mm;
        const double uy = a.y_mm - b.y_mm;
        const double uz = a.z_mm - b.z_mm;
        const double vx = c.x_mm - b.x_mm;
        const double vy = c.y_mm - b.y_mm;
        const double vz = c.z_mm - b.z_mm;
        const double uLen = std::sqrt(ux * ux + uy * uy + uz * uz);
        const double vLen = std::sqrt(vx * vx + vy * vy + vz * vz);
        if (uLen < 1e-9 || vLen < 1e-9) continue;
        const double cosA = (ux * vx + uy * vy + uz * vz) / (uLen * vLen);
        const double clamped = std::max(-1.0, std::min(1.0, cosA));
        const double alpha = std::acos(clamped);              // interior angle at b
        if (alpha < 1e-9) continue;
        // Sharper bends have smaller alpha — radius proxy = min(uLen, vLen) · tan(alpha/2).
        const double r = std::min(uLen, vLen) * std::tan(alpha / 2.0);
        if (r < minR) minR = r;
    }
    return minR;
}

// Detect a 3-collinear-points situation (≤ ~0.5° deviation).
bool hasCollinearTrio(const std::vector<Waypoint>& w)
{
    constexpr double kEps = 0.5 * M_PI / 180.0;  // 0.5 deg
    for (size_t i = 1; i + 1 < w.size(); ++i) {
        const Waypoint& a = w[i - 1];
        const Waypoint& b = w[i];
        const Waypoint& c = w[i + 1];
        const double ux = a.x_mm - b.x_mm;
        const double uy = a.y_mm - b.y_mm;
        const double uz = a.z_mm - b.z_mm;
        const double vx = c.x_mm - b.x_mm;
        const double vy = c.y_mm - b.y_mm;
        const double vz = c.z_mm - b.z_mm;
        const double uLen = std::sqrt(ux * ux + uy * uy + uz * uz);
        const double vLen = std::sqrt(vx * vx + vy * vy + vz * vz);
        if (uLen < 1e-9 || vLen < 1e-9) continue;
        const double cosA = (ux * vx + uy * vy + uz * vz) / (uLen * vLen);
        // Collinear with reversal → cosA close to -1 → angle close to π.
        if (cosA <= -1.0 + kEps * kEps / 2.0) return true;
    }
    return false;
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.waypoints.size() < 2) {
        r.add("DFM-INPUT", "error",
              "wire_form requires ≥ 2 waypoints (got " +
              std::to_string(in.waypoints.size()) + ")");
        return r;
    }
    if (in.wire_dia_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "wire_form wire_dia_mm must be > 0 (got " +
              std::to_string(in.wire_dia_mm) + ")");
    }

    // Reject zero-length segments.
    for (size_t i = 1; i < in.waypoints.size(); ++i) {
        if (segmentLen(in.waypoints[i - 1], in.waypoints[i]) < 1e-6) {
            r.add("DFM-INPUT", "error",
                  "wire_form zero-length segment at index " +
                  std::to_string(i));
            break;
        }
    }

    const double minR = minBendRadius(in.waypoints);
    if (in.wire_dia_mm > 0.0 && std::isfinite(minR) &&
        minR < 2.0 * in.wire_dia_mm) {
        r.add("DFM-WF-BEND-R", "error",
              "wire_form implied bend radius " + std::to_string(minR) +
              " mm < 2 × wire_dia (" +
              std::to_string(2.0 * in.wire_dia_mm) +
              " mm) — outer-fibre cracking risk");
    }

    if (hasCollinearTrio(in.waypoints)) {
        r.add("DFM-WF-COLLINEAR", "info",
              "wire_form contains a 180° reversal at one or more interior "
              "waypoints — wastes a programming step");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────
//
// Metadata-only stamp; geometry passes through unchanged.

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "wire_form DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const double L = polylineLength(in.waypoints);
    const double minR = minBendRadius(in.waypoints);
    const int bendCount = static_cast<int>(in.waypoints.size()) - 2;

    json wpJ = json::array();
    for (const auto& p : in.waypoints) {
        wpJ.push_back({ { "x_mm", p.x_mm },
                        { "y_mm", p.y_mm },
                        { "z_mm", p.z_mm } });
    }

    json params = {
        { "waypoints",     wpJ },
        { "wire_dia_mm",   in.wire_dia_mm },
    };
    json pattern = {
        { "kind",                 kSkillId },
        { "waypoints",            wpJ },
        { "waypoint_count",       static_cast<int>(in.waypoints.size()) },
        { "wire_dia_mm",          in.wire_dia_mm },
        { "polyline_length_mm",   L },
        { "bend_count",           std::max(0, bendCount) },
        { "min_bend_radius_mm",   std::isfinite(minR) ? minR : 0.0 },
        { "geometry_changed",     false },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code", f.code },
                            { "severity", f.severity },
                            { "message", f.message }});

    ToolingMeta tooling;
    tooling.tool_type         = "wire_former";
    tooling.tool_dia_mm       = in.wire_dia_mm;
    tooling.tool_length_mm    = L;
    tooling.tool_material     = "tool_steel";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    tooling.est_cycle_time_s  = 1.0 + bendCount * 0.4 + L * 0.02;
    tooling.extra = json{
        { "process",            "wire_forming" },
        { "polyline_length_mm", L },
        { "bend_count",         std::max(0, bendCount) },
        { "min_bend_radius_mm", std::isfinite(minR) ? minR : 0.0 },
        { "geometric_no_op",    true },
        { "dfm_findings",       findings },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug("skill::wire_form applied: {} waypoints, L={:.2f} mm, bends={}",
                  in.waypoints.size(), L, bendCount);

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

}  // namespace koocadcam::skill::wire_form
