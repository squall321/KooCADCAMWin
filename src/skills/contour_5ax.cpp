// @lat: [[engine/skills#contour_5ax]]

#include "contour_5ax.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepAdaptor_Surface.hxx>
#include <BRep_Builder.hxx>
#include <Standard_Failure.hxx>
#include <TopoDS_Compound.hxx>
#include <gp_Ax2.hxx>
#include <gp_Cylinder.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <vector>

namespace koocadcam::skill::contour_5ax {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;
    if (in.tool_dia_mm < 1.5) {
        r.add("DFM-002", "error",
              "contour_5ax tool_dia_mm " + std::to_string(in.tool_dia_mm) +
              " < 1.5 mm (5-axis end-mill min)");
    }
    if (in.tool_length_mm <= 0.0) {
        r.add("DFM-INPUT", "error", "contour_5ax tool_length_mm must be > 0");
    }
    if (in.contour_polyline_3d.size() < 2) {
        r.add("DFM-INPUT", "error",
              "contour_5ax contour_polyline_3d requires >= 2 waypoints (got " +
              std::to_string(in.contour_polyline_3d.size()) + ")");
    }
    for (size_t i = 0; i < in.contour_polyline_3d.size(); ++i) {
        const double t = in.contour_polyline_3d[i][3];
        if (std::abs(t) > 60.0) {
            r.add("DFM-CONTOUR-TILT", "error",
                  "contour_5ax waypoint[" + std::to_string(i) + "] tilt_deg " +
                  std::to_string(t) + " outside [-60, 60]");
            break;
        }
    }
    r.add("DFM-005", "info",
          "contour_5ax requires 5-axis machine kinematics — per-waypoint "
          "tool-axis tilt for finishing complex sculpted surfaces");
    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

namespace {

// Tilt the -Z (downward) tool axis by tiltDeg around (tangent × -Z), using
// Rodrigues rotation.  Matches the convention in swarf_milling but with a
// per-waypoint angle.  If tangent is parallel to -Z, fall back to tilting
// around +X.
gp_Dir computeTiltedToolAxis(const std::array<double, 4>& prev,
                             const std::array<double, 4>& next,
                             double tiltDeg)
{
    const gp_Vec tangent(next[0] - prev[0],
                         next[1] - prev[1],
                         next[2] - prev[2]);
    const double tLen = tangent.Magnitude();
    const gp_Dir z0(0.0, 0.0, -1.0);

    if (tLen < 1e-9 || std::abs(tiltDeg) < 1e-3) {
        return z0;
    }
    const gp_Dir tDir(tangent);

    gp_Vec rotAxis = gp_Vec(z0).Crossed(gp_Vec(tDir));
    if (rotAxis.Magnitude() < 1e-6) {
        rotAxis = gp_Vec(1.0, 0.0, 0.0);
    }
    rotAxis.Normalize();
    const gp_Dir R(rotAxis);

    const double a = tiltDeg * M_PI / 180.0;
    const double c = std::cos(a);
    const double s = std::sin(a);
    const gp_Vec z0v(z0);
    const gp_Vec Rv (R);
    // Rodrigues: v' = v cos θ + (R × v) sin θ + R (R · v) (1 - cos θ)
    const gp_Vec v_rot =
        z0v * c + Rv.Crossed(z0v) * s + Rv * (Rv.Dot(z0v) * (1.0 - c));
    return gp_Dir(v_rot);
}

}  // namespace

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "contour_5ax DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    auto entryId = wp.resolve(in.entry_face_datum);
    if (!entryId) throw SkillError("contour_5ax: entry_face datum unresolved");

    const double radius  = in.tool_dia_mm / 2.0;
    const double halfLen = in.tool_length_mm / 2.0;
    const size_t N       = in.contour_polyline_3d.size();

    std::vector<TopoDS_Shape> cylinders;
    cylinders.reserve(N);
    double tiltSum = 0.0;
    double tiltAbsMax = 0.0;
    for (size_t i = 0; i < N; ++i) {
        const auto& wp_i = in.contour_polyline_3d[i];
        const auto& prev = (i == 0) ? wp_i : in.contour_polyline_3d[i - 1];
        const auto& next = (i + 1 == N) ? wp_i : in.contour_polyline_3d[i + 1];
        const double tilt = wp_i[3];
        tiltSum += tilt;
        tiltAbsMax = std::max(tiltAbsMax, std::abs(tilt));
        const gp_Dir toolAxis = computeTiltedToolAxis(prev, next, tilt);

        const gp_Pnt base(
            wp_i[0] - toolAxis.X() * halfLen,
            wp_i[1] - toolAxis.Y() * halfLen,
            wp_i[2] - toolAxis.Z() * halfLen);
        const gp_Ax2 ax(base, toolAxis);
        try {
            cylinders.push_back(pr::cylinder(ax, radius, in.tool_length_mm));
        } catch (const Standard_Failure&) {
            continue;
        }
    }
    if (cylinders.empty())
        throw SkillError("contour_5ax: no valid cylinders built from contour_polyline_3d");

    TopoDS_Compound compound;
    BRep_Builder builder;
    builder.MakeCompound(compound);
    for (const auto& c : cylinders) builder.Add(compound, c);

    const TopoDS_Shape newShape = pr::cut(wp.shape(), compound);

    double pathLen = 0.0;
    for (size_t i = 1; i < N; ++i) {
        const auto& a = in.contour_polyline_3d[i - 1];
        const auto& b = in.contour_polyline_3d[i];
        pathLen += std::sqrt(
            (b[0] - a[0]) * (b[0] - a[0]) +
            (b[1] - a[1]) * (b[1] - a[1]) +
            (b[2] - a[2]) * (b[2] - a[2]));
    }
    const double avgTilt = (N > 0) ? tiltSum / static_cast<double>(N) : 0.0;

    json wpJson = json::array();
    for (const auto& w : in.contour_polyline_3d)
        wpJson.push_back({ w[0], w[1], w[2], w[3] });

    json params = {
        { "entry_face_id",       *entryId },
        { "contour_polyline_3d", wpJson },
        { "tool_dia_mm",         in.tool_dia_mm },
        { "tool_length_mm",      in.tool_length_mm },
    };
    json pattern = {
        { "kind",                    kSkillId },
        { "is_5axis_cut",            true },
        { "cylindrical_face_count",  N },
        { "waypoint_count",          N },
        { "avg_tilt_deg",            avgTilt },
        { "max_abs_tilt_deg",        tiltAbsMax },
        { "tilt_varies",             tiltAbsMax > 1e-3 },
        { "guide_length",            pathLen },
        { "geometry",                "tilted_cylinder_chain_per_waypoint_tilt" },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code", f.code },
                            { "severity", f.severity },
                            { "message", f.message }});

    ToolingMeta tooling;
    tooling.tool_type         = "ball_mill";   // finishing surface contours
    tooling.tool_dia_mm       = in.tool_dia_mm;
    tooling.tool_length_mm    = in.tool_length_mm;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 4;
    tooling.cutting_speed_sfm = 350.0;
    tooling.feed_per_tooth_mm = 0.02;        // fine finishing
    tooling.stock_removed_mm3 = M_PI * radius * radius * pathLen;
    tooling.est_cycle_time_s  = std::max(5.0, pathLen / 15.0);
    tooling.extra["dfm_findings"]         = findings;
    tooling.extra["machining_constraint"] =
        "5-axis contour finishing — tool axis interpolated through waypoint tilts";

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    wpNew->addFeature(sig);

    spdlog::debug("skill::contour_5ax applied: N={} dia={} avg_tilt={:.2f} faces {}->{}",
                  N, in.tool_dia_mm, avgTilt,
                  wp.faceCount(), wpNew->faceCount());

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// Heuristic: ≥ 2 cylindrical faces whose axes are NOT parallel to a global
// axis, AND whose axes show VARIATION across the chain (different tilt
// angles from vertical).  Variation distinguishes contour_5ax from
// swarf_milling (constant tilt) — but the marker is fragile, so confidence
// stays low (~0.3).

namespace {

bool isAxisAlignedDir(const gp_Dir& d)
{
    const double ax = std::abs(d.X());
    const double ay = std::abs(d.Y());
    const double az = std::abs(d.Z());
    return ax > 0.99 || ay > 0.99 || az > 0.99;
}

}  // namespace

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;

    struct Cyl {
        int     faceIdx;
        gp_Pnt  centre;
        double  radius;
        double  tiltDeg;
    };
    std::vector<Cyl> tilted;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFaceCylinder(i)) continue;
        BRepAdaptor_Surface surf(wp.face(i));
        const gp_Cylinder cyl = surf.Cylinder();
        const gp_Dir adir = cyl.Axis().Direction();
        if (isAxisAlignedDir(adir)) continue;
        const double cosT = std::clamp(adir.Dot(gp_Dir(0, 0, -1)), -1.0, 1.0);
        const double tDeg = std::acos(std::abs(cosT)) * 180.0 / M_PI;
        tilted.push_back({ i, wp.faceCenter(i), cyl.Radius(), tDeg });
    }
    if (tilted.size() < 2) return out;

    // Check whether tilt varies; if not, defer to swarf_milling.
    double tMin = tilted[0].tiltDeg, tMax = tilted[0].tiltDeg;
    for (const auto& t : tilted) {
        tMin = std::min(tMin, t.tiltDeg);
        tMax = std::max(tMax, t.tiltDeg);
    }
    const bool tiltVaries = (tMax - tMin) > 2.0;   // > 2° spread
    if (!tiltVaries) return out;

    // Order centres along greatest spread.
    int bestI = 0, bestJ = 1;
    double bestD = 0.0;
    for (size_t a = 0; a < tilted.size(); ++a)
        for (size_t b = a + 1; b < tilted.size(); ++b) {
            const double d = tilted[a].centre.Distance(tilted[b].centre);
            if (d > bestD) { bestD = d; bestI = static_cast<int>(a); bestJ = static_cast<int>(b); }
        }
    const gp_Pnt& A = tilted[bestI].centre;
    const gp_Pnt& B = tilted[bestJ].centre;
    const gp_Vec lineDir(A, B);
    if (lineDir.Magnitude() < 1e-6) return out;
    const gp_Vec lineUnit = lineDir.Normalized();

    struct Sortable { double t; int faceIdx; gp_Pnt centre; double tiltDeg; double radius; };
    std::vector<Sortable> sortable;
    for (const auto& c : tilted) {
        const gp_Vec v(A, c.centre);
        sortable.push_back({ v.Dot(lineUnit), c.faceIdx, c.centre, c.tiltDeg, c.radius });
    }
    std::sort(sortable.begin(), sortable.end(),
              [](const Sortable& a, const Sortable& b) { return a.t < b.t; });

    json poly    = json::array();
    json faceIds = json::array();
    double rSum = 0.0;
    for (const auto& s : sortable) {
        poly.push_back({ s.centre.X(), s.centre.Y(), s.centre.Z(), s.tiltDeg });
        faceIds.push_back(s.faceIdx);
        rSum += s.radius;
    }
    const double avgR = rSum / static_cast<double>(sortable.size());

    json recovered = {
        { "contour_polyline_3d", poly },
        { "tool_dia_mm",         2.0 * avgR },
        { "tool_length_mm",      0.0 },
    };
    json matched = {
        { "cylinder_face_ids", faceIds },
        { "tilt_min_deg",      tMin },
        { "tilt_max_deg",      tMax },
        { "linear_extent",     bestD },
    };
    out.push_back(RecognizedFeature{
        kSkillId, recovered, /*confidence*/ 0.30, matched
    });
    return out;
}

}  // namespace koocadcam::skill::contour_5ax
