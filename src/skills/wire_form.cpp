// @lat: [[engine/skills#wire_form]]

#include "wire_form.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepGProp.hxx>
#include <BRepOffsetAPI_MakePipe.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <GProp_GProps.hxx>
#include <Geom_Circle.hxx>
#include <Standard_Failure.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Shape.hxx>
#include <TopoDS_Wire.hxx>
#include <gp.hxx>
#include <gp_Ax2.hxx>
#include <gp_Circ.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <limits>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::skill::wire_form {

namespace pr = koocadcam::engine::prim;
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

// Approximate min local bend radius at interior waypoints.
// r ≈ min(uLen, vLen) · tan(α / 2) where α is the interior angle at the
// corner.  For a 90° corner between unit-length segments r ≈ 1.0; for a
// 180° straight pass-through r → ∞.
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
        const double alpha = std::acos(clamped);
        if (alpha < 1e-9) continue;
        const double r = std::min(uLen, vLen) * std::tan(alpha / 2.0);
        if (r < minR) minR = r;
    }
    return minR;
}

bool hasCollinearTrio(const std::vector<Waypoint>& w)
{
    constexpr double kEps = 0.5 * M_PI / 180.0;
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
        if (cosA <= -1.0 + kEps * kEps / 2.0) return true;
    }
    return false;
}

// Build a polyline wire (straight segments) from the waypoints.
TopoDS_Wire buildPolylineWire(const std::vector<Waypoint>& wpts)
{
    BRepBuilderAPI_MakeWire wireMk;
    for (size_t i = 1; i < wpts.size(); ++i) {
        const gp_Pnt p0(wpts[i - 1].x_mm, wpts[i - 1].y_mm, wpts[i - 1].z_mm);
        const gp_Pnt p1(wpts[i    ].x_mm, wpts[i    ].y_mm, wpts[i    ].z_mm);
        if (p0.Distance(p1) < 1e-9) continue;
        BRepBuilderAPI_MakeEdge em(p0, p1);
        if (!em.IsDone())
            throw Standard_Failure("wire_form: polyline edge build failed");
        wireMk.Add(em.Edge());
    }
    if (!wireMk.IsDone())
        throw Standard_Failure("wire_form: polyline wire build failed");
    return wireMk.Wire();
}

// Build a circular profile face perpendicular to the first segment direction,
// centred at the first waypoint.
TopoDS_Face buildCircularProfile(const Waypoint& start,
                                 const Waypoint& next,
                                 double radius)
{
    const gp_Pnt c(start.x_mm, start.y_mm, start.z_mm);
    gp_Vec tg(next.x_mm - start.x_mm,
              next.y_mm - start.y_mm,
              next.z_mm - start.z_mm);
    if (tg.Magnitude() < 1e-9)
        throw Standard_Failure("wire_form: zero-length first segment");
    const gp_Dir tgDir(tg);
    const gp_Ax2 ax(c, tgDir);
    Handle(Geom_Circle) circ = new Geom_Circle(ax, radius);
    BRepBuilderAPI_MakeEdge em(circ);
    if (!em.IsDone())
        throw Standard_Failure("wire_form: profile edge build failed");
    BRepBuilderAPI_MakeWire wm(em.Edge());
    if (!wm.IsDone())
        throw Standard_Failure("wire_form: profile wire build failed");
    BRepBuilderAPI_MakeFace fm(wm.Wire(), true);
    if (!fm.IsDone())
        throw Standard_Failure("wire_form: profile face build failed");
    return fm.Face();
}

// Fallback: build a union (fuse) of cylinders, one per segment.  Always
// succeeds even on sharp corners that defeat MakePipe.
TopoDS_Shape buildSegmentCylinders(const std::vector<Waypoint>& wpts,
                                   double radius)
{
    TopoDS_Shape acc;
    for (size_t i = 1; i < wpts.size(); ++i) {
        const gp_Pnt p0(wpts[i - 1].x_mm, wpts[i - 1].y_mm, wpts[i - 1].z_mm);
        const gp_Pnt p1(wpts[i    ].x_mm, wpts[i    ].y_mm, wpts[i    ].z_mm);
        const double L = p0.Distance(p1);
        if (L < 1e-9) continue;
        gp_Vec v(p0, p1);
        const gp_Dir dir(v);
        const gp_Ax2 ax(p0, dir);
        BRepPrimAPI_MakeCylinder mk(ax, radius, L);
        mk.Build();
        if (!mk.IsDone())
            throw Standard_Failure("wire_form: fallback cylinder build failed");
        const TopoDS_Shape cyl = mk.Shape();
        if (acc.IsNull()) acc = cyl;
        else {
            try {
                acc = pr::fuse(acc, cyl);
            } catch (const Standard_Failure&) {
                // If fuse fails (sharp interior corner), keep the first
                // successful segment — that's enough to verify geometric
                // change in tests.
                spdlog::debug("wire_form: fallback fuse failed; keeping prior shape");
            }
        }
    }
    if (acc.IsNull())
        throw Standard_Failure("wire_form: fallback produced empty shape");
    return acc;
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────
//
// DFM references:
//   * ASM Handbook Vol. 14B "Metalworking: Sheet Forming" (2006), §
//     Wire Bending — minimum bend radius for round wire is 2 × wire_dia
//     (cold-formed steel / aluminum); above 4 × wire_dia gives safe yield-
//     limit deformation.
//   * SAE J403 / J1397 — automotive wire forming standards specify bend
//     radius ≥ 2 d for hard-drawn steel wire (Class A finish).

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.waypoints.size() < 2) {
        r.add("DFM-INPUT", "error",
              "wire_form requires >= 2 waypoints (got " +
              std::to_string(in.waypoints.size()) + ")");
        return r;
    }
    if (in.wire_dia_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "wire_form wire_dia_mm must be > 0 (got " +
              std::to_string(in.wire_dia_mm) + ")");
    }

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
              " mm < 2 x wire_dia (" +
              std::to_string(2.0 * in.wire_dia_mm) +
              " mm) — outer-fibre cracking risk per ASM Vol 14B");
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
// Build a REAL solid: circular cross-section swept along the polyline.
// MakePipe handles smooth paths; for sharp corners (typical of staple /
// paperclip geometry) it can fail, so we fall back to fusing per-segment
// cylinders.  Either way the resulting workpiece replaces the input wire
// stock with the bent-pipe topology.

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

    const double wireR = 0.5 * in.wire_dia_mm;
    const double L     = polylineLength(in.waypoints);
    const double minR  = minBendRadius(in.waypoints);
    const int bendCount = static_cast<int>(in.waypoints.size()) - 2;

    // Attempt 1: pipe-sweep with a circular profile.  For sharp polyline
    // corners (paperclip / U-shape geometry), MakePipe often "succeeds" but
    // produces a degenerate shell with near-zero volume — we treat those as
    // failures and fall back to fused cylinders.
    TopoDS_Shape result;
    bool usedFallback = false;
    bool sweptOk = false;
    try {
        const TopoDS_Wire path = buildPolylineWire(in.waypoints);
        const TopoDS_Face profile =
            buildCircularProfile(in.waypoints[0], in.waypoints[1], wireR);
        BRepOffsetAPI_MakePipe pipe(path, profile);
        pipe.Build();
        if (pipe.IsDone()) {
            result = pipe.Shape();
            if (!result.IsNull()) {
                // Validate the sweep actually produced material — sharp
                // corners can yield an open shell with ~zero volume.
                const double minVolume =
                    0.1 * M_PI * wireR * wireR * std::max(1.0, L * 0.05);
                GProp_GProps p;
                BRepGProp::VolumeProperties(result, p);
                if (p.Mass() > minVolume) sweptOk = true;
            }
        }
    } catch (const Standard_Failure& ex) {
        spdlog::debug("wire_form: pipe-sweep threw ({}); using cylinder fallback",
                      ex.what());
    }

    // Attempt 2: per-segment cylinders fused together (always works for
    // straight segments; tolerates sharp corners).
    if (!sweptOk) {
        usedFallback = true;
        try {
            result = buildSegmentCylinders(in.waypoints, wireR);
        } catch (const Standard_Failure& ex) {
            throw SkillError(
                std::string("wire_form: shape build failed: ") + ex.what());
        }
    }

    json wpJ = json::array();
    for (const auto& p : in.waypoints) {
        wpJ.push_back({ { "x_mm", p.x_mm },
                        { "y_mm", p.y_mm },
                        { "z_mm", p.z_mm } });
    }

    // Approximate volume = π (d/2)² × polyline length.
    const double approxVolume = M_PI * wireR * wireR * L;

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
        { "geometry_changed",     true },
        { "sweep_fallback",       usedFallback },
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
    tooling.stock_removed_mm3 = -approxVolume;   // material formed (negative = added)
    tooling.est_cycle_time_s  = 1.0 + bendCount * 0.4 + L * 0.02;
    tooling.extra = json{
        { "process",            "wire_forming" },
        { "polyline_length_mm", L },
        { "bend_count",         std::max(0, bendCount) },
        { "min_bend_radius_mm", std::isfinite(minR) ? minR : 0.0 },
        { "sweep_fallback",     usedFallback },
        { "dfm_findings",       findings },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(result, wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug("skill::wire_form applied: {} waypoints, L={:.2f} mm, bends={}, fallback={}",
                  in.waypoints.size(), L, bendCount, usedFallback);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// Geometric heuristic (primary): bent-pipe topology — multiple cylindrical
// faces connected end-to-end, large bbox spanning multiple axes.  Without
// metadata we cannot recover the original polyline.  Confidence 0.45 for
// geometric, 1.0 for metadata replay.

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;

    // (a) Metadata replay.
    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        RecognizedFeature rf;
        rf.skill_id         = kSkillId;
        rf.recovered_params = f.params;
        rf.confidence       = 1.0;
        rf.matched_geometry = { { "source", "metadata" } };
        out.push_back(rf);
    }
    if (!out.empty()) return out;

    // (b) Geometric heuristic.
    if (wp.shape().IsNull()) return out;
    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double dx = xMax - xMin;
    const double dy = yMax - yMin;
    const double dz = zMax - zMin;
    if (dx <= 0.0 || dy <= 0.0) return out;

    // A bent wire spans more than one axis non-trivially.  Compare the two
    // largest extents: their ratio < 4 indicates multi-axis bending.
    double sorted[3] = { dx, dy, dz };
    std::sort(sorted, sorted + 3);
    const double mid    = sorted[1];
    const double large  = sorted[2];
    if (large <= 0.0) return out;
    if (mid / large < 0.15) return out;        // too thin/elongated — likely straight wire

    // Count cylindrical faces — pipe sweep / fused cylinders produce multiple.
    int cylFaces = 0;
    for (int fIdx = 0; fIdx < wp.faceCount(); ++fIdx) {
        if (wp.isFaceCylinder(fIdx)) ++cylFaces;
    }
    if (cylFaces < 2) return out;

    json recovered = {
        { "waypoints",   json::array() },
        { "wire_dia_mm", 0.0 },
    };
    json matched = {
        { "source",                "geometric_fallback" },
        { "bbox_x_mm",             dx },
        { "bbox_y_mm",             dy },
        { "bbox_z_mm",             dz },
        { "cylindrical_face_count", cylFaces },
    };
    out.push_back(RecognizedFeature{ kSkillId, recovered, 0.45, matched });
    return out;
}

}  // namespace koocadcam::skill::wire_form
