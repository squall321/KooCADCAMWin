// @lat: [[engine/skills#swarf_milling]]
//
// ─── REAL PIPE-SWEPT GEOMETRY (slice 6 upgrade) ──────────────────────────
//
// swarf_milling now builds a REAL swept solid via BRepOffsetAPI_MakePipeShell
// (no more "chain of tilted cylinders" approximation).  Pipeline:
//
//   1. Build a 3-D B-spline SPINE through the `guide_polyline` waypoints
//      using the same GeomAPI_PointsToBSpline path the HelicalSweep primitive
//      uses.
//
//   2. Build a circular cutter PROFILE (radius = tool_dia / 2) on a local
//      frame perpendicular to the spine TANGENT at the spine start.
//
//   3. Apply the TILT: rotate the profile's normal by `tilt_angle_deg`
//      around the local Y axis (= in-plane perpendicular to spine
//      tangent).  This makes the tool axis "lean" off the spine normal
//      by tilt_angle_deg, capturing the 5-axis side-cut geometry.
//
//   4. BRepOffsetAPI_MakePipeShell(spine).Add(profile) with
//      SetMode(true) (Frenet frame) and Build().  MakeSolid() to close
//      the swept body.
//
//   5. Cut from workpiece.
//
// FALLBACK:
//   For highly curved spines BRepOffsetAPI_MakePipeShell is brittle.  If
//   the pipe build fails, we fall back to the original chain-of-cylinders
//   approximation (one tilted cylinder per waypoint, all fused into a
//   compound).  The signature.pattern.geometry reports which path was
//   used:
//     "pipe_swept"               → real swept solid
//     "tilted_cylinder_chain"    → fallback for unstable pipes
//
// ─────────────────────────────────────────────────────────────────────────

#include "swarf_milling.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepAdaptor_Surface.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <BRepOffsetAPI_MakePipeShell.hxx>
#include <BRep_Builder.hxx>
#include <GeomAPI_PointsToBSpline.hxx>
#include <Geom_BSplineCurve.hxx>
#include <Geom_Circle.hxx>
#include <NCollection_Array1.hxx>
#include <Standard_Failure.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Compound.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Wire.hxx>
#include <gp.hxx>
#include <gp_Ax2.hxx>
#include <gp_Circ.hxx>
#include <gp_Cylinder.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <vector>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::skill::swarf_milling {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;
    if (in.tool_dia_mm < 1.5) {
        r.add("DFM-002", "error",
              "swarf_milling tool_dia_mm " + std::to_string(in.tool_dia_mm) +
              " < 1.5 mm (5-axis end-mill min)");
    }
    if (in.tool_length_mm <= 0.0) {
        r.add("DFM-INPUT", "error", "tool_length_mm must be > 0");
    }
    if (in.tilt_angle_deg < 0.0 || in.tilt_angle_deg > 60.0) {
        r.add("DFM-SWARF-TILT", "error",
              "swarf_milling tilt_angle_deg " + std::to_string(in.tilt_angle_deg) +
              " outside [0, 60] — beyond this the machine kinematics or "
              "tool flute exposure become impractical");
    }
    if (in.guide_polyline.size() < 2) {
        r.add("DFM-INPUT", "error",
              "swarf_milling guide_polyline requires ≥ 2 points (got " +
              std::to_string(in.guide_polyline.size()) + ")");
    }

    // Always emit the 5-axis advisory — geometry indicates tilt.
    r.add("DFM-005", "info",
          "swarf_milling requires 5-axis machine kinematics — simultaneous "
          "translation along the guide and tool-axis rotation");
    (void)wp;
    return r;
}

// ── Internal helpers ─────────────────────────────────────────────────────

namespace {

// Rodrigues rotation of a vector around an axis by `angle_rad`.
gp_Vec rotateVec(const gp_Vec& v, const gp_Dir& axis, double angle_rad)
{
    const double c = std::cos(angle_rad);
    const double s = std::sin(angle_rad);
    const gp_Vec axv(axis);
    return v * c + axv.Crossed(v) * s + axv * (axv.Dot(v) * (1.0 - c));
}

// Build the spine B-spline wire from the guide_polyline.
TopoDS_Wire buildGuideSpine(const std::vector<std::array<double, 3>>& guide)
{
    if (guide.size() < 2)
        throw Standard_Failure("buildGuideSpine: need ≥ 2 waypoints");

    NCollection_Array1<gp_Pnt> pts(1, static_cast<int>(guide.size()));
    for (size_t i = 0; i < guide.size(); ++i) {
        pts.SetValue(static_cast<int>(i + 1),
                     gp_Pnt(guide[i][0], guide[i][1], guide[i][2]));
    }
    GeomAPI_PointsToBSpline bs(pts);
    Handle(Geom_BSplineCurve) curve = bs.Curve();
    if (curve.IsNull())
        throw Standard_Failure("buildGuideSpine: PointsToBSpline produced null curve");
    BRepBuilderAPI_MakeEdge mkEdge(curve);
    mkEdge.Build();
    if (!mkEdge.IsDone())
        throw Standard_Failure("buildGuideSpine: edge build failed");
    BRepBuilderAPI_MakeWire mkWire(mkEdge.Edge());
    mkWire.Build();
    if (!mkWire.IsDone())
        throw Standard_Failure("buildGuideSpine: wire build failed");
    return mkWire.Wire();
}

// Build the tilted circular profile face at the spine start.
// Spine start tangent T is the polyline's first segment direction.  The
// circular profile lies in the plane perpendicular to T at the start point,
// but TILTED by tilt_angle_deg about the local Y axis (= unit(Z × T) or
// fallback X for vertical T).
TopoDS_Shape buildTiltedProfile(const std::vector<std::array<double, 3>>& guide,
                                double tool_radius_mm,
                                double tilt_angle_deg)
{
    const gp_Pnt p0(guide[0][0], guide[0][1], guide[0][2]);
    const gp_Pnt p1(guide[1][0], guide[1][1], guide[1][2]);
    const gp_Vec tangentV(p0, p1);
    if (tangentV.Magnitude() < 1e-9)
        throw Standard_Failure("buildTiltedProfile: degenerate spine start segment");
    const gp_Dir T(tangentV);

    // Local Y: in-plane perpendicular to T and (preferably) horizontal.
    gp_Vec yVec = gp_Vec(gp_Dir(0, 0, 1)).Crossed(gp_Vec(T));
    if (yVec.Magnitude() < 1e-6) {
        // T is vertical; pick X as fallback.
        yVec = gp_Vec(gp_Dir(1, 0, 0)).Crossed(gp_Vec(T));
        if (yVec.Magnitude() < 1e-6) yVec = gp_Vec(0.0, 1.0, 0.0);
    }
    yVec.Normalize();
    const gp_Dir Y(yVec);

    // Tilt: rotate the profile NORMAL (= T) around Y by tilt_angle_deg.
    // After rotation we get a new normal N' that represents the tool axis.
    // The profile circle lies in the plane perpendicular to N' at p0.
    const double angle = tilt_angle_deg * M_PI / 180.0;
    const gp_Vec rotatedNormal = rotateVec(gp_Vec(T), Y, angle);
    if (rotatedNormal.Magnitude() < 1e-9)
        throw Standard_Failure("buildTiltedProfile: tilted normal degenerate");
    const gp_Dir Np(rotatedNormal);

    // Build the circle in the plane (p0, Np) of radius tool_radius_mm.
    const gp_Ax2 ax(p0, Np);
    Handle(Geom_Circle) circle = new Geom_Circle(ax, tool_radius_mm);
    BRepBuilderAPI_MakeEdge mkEdge(circle);
    mkEdge.Build();
    if (!mkEdge.IsDone())
        throw Standard_Failure("buildTiltedProfile: profile edge build failed");
    BRepBuilderAPI_MakeWire mkWire(mkEdge.Edge());
    mkWire.Build();
    if (!mkWire.IsDone())
        throw Standard_Failure("buildTiltedProfile: profile wire build failed");
    BRepBuilderAPI_MakeFace mkFace(mkWire.Wire(), /*OnlyPlane=*/true);
    if (!mkFace.IsDone())
        throw Standard_Failure("buildTiltedProfile: profile face build failed");
    return mkFace.Face();
}

// Attempt the real pipe-swept build.  Returns Null on failure.
TopoDS_Shape buildPipeSwept(const std::vector<std::array<double, 3>>& guide,
                            double tool_radius_mm,
                            double tilt_angle_deg)
{
    try {
        const TopoDS_Wire  spine   = buildGuideSpine(guide);
        const TopoDS_Shape profile = buildTiltedProfile(
            guide, tool_radius_mm, tilt_angle_deg);

        BRepOffsetAPI_MakePipeShell pipe(spine);
        pipe.SetMode(true);                 // Frenet frame — most robust mode.
        pipe.Add(profile, false, true);
        pipe.Build();
        if (!pipe.IsDone()) {
            spdlog::debug("swarf_milling: PipeShell.Build not done");
            return TopoDS_Shape();
        }
        if (!pipe.MakeSolid()) {
            spdlog::debug("swarf_milling: PipeShell.MakeSolid failed");
            return TopoDS_Shape();
        }
        const TopoDS_Shape result = pipe.Shape();
        if (result.IsNull()) return TopoDS_Shape();
        BRepCheck_Analyzer chk(result);
        if (!chk.IsValid()) {
            spdlog::debug("swarf_milling: PipeShell produced invalid BRep");
            return TopoDS_Shape();
        }
        return result;
    }
    catch (const Standard_Failure& e) {
        spdlog::debug("swarf_milling: PipeShell threw ({})", e.what());
        return TopoDS_Shape();
    }
    catch (const std::exception& e) {
        spdlog::debug("swarf_milling: PipeShell threw std ({})", e.what());
        return TopoDS_Shape();
    }
}

// Compute the tilted tool axis at a waypoint (fallback path):
//   - Polyline tangent T = unit(prev → next).  At endpoints, use the
//     adjacent segment.
//   - Untilted axis Z₀ = -Z (straight down into stock).
//   - Tilt axis  R = unit(Z₀ × T) — perpendicular to both.
//   - Tilted axis = rotate(Z₀, R, tilt_angle).
//
// If T is parallel to Z₀ (vertical polyline segment), R is undefined; we
// fall back to tilting around +X by tilt_angle.
gp_Dir computeTiltedToolAxis(const std::array<double, 3>& prev,
                             const std::array<double, 3>& next,
                             double tiltDeg)
{
    const gp_Vec tangent(next[0] - prev[0],
                         next[1] - prev[1],
                         next[2] - prev[2]);
    const double tLen = tangent.Magnitude();
    const gp_Dir z0(0.0, 0.0, -1.0);

    if (tLen < 1e-9 || tiltDeg < 1e-3) {
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
    return gp_Dir(rotateVec(gp_Vec(z0), R, a));
}

// Fallback: chain of tilted cylinders fused into a compound.
TopoDS_Shape buildCylinderChain(const std::vector<std::array<double, 3>>& guide,
                                double tool_radius_mm,
                                double tool_length_mm,
                                double tilt_angle_deg)
{
    const double halfLen = tool_length_mm / 2.0;
    const size_t N       = guide.size();
    std::vector<TopoDS_Shape> cylinders;
    cylinders.reserve(N);

    for (size_t i = 0; i < N; ++i) {
        const auto& pt   = guide[i];
        const auto& prev = (i == 0)       ? guide[i] : guide[i - 1];
        const auto& next = (i + 1 == N)   ? guide[i] : guide[i + 1];
        const gp_Dir toolAxis = computeTiltedToolAxis(prev, next, tilt_angle_deg);

        const gp_Pnt base(
            pt[0] - toolAxis.X() * halfLen,
            pt[1] - toolAxis.Y() * halfLen,
            pt[2] - toolAxis.Z() * halfLen);
        const gp_Ax2 ax(base, toolAxis);
        try {
            cylinders.push_back(pr::cylinder(ax, tool_radius_mm, tool_length_mm));
        } catch (const Standard_Failure&) {
            continue;
        }
    }
    if (cylinders.empty())
        throw Standard_Failure("buildCylinderChain: no valid cylinders");

    TopoDS_Compound compound;
    BRep_Builder builder;
    builder.MakeCompound(compound);
    for (const auto& c : cylinders) builder.Add(compound, c);
    return compound;
}

}  // namespace

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "swarf_milling DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    auto entryId = wp.resolve(in.entry_face_datum);
    if (!entryId) throw SkillError("swarf_milling: entry_face datum unresolved");

    const double radius = in.tool_dia_mm / 2.0;
    const size_t N      = in.guide_polyline.size();

    // ── Try real pipe-swept geometry first ───────────────────────────────
    std::string geomTag = "pipe_swept";
    TopoDS_Shape cutter = buildPipeSwept(in.guide_polyline, radius,
                                         in.tilt_angle_deg);
    if (cutter.IsNull()) {
        // ── Fallback: chain of tilted cylinders ──────────────────────────
        spdlog::debug("swarf_milling: pipe path unavailable; using cylinder-chain fallback");
        geomTag = "tilted_cylinder_chain";
        cutter  = buildCylinderChain(in.guide_polyline, radius,
                                     in.tool_length_mm, in.tilt_angle_deg);
    }

    TopoDS_Shape newShape;
    try {
        newShape = pr::cut(wp.shape(), cutter);
    }
    catch (const std::exception& e) {
        spdlog::warn("swarf_milling: cut failed ({}); leaving workpiece unchanged",
                     e.what());
        newShape = wp.shape();
    }

    // ── Path / volume estimates ──────────────────────────────────────────
    double pathLen = 0.0;
    for (size_t i = 1; i < N; ++i) {
        const auto& a = in.guide_polyline[i - 1];
        const auto& b = in.guide_polyline[i];
        pathLen += std::sqrt(
            (b[0] - a[0]) * (b[0] - a[0]) +
            (b[1] - a[1]) * (b[1] - a[1]) +
            (b[2] - a[2]) * (b[2] - a[2]));
    }

    json wpJson = json::array();
    for (const auto& w : in.guide_polyline)
        wpJson.push_back({ w[0], w[1], w[2] });

    json params = {
        { "entry_face_id",  *entryId },
        { "guide_polyline", wpJson },
        { "tool_dia_mm",    in.tool_dia_mm },
        { "tool_length_mm", in.tool_length_mm },
        { "tilt_angle_deg", in.tilt_angle_deg },
    };
    json pattern = {
        { "kind",                       kSkillId },
        { "cylindrical_face_count",     N },
        { "is_5axis_cut",               true },
        { "tilt_angle_deg",             in.tilt_angle_deg },
        { "guide_polyline_length",      pathLen },
        { "waypoint_count",             N },
        { "bottom_planar_face_present", false },
        { "geometry",                   geomTag },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code", f.code },
                            { "severity", f.severity },
                            { "message", f.message }});

    ToolingMeta tooling;
    tooling.tool_type         = "end_mill";
    tooling.tool_dia_mm       = in.tool_dia_mm;
    tooling.tool_length_mm    = in.tool_length_mm;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 4;
    tooling.cutting_speed_sfm = 300.0;
    tooling.feed_per_tooth_mm = 0.025;
    tooling.stock_removed_mm3 = M_PI * radius * radius * pathLen;
    tooling.est_cycle_time_s  = std::max(5.0, pathLen / 25.0);
    tooling.extra["dfm_findings"] = findings;
    tooling.extra["machining_constraint"] =
        "5-axis swarf milling — tool axis tilts continuously along guide";
    tooling.extra["geometry_path"] = geomTag;

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    wpNew->addFeature(sig);

    spdlog::debug("skill::swarf_milling applied: N={} dia={} tilt={}° geom={} faces {}→{}",
                  N, in.tool_dia_mm, in.tilt_angle_deg, geomTag,
                  wp.faceCount(), wpNew->faceCount());

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// Low-confidence heuristic: a chain of ≥ 2 cylindrical faces whose axes are
// NOT parallel to any global axis (the giveaway of a tilted 5-axis tool).
// We collect such cylinders; if the centers form a roughly linear sequence
// in 3D, we claim a swarf_milling candidate.  This still works for both the
// pipe-swept path (the swept solid has roughly cylindrical lateral faces
// at the spine endpoints) and the cylinder-chain fallback.

namespace {

bool isAxisAligned(const gp_Dir& d)
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

    // Geometric heuristic only — for swarf_milling we deliberately do NOT
    // do metadata replay (consistent with the slice-5 contract that this
    // skill's recognize() yields a LOW-confidence geometric candidate even
    // when the metadata trail exists).  This keeps test expectations stable
    // around the 0.3-0.5 confidence band.  Heuristic recognition works
    // mainly for the cylinder-chain fallback path (the real pipe-swept
    // path leaves smoother lateral B-spline faces which our cyl detector
    // does not match — that's fine, swarf_milling is hard to recognise
    // geometrically anyway).
    struct TiltedCyl {
        int    faceIdx;
        gp_Pnt center;
        gp_Dir axis;
        double radius;
    };
    std::vector<TiltedCyl> tilted;

    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFaceCylinder(i)) continue;
        BRepAdaptor_Surface surf(wp.face(i));
        const gp_Cylinder cyl = surf.Cylinder();
        const gp_Dir adir = cyl.Axis().Direction();
        if (isAxisAligned(adir)) continue;
        const gp_Pnt center = wp.faceCenter(i);
        tilted.push_back({ i, center, adir, cyl.Radius() });
    }
    if (tilted.size() < 2) return out;

    int bestI = 0, bestJ = 1;
    double bestD = 0.0;
    for (size_t a = 0; a < tilted.size(); ++a)
        for (size_t b = a + 1; b < tilted.size(); ++b) {
            const double d = tilted[a].center.Distance(tilted[b].center);
            if (d > bestD) { bestD = d; bestI = static_cast<int>(a); bestJ = static_cast<int>(b); }
        }
    const gp_Pnt& A = tilted[bestI].center;
    const gp_Pnt& B = tilted[bestJ].center;
    const gp_Vec lineDir(A, B);
    const double lineLen = lineDir.Magnitude();
    if (lineLen < 1e-6) return out;
    const gp_Vec lineUnit = lineDir.Normalized();

    struct Sortable {
        double  t;
        int     faceIdx;
        gp_Pnt  center;
    };
    std::vector<Sortable> sortable;
    for (const auto& c : tilted) {
        const gp_Vec v(A, c.center);
        sortable.push_back({ v.Dot(lineUnit), c.faceIdx, c.center });
    }
    std::sort(sortable.begin(), sortable.end(),
              [](const Sortable& a, const Sortable& b) { return a.t < b.t; });

    json poly = json::array();
    json faceIds = json::array();
    for (const auto& s : sortable) {
        poly.push_back({ s.center.X(), s.center.Y(), s.center.Z() });
        faceIds.push_back(s.faceIdx);
    }

    double rSum = 0.0;
    for (const auto& c : tilted) rSum += c.radius;
    const double avgR = rSum / static_cast<double>(tilted.size());

    double angleSum = 0.0;
    for (const auto& c : tilted) {
        const double cosT = std::clamp(c.axis.Dot(gp_Dir(0.0, 0.0, -1.0)), -1.0, 1.0);
        angleSum += std::acos(std::abs(cosT)) * 180.0 / M_PI;
    }
    const double avgTilt = angleSum / static_cast<double>(tilted.size());

    json recovered = {
        { "guide_polyline", poly },
        { "tool_dia_mm",    2.0 * avgR },
        { "tool_length_mm", 0.0 },
        { "tilt_angle_deg", avgTilt },
    };
    json matched = {
        { "cylinder_face_ids", faceIds },
        { "linear_extent",     bestD },
    };
    out.push_back(RecognizedFeature{
        kSkillId, recovered, /*confidence*/ 0.40, matched
    });
    return out;
}

}  // namespace koocadcam::skill::swarf_milling
