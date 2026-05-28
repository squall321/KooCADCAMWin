#pragma once
// @lat: [[engine/skills#separation cutting — shared helpers]]
//
// **Shared internal helpers, not a public primitive.**
//
// Used ONLY by the five SEPARATION CUTTING skills (saw_cut, waterjet_cut,
// laser_cut, plasma_cut, oxyfuel_cut).  They all share the same input shape
// (a 2D path: linear / circular / polyline) and the same through-channel
// synthesis topology — only the tooling metadata differs (kerf width,
// max thickness, surface finish).
//
// Anyone tempted to reach for these helpers from a different skill family
// should instead consider promoting the relevant function into
// `engine/primitives/` as a public primitive.  These helpers are NOT
// stabilised contracts.
//
// Provided here:
//   - `LinearCutInput`         — shared Input struct (kind = Linear/Circular/Polyline)
//   - `buildLinearCutter()`    — linear stadium prism (start→end, width = kerf)
//   - `buildCircularCutter()`  — annular arc sector swept vertically
//   - `buildSeparationCutter()`— dispatches on LinearCutInput::Kind
//   - `recognizeThroughChannels()` — finds through-channels in a workpiece
//                                    and returns confidence-weighted
//                                    candidates per skill_id
//
// Topology produced by all three Kinds: a through-channel (no bottom face),
// straight vertical walls from top to bottom of the workpiece, kerf width
// along the path.

#include "Datum.hpp"
#include "Skill.hpp"
#include "Workpiece.hpp"

#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/PolylineOffset.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepAdaptor_Curve.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepPrimAPI_MakePrism.hxx>
#include <Geom_Circle.hxx>
#include <NCollection_IndexedDataMap.hxx>
#include <NCollection_List.hxx>
#include <Standard_Failure.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopTools_ShapeMapHasher.hxx>
#include <TopoDS.hxx>
#include <gp.hxx>
#include <gp_Ax2.hxx>
#include <gp_Cylinder.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>

#include <nlohmann/json.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <set>
#include <string>
#include <vector>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::skill::separation_common {

namespace pr = koocadcam::engine::prim;

// ── Shared input struct ──────────────────────────────────────────────────
//
// All five separation-cutting skills take this same input shape.  They differ
// only in tooling metadata (which is set inside each skill's apply()).
struct LinearCutInput
{
    FaceDatum  entry_face;

    enum class Kind { Linear, Circular, Polyline };
    Kind       kind = Kind::Linear;

    // Linear: 2 endpoints in XY (z taken from stock top)
    double     start_x_mm       = 0.0;
    double     start_y_mm       = 0.0;
    double     end_x_mm         = 0.0;
    double     end_y_mm         = 0.0;

    // Circular: center + radius + start/end angles (degrees, CCW from +X)
    double     cx_mm            = 0.0;
    double     cy_mm            = 0.0;
    double     radius_mm        = 0.0;
    double     start_angle_deg  = 0.0;
    double     end_angle_deg    = 0.0;

    // Polyline: ordered list of XY waypoints (≥ 2)
    std::vector<std::array<double, 2>> waypoints;

    // 0 = through the full thickness of the stock (standard separation cut);
    // any > 0 partial-depth value is honored.
    double     cut_through_depth_mm = 0.0;
};

// ── Linear stadium cutter (mill_slot style) ─────────────────────────────
//
// Build (2 cyl + 1 box, fused) of width = kerf along start→end at
// z = base_z extruded downward by height.  Origin = (sx, sy, base_z).
inline TopoDS_Shape buildLinearCutter(double sx, double sy,
                                      double ex, double ey,
                                      double kerf, double base_z,
                                      double height)
{
    const double r       = kerf / 2.0;
    const gp_Dir downZ(0.0, 0.0, -1.0);

    const gp_Ax2 axStart(gp_Pnt(sx, sy, base_z), downZ);
    const gp_Ax2 axEnd  (gp_Pnt(ex, ey, base_z), downZ);
    const TopoDS_Shape cylStart = pr::cylinder(axStart, r, height);
    const TopoDS_Shape cylEnd   = pr::cylinder(axEnd,   r, height);

    const gp_Vec dir2D(ex - sx, ey - sy, 0.0);
    const double L = dir2D.Magnitude();
    if (L < 1e-9) {
        // Degenerate (start == end) — return the single cylinder.
        return cylStart;
    }
    const gp_Dir xDir(dir2D.X() / L, dir2D.Y() / L, 0.0);
    // OCCT box frame: gp_Ax2(P, mainDir=+Z, xDir) → yDir = +Z × xDir =
    // (-xDir.Y, xDir.X, 0).  DY extends along OCCT's yDir, so to centre the
    // box on the start→end line we shift the origin by -r along OCCT's yDir.
    const gp_Dir yDir(-xDir.Y(), xDir.X(), 0.0);

    const gp_Pnt boxOrigin(
        sx - r * yDir.X(),
        sy - r * yDir.Y(),
        base_z - height);
    const gp_Ax2 boxAx(boxOrigin, gp_Dir(0.0, 0.0, 1.0), xDir);
    const TopoDS_Shape connBox = pr::box(boxAx, L, kerf, height);

    return pr::fuseMany(cylStart, { cylEnd, connBox });
}

// ── Circular annular-arc cutter ──────────────────────────────────────────
//
// Build a wire enclosing the annular sector (R − kerf/2 → R + kerf/2) over
// the angle range [a0, a1] (radians), then a planar face from it, then
// prism-extrude downward by `height` from `base_z`.
//
// OCCT 8.0:
//   `Geom_Circle(gp_Ax2, radius)` defines the parametrised circle (param =
//   angle in radians CCW around axis Z).  `BRepBuilderAPI_MakeEdge(circ,
//   p1, p2)` builds the arc edge from start param to end param.
inline TopoDS_Shape buildCircularCutter(double cx, double cy,
                                        double radius_mm, double kerf,
                                        double start_angle_deg,
                                        double end_angle_deg,
                                        double base_z, double height)
{
    if (radius_mm <= kerf / 2.0 + 1e-9)
        throw Standard_Failure("buildCircularCutter: radius_mm must be > kerf/2");

    double a0 = start_angle_deg * M_PI / 180.0;
    double a1 = end_angle_deg   * M_PI / 180.0;
    if (std::abs(a1 - a0) < 1e-9)
        throw Standard_Failure("buildCircularCutter: sweep angle must be > 0");
    // OCCT's BRepBuilderAPI_MakeEdge(Geom_Circle, p1, p2) expects p1 < p2.
    // If the caller supplied end < start, swap (we only need the geometric
    // sector, not a directed sweep).
    if (a1 < a0) std::swap(a0, a1);

    const double rOuter = radius_mm + kerf / 2.0;
    const double rInner = radius_mm - kerf / 2.0;

    const gp_Pnt cZ0(cx, cy, base_z);
    const gp_Ax2 ax(cZ0, gp_Dir(0.0, 0.0, 1.0), gp_Dir(1.0, 0.0, 0.0));

    Handle(Geom_Circle) outerCirc = new Geom_Circle(ax, rOuter);
    Handle(Geom_Circle) innerCirc = new Geom_Circle(ax, rInner);

    // 4 corners of the annular sector (XY at z = base_z):
    auto cornerPt = [&](double r, double a) {
        return gp_Pnt(cx + r * std::cos(a), cy + r * std::sin(a), base_z);
    };
    const gp_Pnt pOuterStart = cornerPt(rOuter, a0);
    const gp_Pnt pOuterEnd   = cornerPt(rOuter, a1);
    const gp_Pnt pInnerStart = cornerPt(rInner, a0);
    const gp_Pnt pInnerEnd   = cornerPt(rInner, a1);

    // Wire (CCW around the sector boundary):
    //   outer-arc (a0 → a1) → radial-in (pOuterEnd → pInnerEnd)
    //   → inner-arc reversed (pInnerEnd → pInnerStart) → radial-out (pInnerStart → pOuterStart)
    //
    // OCCT pitfall (BT-arc-orientation): MakeEdge(Geom_Circle, p1, p2) with
    // p2 < p1 does NOT produce a reversed edge — OCCT internally normalises
    // the parameter range so the resulting edge still traverses CCW from
    // min(p1,p2) to max(p1,p2).  Pass parameters in CCW order (a0, a1) and
    // explicitly flip the edge orientation with TopoDS_Edge::Reversed() to
    // get the desired CW traversal for the inner arc.
    BRepBuilderAPI_MakeEdge eOuter(outerCirc, a0, a1);
    BRepBuilderAPI_MakeEdge eRadIn(pOuterEnd, pInnerEnd);
    BRepBuilderAPI_MakeEdge eInnerFwd(innerCirc, a0, a1);  // CCW
    BRepBuilderAPI_MakeEdge eRadOut(pInnerStart, pOuterStart);
    if (!eOuter.IsDone() || !eRadIn.IsDone() || !eInnerFwd.IsDone() || !eRadOut.IsDone())
        throw Standard_Failure("buildCircularCutter: arc edge build failed");

    // Flip inner arc to traverse CW (pInnerEnd → pInnerStart) to close the
    // sector boundary in a consistent direction.
    const TopoDS_Edge innerEdgeReversed = TopoDS::Edge(eInnerFwd.Edge().Reversed());

    BRepBuilderAPI_MakeWire wireMaker;
    wireMaker.Add(eOuter.Edge());
    wireMaker.Add(eRadIn.Edge());
    wireMaker.Add(innerEdgeReversed);
    wireMaker.Add(eRadOut.Edge());
    if (!wireMaker.IsDone())
        throw Standard_Failure("buildCircularCutter: wire build failed");

    BRepBuilderAPI_MakeFace faceMaker(wireMaker.Wire(), /*OnlyPlane=*/true);
    if (!faceMaker.IsDone())
        throw Standard_Failure("buildCircularCutter: face build failed");

    const gp_Vec extrude(0.0, 0.0, -height);
    BRepPrimAPI_MakePrism prism(faceMaker.Face(), extrude);
    prism.Build();
    if (!prism.IsDone())
        throw Standard_Failure("buildCircularCutter: prism failed");
    return prism.Shape();
}

// ── Dispatch on Kind ────────────────────────────────────────────────────
//
// `wpBboxMax`, `wpBboxMin` are the workpiece's Z extents (typically zMin,
// zMax).  `kerf_mm` is the per-skill kerf width.  `cut_through_depth_mm`:
//   ≤ 0  → through-cut (extends 2 mm beyond top + bottom for clean booleans)
//   > 0  → partial depth from top
inline TopoDS_Shape buildSeparationCutter(const LinearCutInput& in,
                                          double kerf_mm,
                                          double wpZMin, double wpZMax)
{
    constexpr double kOverhang = 1.0;
    const double zTop  = wpZMax + kOverhang;
    const double through = (wpZMax - wpZMin) + 2.0 * kOverhang;
    const double height = (in.cut_through_depth_mm > 0.0)
                          ? in.cut_through_depth_mm + kOverhang
                          : through;
    using K = LinearCutInput::Kind;
    switch (in.kind) {
    case K::Linear:
        return buildLinearCutter(in.start_x_mm, in.start_y_mm,
                                 in.end_x_mm,   in.end_y_mm,
                                 kerf_mm, zTop, height);
    case K::Circular:
        return buildCircularCutter(in.cx_mm, in.cy_mm,
                                   in.radius_mm, kerf_mm,
                                   in.start_angle_deg, in.end_angle_deg,
                                   zTop, height);
    case K::Polyline:
        return pr::offsetPolylineCutter(in.waypoints, kerf_mm, height, zTop);
    }
    throw Standard_Failure("buildSeparationCutter: unknown Kind");
}

// ── Path length helper (for stock-removal & cycle time) ──────────────────
inline double pathLength(const LinearCutInput& in)
{
    using K = LinearCutInput::Kind;
    switch (in.kind) {
    case K::Linear:
        return std::hypot(in.end_x_mm - in.start_x_mm,
                          in.end_y_mm - in.start_y_mm);
    case K::Circular: {
        const double da = std::abs(
            (in.end_angle_deg - in.start_angle_deg) * M_PI / 180.0);
        return in.radius_mm * da;
    }
    case K::Polyline: {
        double s = 0.0;
        for (size_t i = 1; i < in.waypoints.size(); ++i) {
            s += std::hypot(in.waypoints[i][0] - in.waypoints[i-1][0],
                            in.waypoints[i][1] - in.waypoints[i-1][1]);
        }
        return s;
    }
    }
    return 0.0;
}

inline const char* kindToString(LinearCutInput::Kind k)
{
    switch (k) {
    case LinearCutInput::Kind::Linear:   return "Linear";
    case LinearCutInput::Kind::Circular: return "Circular";
    case LinearCutInput::Kind::Polyline: return "Polyline";
    }
    return "Unknown";
}

// ── Shared kind→params JSON serializer ──────────────────────────────────
inline nlohmann::json kindParamsToJson(const LinearCutInput& in)
{
    nlohmann::json p = nlohmann::json::object();
    p["cut_kind"] = kindToString(in.kind);
    p["cut_through_depth_mm"] = in.cut_through_depth_mm;
    using K = LinearCutInput::Kind;
    switch (in.kind) {
    case K::Linear:
        p["start_x_mm"] = in.start_x_mm;
        p["start_y_mm"] = in.start_y_mm;
        p["end_x_mm"]   = in.end_x_mm;
        p["end_y_mm"]   = in.end_y_mm;
        break;
    case K::Circular:
        p["cx_mm"] = in.cx_mm;
        p["cy_mm"] = in.cy_mm;
        p["radius_mm"] = in.radius_mm;
        p["start_angle_deg"] = in.start_angle_deg;
        p["end_angle_deg"]   = in.end_angle_deg;
        break;
    case K::Polyline: {
        nlohmann::json wpJson = nlohmann::json::array();
        for (const auto& w : in.waypoints) wpJson.push_back({ w[0], w[1] });
        p["waypoints"] = wpJson;
        break;
    }
    }
    return p;
}

// ── Shared DFM gating ───────────────────────────────────────────────────
//
// Per-skill process limits passed in.  Adds:
//   DFM-002       error if kerf below process minimum
//   DFM-THICKNESS error if workpiece thickness exceeds max
//   DFM-SURFACE   info-level note about expected finish
//   DFM-INPUT     error if the path is geometrically degenerate
struct ProcessLimits
{
    std::string skill_id;
    double      kerf_min_mm;
    double      kerf_max_mm;
    double      max_thickness_mm;
    std::string surface_finish;
    std::string tool_type;
    double      process_kerf_mm;  // representative kerf used by this skill
};

inline DFMReport validateCommon(const Workpiece& wp,
                                const LinearCutInput& in,
                                const ProcessLimits& lim)
{
    DFMReport r;

    // Path geometry validation.
    using K = LinearCutInput::Kind;
    switch (in.kind) {
    case K::Linear: {
        const double L = std::hypot(in.end_x_mm - in.start_x_mm,
                                    in.end_y_mm - in.start_y_mm);
        if (L < 1e-6)
            r.add("DFM-INPUT", "error",
                  lim.skill_id + ": linear start and end must differ (path length > 0)");
        break;
    }
    case K::Circular: {
        if (in.radius_mm <= lim.process_kerf_mm / 2.0)
            r.add("DFM-INPUT", "error",
                  lim.skill_id + ": circular radius_mm <= kerf/2 (degenerate annulus)");
        if (std::abs(in.end_angle_deg - in.start_angle_deg) < 1e-6)
            r.add("DFM-INPUT", "error",
                  lim.skill_id + ": circular sweep angle must be > 0");
        break;
    }
    case K::Polyline: {
        if (in.waypoints.size() < 2)
            r.add("DFM-INPUT", "error",
                  lim.skill_id + ": polyline needs ≥ 2 waypoints (got " +
                  std::to_string(in.waypoints.size()) + ")");
        for (size_t i = 1; i < in.waypoints.size(); ++i) {
            const double dx = in.waypoints[i][0] - in.waypoints[i-1][0];
            const double dy = in.waypoints[i][1] - in.waypoints[i-1][1];
            if (std::hypot(dx, dy) < 1e-6) {
                r.add("DFM-INPUT", "error",
                      lim.skill_id + ": duplicate consecutive waypoints at index " +
                      std::to_string(i));
                break;
            }
        }
        break;
    }
    }

    // Kerf bounds (DFM-002).  The skill's process_kerf is fixed; this rule
    // helps catch "wrong process selected" — e.g. trying to laser-cut a
    // 5 mm channel.  We treat it as informational borderline if outside.
    if (lim.process_kerf_mm < lim.kerf_min_mm) {
        r.add("DFM-002", "error",
              lim.skill_id + ": kerf " + std::to_string(lim.process_kerf_mm) +
              " mm < process min " + std::to_string(lim.kerf_min_mm) + " mm");
    } else if (lim.process_kerf_mm > lim.kerf_max_mm) {
        r.add("DFM-002", "warning",
              lim.skill_id + ": kerf " + std::to_string(lim.process_kerf_mm) +
              " mm > process typical max " + std::to_string(lim.kerf_max_mm) + " mm");
    }

    // Thickness bound — workpiece Z extent vs. max process thickness.
    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double thickness = zMax - zMin;
    if (thickness > lim.max_thickness_mm) {
        r.add("DFM-THICKNESS", "error",
              lim.skill_id + ": workpiece thickness " + std::to_string(thickness) +
              " mm > process max " + std::to_string(lim.max_thickness_mm) + " mm");
    } else if (thickness > 0.9 * lim.max_thickness_mm) {
        r.add("DFM-THICKNESS", "warning",
              lim.skill_id + ": workpiece thickness " + std::to_string(thickness) +
              " mm > 90% of process max — borderline");
    }

    // Surface finish — informational only.
    r.add("DFM-SURFACE", "info",
          lim.skill_id + ": expected cut-edge finish = " + lim.surface_finish);

    return r;
}

// ── Shared signature builder ────────────────────────────────────────────
//
// Builds the FeatureSignature for a separation skill from the input + the
// process limits.  Per-skill apply() supplements with cycle-time and
// stock-removal volume.
inline FeatureSignature buildSignature(const LinearCutInput& in,
                                       const ProcessLimits& lim,
                                       int entry_face_id,
                                       double workpiece_thickness,
                                       const DFMReport& dfm)
{
    using nlohmann::json;

    json params = kindParamsToJson(in);
    params["entry_face_id"] = entry_face_id;

    json pattern = {
        { "kind",                    lim.skill_id },
        { "cut_kind",                kindToString(in.kind) },
        { "kerf_mm",                 lim.process_kerf_mm },
        { "tool_type",               lim.tool_type },
        { "surface_finish",          lim.surface_finish },
        { "is_through_cut",          in.cut_through_depth_mm <= 0.0 },
        { "cut_through_depth_mm",    in.cut_through_depth_mm },
    };

    const double pathLen = pathLength(in);
    pattern["path_length_mm"] = pathLen;

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code", f.code },
                            { "severity", f.severity },
                            { "message", f.message }});

    ToolingMeta tooling;
    tooling.tool_type         = lim.tool_type;
    tooling.tool_dia_mm       = lim.process_kerf_mm;
    tooling.tool_length_mm    = workpiece_thickness;
    tooling.tool_material     = "n/a";   // thermal / hydraulic, not mechanical
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    // Stock removed ≈ pathLength × kerf × min(depth, thickness)
    const double effDepth = (in.cut_through_depth_mm > 0.0)
        ? std::min(in.cut_through_depth_mm, workpiece_thickness)
        : workpiece_thickness;
    tooling.stock_removed_mm3 = pathLen * lim.process_kerf_mm * effDepth;
    // Per-process feed rates wildly differ; per-skill apply() overrides.
    tooling.est_cycle_time_s  = std::max(1.0, pathLen / 50.0);
    tooling.extra["dfm_findings"]      = findings;
    tooling.extra["surface_finish"]    = lim.surface_finish;
    tooling.extra["process_category"]  = "separation_cutting";

    FeatureSignature sig{ lim.skill_id, params, pattern, tooling };
    return sig;
}

// ── Recognize: through-channel scanner ──────────────────────────────────
//
// Scan workpiece for vertical through-channels: a connected group of
// planar/cylindrical faces whose normals are horizontal and which span the
// full Z extent (touching top + bottom faces).
//
// Heuristic: for each face whose normal is mostly horizontal, check whether
// it shares edges with both a top (z = zMax) and a bottom (z = zMin) planar
// face.  Estimate the channel kerf from the face's tangential extent (for
// a cylindrical face = 2 × radius; for a planar wall in a pair = the
// distance between the pair).  We classify channel candidates by kerf
// bucket and emit per-skill RecognizedFeature entries with the user-specified
// confidence weighting.
//
// Returns a single skill's candidates filtered by `target_skill_id`.
struct ChannelCandidate
{
    double      kerf_mm    = 0.0;
    double      path_length_mm = 0.0;
    std::string cut_kind   = "Polyline";
    nlohmann::json wall_face_ids;
};

namespace detail {

using EdgeFaceMap = NCollection_IndexedDataMap<
    TopoDS_Shape, NCollection_List<TopoDS_Shape>, TopTools_ShapeMapHasher>;

inline EdgeFaceMap buildEdgeFaceMap(const TopoDS_Shape& shape)
{
    EdgeFaceMap m;
    TopExp::MapShapesAndAncestors(shape, TopAbs_EDGE, TopAbs_FACE, m);
    return m;
}

}  // namespace detail

inline std::vector<ChannelCandidate> scanThroughChannels(const Workpiece& wp)
{
    std::vector<ChannelCandidate> out;
    const auto edgeFaces = detail::buildEdgeFaceMap(wp.shape());

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double bot = zMin, top = zMax;

    // Identify top + bottom horizontal faces.
    std::set<int> topFaces, botFaces;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFacePlanar(i)) continue;
        gp_Dir n;
        try { n = wp.faceNormal(i); } catch (...) { continue; }
        if (std::abs(n.Z()) < 0.95) continue;
        const gp_Pnt c = wp.faceCenter(i);
        if (std::abs(c.Z() - top) < 1e-2) topFaces.insert(i);
        if (std::abs(c.Z() - bot) < 1e-2) botFaces.insert(i);
    }

    // Find vertical-walled faces (planar normal horizontal, or cylindrical
    // with axis vertical) that share edges with BOTH top and bottom.
    std::vector<int> wallIds;
    for (int i = 0; i < wp.faceCount(); ++i) {
        bool eligible = false;
        if (wp.isFacePlanar(i)) {
            gp_Dir n;
            try { n = wp.faceNormal(i); } catch (...) { continue; }
            if (std::abs(n.Z()) < 0.02) eligible = true;
        } else if (wp.isFaceCylinder(i)) {
            BRepAdaptor_Surface surf(wp.face(i));
            const gp_Cylinder cyl = surf.Cylinder();
            const gp_Dir adir = cyl.Axis().Direction();
            if (std::abs(std::abs(adir.Z()) - 1.0) > 1e-2) continue;
            eligible = true;
        }
        if (!eligible) continue;

        bool touchesTop = false, touchesBot = false;
        for (TopExp_Explorer exp(wp.face(i), TopAbs_EDGE); exp.More(); exp.Next()) {
            const TopoDS_Edge& e = TopoDS::Edge(exp.Current());
            if (!edgeFaces.Contains(e)) continue;
            const auto& adj = edgeFaces.FindFromKey(e);
            for (NCollection_List<TopoDS_Shape>::Iterator it(adj); it.More(); it.Next()) {
                const TopoDS_Face& af = TopoDS::Face(it.Value());
                if (af.IsSame(wp.face(i))) continue;
                for (int t : topFaces)
                    if (wp.face(t).IsSame(af)) { touchesTop = true; break; }
                for (int b : botFaces)
                    if (wp.face(b).IsSame(af)) { touchesBot = true; break; }
            }
        }
        if (touchesTop && touchesBot) wallIds.push_back(i);
    }

    if (wallIds.empty()) return out;

    // Group walls by approximate kerf.  Simple heuristic for slice 1:
    //   - any vertical cylindrical face → kerf = 2 × radius, kind = "Circular"
    //   - planar walls in pairs (parallel, opposite normals) → kerf = pair
    //     distance, kind = "Linear" / "Polyline"
    //
    // For slice 1 we emit ONE coarse candidate per cylinder + one combined
    // candidate over all planar walls (treating their footprint as a closed
    // path; the kerf is the median of pair distances).

    // Cylindrical walls: group by axis (x, y) — concentric pairs indicate an
    // annular-sector cut (circular cut kind) whose kerf = R_outer − R_inner.
    // Lone cylinders (no concentric partner) are end-caps of a linear / poly
    // cut whose kerf = 2 × radius.
    struct CylInfo { int faceId; double r; double cx; double cy; };
    std::vector<CylInfo> cylWalls;
    for (int wId : wallIds) {
        if (!wp.isFaceCylinder(wId)) continue;
        BRepAdaptor_Surface surf(wp.face(wId));
        const gp_Cylinder cyl = surf.Cylinder();
        const gp_Pnt loc = cyl.Axis().Location();
        cylWalls.push_back({ wId, cyl.Radius(), loc.X(), loc.Y() });
    }
    std::vector<bool> paired(cylWalls.size(), false);
    for (size_t a = 0; a < cylWalls.size(); ++a) {
        if (paired[a]) continue;
        for (size_t b = a + 1; b < cylWalls.size(); ++b) {
            if (paired[b]) continue;
            const double dCenter = std::hypot(cylWalls[a].cx - cylWalls[b].cx,
                                              cylWalls[a].cy - cylWalls[b].cy);
            if (dCenter > 1e-3) continue;   // not concentric
            // Concentric pair → annular sector cut.
            const double kerf = std::abs(cylWalls[a].r - cylWalls[b].r);
            const double rMid = (cylWalls[a].r + cylWalls[b].r) / 2.0;
            ChannelCandidate cc;
            cc.kerf_mm        = kerf;
            cc.path_length_mm = 2.0 * M_PI * rMid;   // full circle bound
            cc.cut_kind       = "Circular";
            cc.wall_face_ids  = nlohmann::json::array(
                { cylWalls[a].faceId, cylWalls[b].faceId });
            out.push_back(cc);
            paired[a] = paired[b] = true;
            break;
        }
    }
    // Lone cylinders → linear-cut end-caps.
    for (size_t i = 0; i < cylWalls.size(); ++i) {
        if (paired[i]) continue;
        ChannelCandidate cc;
        cc.kerf_mm        = 2.0 * cylWalls[i].r;
        cc.path_length_mm = 2.0 * M_PI * cylWalls[i].r;
        cc.cut_kind       = "Linear";
        cc.wall_face_ids  = nlohmann::json::array({ cylWalls[i].faceId });
        out.push_back(cc);
    }

    // Combined planar candidate (best-effort kerf inference).
    std::vector<int> planarWalls;
    for (int w : wallIds)
        if (wp.isFacePlanar(w)) planarWalls.push_back(w);

    if (planarWalls.size() >= 2) {
        // Compute parallel-pair distances and use median as kerf estimate.
        std::vector<double> dists;
        for (size_t a = 0; a < planarWalls.size(); ++a) {
            const int ia = planarWalls[a];
            gp_Dir na;
            try { na = wp.faceNormal(ia); } catch (...) { continue; }
            const gp_Pnt pa = wp.faceCenter(ia);
            for (size_t b = a + 1; b < planarWalls.size(); ++b) {
                const int ib = planarWalls[b];
                gp_Dir nb;
                try { nb = wp.faceNormal(ib); } catch (...) { continue; }
                // Parallel & opposite?
                if (std::abs(na.Dot(nb) + 1.0) > 0.05) continue;
                const gp_Pnt pb = wp.faceCenter(ib);
                const double d = std::abs((pb.X() - pa.X()) * na.X()
                                          + (pb.Y() - pa.Y()) * na.Y()
                                          + (pb.Z() - pa.Z()) * na.Z());
                if (d > 1e-6) dists.push_back(d);
            }
        }
        if (!dists.empty()) {
            std::sort(dists.begin(), dists.end());
            const double kerfEst = dists[dists.size() / 2];   // median

            ChannelCandidate cc;
            cc.kerf_mm        = kerfEst;
            cc.path_length_mm = 0.0;
            cc.cut_kind       = (planarWalls.size() == 2) ? "Linear" : "Polyline";
            cc.wall_face_ids  = nlohmann::json(planarWalls);
            out.push_back(cc);
        }
    }
    return out;
}

// Confidence table per the design spec.  For a given (kerf, target_skill_id)
// return the recognise confidence in [0, 1] (0 = "don't emit").
//
// Bands (mutually exclusive — else-if chain so each kerf falls into exactly
// one band and the function returns deterministically):
//   kerf < 0.5  → laser primary (0.70), waterjet secondary (0.50)
//   0.5 ≤ kerf < 1.2 → waterjet primary (0.70), laser fallback (0.30)
//   1.2 ≤ kerf < 3.0 → saw primary (0.60), waterjet (0.50), plasma (0.30)
//   3.0 ≤ kerf ≤ 6.0 → plasma primary (0.60), saw (0.40)
//   kerf > 6.0 → oxyfuel primary (0.70), plasma (0.30)
inline double confidenceForKerf(double kerf_mm, const std::string& skill)
{
    // kerf < 0.5
    if (kerf_mm < 0.5) {
        if (skill == "laser_cut")    return 0.70;
        if (skill == "waterjet_cut") return 0.50;
        return 0.0;
    }
    // 0.5 ≤ kerf < 1.2
    else if (kerf_mm < 1.2) {
        if (skill == "waterjet_cut") return 0.70;
        if (skill == "laser_cut")    return 0.30;
        return 0.0;
    }
    // 1.2 ≤ kerf < 3.0
    else if (kerf_mm < 3.0) {
        if (skill == "saw_cut")      return 0.60;
        if (skill == "waterjet_cut") return 0.50;
        if (skill == "plasma_cut")   return 0.30;
        return 0.0;
    }
    // 3.0 ≤ kerf ≤ 6.0
    else if (kerf_mm <= 6.0) {
        if (skill == "plasma_cut")   return 0.60;
        if (skill == "saw_cut")      return 0.40;
        return 0.0;
    }
    // kerf > 6.0
    else {
        if (skill == "oxyfuel_cut")  return 0.70;
        if (skill == "plasma_cut")   return 0.30;
        return 0.0;
    }
}

// Build the RecognizedFeature list for a given target skill_id by scanning
// the workpiece, classifying each detected channel, and weighting confidence
// via the confidence table.
inline std::vector<RecognizedFeature> recognizeForSkill(
    const Workpiece& wp, const std::string& target_skill_id)
{
    using nlohmann::json;
    std::vector<RecognizedFeature> out;
    const auto channels = scanThroughChannels(wp);
    for (const auto& ch : channels) {
        const double conf = confidenceForKerf(ch.kerf_mm, target_skill_id);
        if (conf <= 0.0) continue;

        json recovered = {
            { "cut_kind",    ch.cut_kind },
            { "kerf_mm",     ch.kerf_mm },
            { "path_length_mm", ch.path_length_mm },
        };
        json matched = {
            { "kerf_mm",       ch.kerf_mm },
            { "cut_kind",      ch.cut_kind },
            { "wall_face_ids", ch.wall_face_ids },
        };
        out.push_back(RecognizedFeature{ target_skill_id, recovered, conf, matched });
    }
    return out;
}

}  // namespace koocadcam::skill::separation_common
