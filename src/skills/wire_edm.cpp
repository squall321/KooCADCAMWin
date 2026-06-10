// @lat: [[engine/skills#wire_edm]]

#include "wire_edm.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"

#include <BRepAdaptor_Curve.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepGProp.hxx>
#include <BRepPrimAPI_MakePrism.hxx>
#include <GProp_GProps.hxx>
#include <NCollection_IndexedDataMap.hxx>
#include <NCollection_List.hxx>
#include <Standard_Failure.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopTools_ShapeMapHasher.hxx>
#include <TopoDS.hxx>
#include <gp.hxx>
#include <gp_Pln.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <map>
#include <set>
#include <utility>
#include <vector>

namespace koocadcam::skill::wire_edm {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.contour_waypoints.size() < 3) {
        r.add("DFM-INPUT", "error",
              "wire_edm contour_waypoints needs ≥ 3 points to form a closed "
              "polygon (got " + std::to_string(in.contour_waypoints.size()) + ")");
    }
    // Reject degenerate contours where consecutive points are identical.
    for (size_t i = 1; i < in.contour_waypoints.size(); ++i) {
        const double dx = in.contour_waypoints[i][0] - in.contour_waypoints[i-1][0];
        const double dy = in.contour_waypoints[i][1] - in.contour_waypoints[i-1][1];
        if (std::hypot(dx, dy) < 1e-6) {
            r.add("DFM-INPUT", "error",
                  "wire_edm contour has duplicate consecutive points at index "
                  + std::to_string(i));
            break;
        }
    }
    // Phase-1 supports only ±Z wire axis.
    if (std::abs(in.axis_dir.X()) > 1e-6 || std::abs(in.axis_dir.Y()) > 1e-6) {
        r.add("DFM-INPUT", "error",
              "wire_edm: only axis_dir along ±Z is supported in slice 1");
    }

    // Informational: wire kerf is 0.25 mm — let CAPP know.
    r.add("DFM-WIRE-KERF", "info",
          "wire_edm assumes 0.25 mm brass-wire kerf (compensated by the "
          "polygon offset); thinner/thicker wires not modelled here");
    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

namespace {

// Build a planar polygonal face in the XY plane at z = z0, then prism-extrude
// it along ±Z by `height`.  The polygon is closed (last → first auto-closure).
TopoDS_Shape buildPolygonalPrism(
    const std::vector<std::array<double, 2>>& pts,
    double z0, double zDir, double height)
{
    // Build the wire edge by edge.
    BRepBuilderAPI_MakeWire wireMaker;
    const size_t N = pts.size();
    for (size_t i = 0; i < N; ++i) {
        const auto& a = pts[i];
        const auto& b = pts[(i + 1) % N];
        const gp_Pnt p1(a[0], a[1], z0);
        const gp_Pnt p2(b[0], b[1], z0);
        // Skip zero-length edges defensively.
        if (p1.Distance(p2) < 1e-9) continue;
        wireMaker.Add(BRepBuilderAPI_MakeEdge(p1, p2).Edge());
    }
    if (!wireMaker.IsDone())
        throw Standard_Failure("wire_edm: wire build failed");

    BRepBuilderAPI_MakeFace faceMaker(wireMaker.Wire(), true);
    if (!faceMaker.IsDone())
        throw Standard_Failure("wire_edm: planar face build failed");

    const gp_Vec extrude(0.0, 0.0, zDir * height);
    BRepPrimAPI_MakePrism prism(faceMaker.Face(), extrude);
    prism.Build();
    if (!prism.IsDone())
        throw Standard_Failure("wire_edm: prism extrude failed");
    return prism.Shape();
}

}  // namespace

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "wire_edm DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    auto entryId = wp.resolve(in.entry_face);
    if (!entryId) throw SkillError("wire_edm: entry_face datum unresolved");

    // Workpiece bbox — the prism must extend strictly beyond the stock so
    // the Boolean cut is clean on both sides.
    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double thickness = zMax - zMin;
    const double overhang  = 0.5;   // both ends — comfortably outside stock
    const double prismHt   = thickness + 2.0 * overhang;
    // Start the prism above the top face (assuming axis_dir.Z() < 0 — cuts
    // downward) so that the extrusion covers the full thickness of the stock.
    const double z0    = zMax + overhang;
    const double zSign = (in.axis_dir.Z() < 0) ? -1.0 : 1.0;

    const TopoDS_Shape cutter =
        buildPolygonalPrism(in.contour_waypoints, z0, zSign, prismHt);

    const TopoDS_Shape newShape = pr::cut(wp.shape(), cutter);

    // Estimate path length (for cycle-time and stock-removed).
    double pathLen = 0.0;
    const size_t N = in.contour_waypoints.size();
    for (size_t i = 0; i < N; ++i) {
        const auto& a = in.contour_waypoints[i];
        const auto& b = in.contour_waypoints[(i + 1) % N];
        pathLen += std::hypot(b[0] - a[0], b[1] - a[1]);
    }

    // Approximate enclosed area via the shoelace formula.
    double polyArea = 0.0;
    for (size_t i = 0; i < N; ++i) {
        const auto& a = in.contour_waypoints[i];
        const auto& b = in.contour_waypoints[(i + 1) % N];
        polyArea += a[0] * b[1] - b[0] * a[1];
    }
    polyArea = std::abs(polyArea) * 0.5;

    json wpJson = json::array();
    for (const auto& w : in.contour_waypoints) wpJson.push_back({ w[0], w[1] });

    json params = {
        { "entry_face_id",     *entryId },
        { "contour_waypoints", wpJson },
        { "axis_dir",          { in.axis_dir.X(), in.axis_dir.Y(), in.axis_dir.Z() } },
    };
    json pattern = {
        { "kind",                  kSkillId },
        { "planar_wall_count",     N },
        { "bottom_planar_face",    false },          // through-cut: no bottom
        { "is_through_cut",        true },
        { "polygon_edge_count",    N },
        { "enclosed_area_mm2",     polyArea },
        { "axis_dir",              { in.axis_dir.X(), in.axis_dir.Y(), in.axis_dir.Z() } },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code", f.code },
                            { "severity", f.severity },
                            { "message", f.message }});

    ToolingMeta tooling;
    tooling.tool_type         = "edm_wire";
    tooling.tool_dia_mm       = 0.25;        // standard brass wire
    tooling.tool_length_mm    = thickness;   // wire spans the full part
    tooling.tool_material     = "brass";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    // Stock removed ≈ enclosed polygon area × thickness (for a single closed loop).
    tooling.stock_removed_mm3 = polyArea * thickness;
    // Wire EDM is slow: typical 2 mm² / minute area-rate; convert to seconds
    // per polygon path × thickness.
    tooling.est_cycle_time_s  = std::max(10.0, (pathLen * thickness) / 2.0);
    tooling.extra["dfm_findings"] = findings;
    tooling.extra["machining_constraint"] =
        "Wire EDM — through cut only, vertical wire, sharp corners "
        "(limited by wire radius)";

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);  // keep full chain history
    wpNew->addFeature(sig);

    spdlog::debug("skill::wire_edm applied: waypoints={} thickness={} faces {}→{}",
                  N, thickness, wp.faceCount(), wpNew->faceCount());

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// Look for vertical-walled through-features whose horizontal cross-section
// is a polygonal loop (≥ 3 distinct planar walls all sharing the same wire
// axis ±Z, with NO bottom face).
//
// Algorithm:
//   1. Find planar faces whose normal is purely horizontal (|nz| ≈ 0) and
//      that share edges with both the workpiece TOP and BOTTOM faces.
//   2. Cluster these walls by VERTICAL-edge connectivity (two walls that
//      share a vertical edge are adjacent in the polygon).
//   3. Each closed loop = one polygon.  The OUTER loop (whose XY bbox
//      matches the workpiece XY bbox) is the stock outline — skip it.
//      The remaining loops are wire-EDM through-cuts.
//   4. Reconstruct the polygon waypoints by walking each loop's vertical
//      edges and collecting their TOP-Z endpoints in order.

namespace {

using EdgeFaceMap = NCollection_IndexedDataMap<
    TopoDS_Shape, NCollection_List<TopoDS_Shape>, TopTools_ShapeMapHasher>;

EdgeFaceMap buildEdgeFaceMap(const TopoDS_Shape& shape)
{
    EdgeFaceMap m;
    TopExp::MapShapesAndAncestors(shape, TopAbs_EDGE, TopAbs_FACE, m);
    return m;
}

// Return true if `e` is a straight line edge with direction parallel to ±Z
// (a vertical wall-to-wall edge in a through-cut polygon).
bool isVerticalLineEdge(const TopoDS_Edge& e, double zMin, double zMax)
{
    BRepAdaptor_Curve crv(e);
    if (crv.GetType() != GeomAbs_Line) return false;
    const gp_Pnt p1 = crv.Value(crv.FirstParameter());
    const gp_Pnt p2 = crv.Value(crv.LastParameter());
    // XY-coincident (so it's vertical) and Z spans the full thickness.
    if (std::hypot(p1.X() - p2.X(), p1.Y() - p2.Y()) > 1e-3) return false;
    const double zLo = std::min(p1.Z(), p2.Z());
    const double zHi = std::max(p1.Z(), p2.Z());
    return (std::abs(zLo - zMin) < 1e-2) && (std::abs(zHi - zMax) < 1e-2);
}

}  // namespace

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;
    const auto edgeFaces = buildEdgeFaceMap(wp.shape());

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);

    // Identify the workpiece's TOP and BOTTOM horizontal faces (the planar
    // faces whose normal is ±Z and centers sit at zMax / zMin).
    std::set<int> topFaces, botFaces;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFacePlanar(i)) continue;
        gp_Dir n;
        try { n = wp.faceNormal(i); } catch (...) { continue; }
        if (std::abs(n.Z()) < 0.95) continue;
        const gp_Pnt c = wp.faceCenter(i);
        if (std::abs(c.Z() - zMax) < 1e-2) topFaces.insert(i);
        if (std::abs(c.Z() - zMin) < 1e-2) botFaces.insert(i);
    }

    // Now find vertical planar walls that share edges with BOTH a top face
    // and a bottom face — those are vertical through-cut walls (slot walls
    // OR the workpiece's outer side walls).
    std::vector<int> verticalWalls;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFacePlanar(i)) continue;
        gp_Dir n;
        try { n = wp.faceNormal(i); } catch (...) { continue; }
        // Vertical wall: |nz| ≈ 0.
        if (std::abs(n.Z()) > 0.02) continue;

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
        if (touchesTop && touchesBot) verticalWalls.push_back(i);
    }
    if (verticalWalls.size() < 3) return out;   // no polygons possible

    // ── Cluster walls into closed loops via shared vertical edges ────────
    // Build an adjacency map: wall idx → list of (otherWall, sharedTopVertex
    // XY) pairs.  A shared vertical edge is a line edge spanning zMin..zMax
    // that both walls own.
    std::map<int, std::vector<std::pair<int, std::array<double, 2>>>> walkAdj;
    for (int wi : verticalWalls) {
        for (TopExp_Explorer exp(wp.face(wi), TopAbs_EDGE); exp.More(); exp.Next()) {
            const TopoDS_Edge& e = TopoDS::Edge(exp.Current());
            if (!isVerticalLineEdge(e, zMin, zMax)) continue;
            // Top-Z endpoint = a polygon corner.
            BRepAdaptor_Curve crv(e);
            const gp_Pnt pA = crv.Value(crv.FirstParameter());
            const gp_Pnt pB = crv.Value(crv.LastParameter());
            const gp_Pnt top = (pA.Z() > pB.Z()) ? pA : pB;
            // Find the OTHER wall sharing this edge.
            if (!edgeFaces.Contains(e)) continue;
            const auto& adj = edgeFaces.FindFromKey(e);
            for (NCollection_List<TopoDS_Shape>::Iterator it(adj); it.More(); it.Next()) {
                const TopoDS_Face& af = TopoDS::Face(it.Value());
                if (af.IsSame(wp.face(wi))) continue;
                for (int wj : verticalWalls) {
                    if (wj == wi) continue;
                    if (!wp.face(wj).IsSame(af)) continue;
                    walkAdj[wi].push_back({ wj, { top.X(), top.Y() } });
                    break;
                }
            }
        }
    }

    // Find connected components (loops) via DFS on walkAdj.
    std::set<int> visited;
    std::vector<std::vector<int>> loops;
    for (int seed : verticalWalls) {
        if (visited.count(seed)) continue;
        std::vector<int> loop;
        std::vector<int> stack { seed };
        while (!stack.empty()) {
            const int w = stack.back(); stack.pop_back();
            if (visited.count(w)) continue;
            visited.insert(w);
            loop.push_back(w);
            auto it = walkAdj.find(w);
            if (it == walkAdj.end()) continue;
            for (const auto& adj : it->second) {
                if (!visited.count(adj.first)) stack.push_back(adj.first);
            }
        }
        if (loop.size() >= 3) loops.push_back(std::move(loop));
    }

    // For each loop, classify outer vs inner by comparing its XY bbox to
    // the workpiece XY bbox.  Outer loop ≈ workpiece bbox → SKIP.
    for (const auto& loop : loops) {
        double lxMin =  1e30, lyMin =  1e30;
        double lxMax = -1e30, lyMax = -1e30;
        std::vector<std::array<double, 2>> corners;   // top-z vertices
        for (int wi : loop) {
            auto it = walkAdj.find(wi);
            if (it == walkAdj.end()) continue;
            for (const auto& adj : it->second) {
                const auto& xy = adj.second;
                corners.push_back(xy);
                lxMin = std::min(lxMin, xy[0]);
                lyMin = std::min(lyMin, xy[1]);
                lxMax = std::max(lxMax, xy[0]);
                lyMax = std::max(lyMax, xy[1]);
            }
        }
        // Outer loop?  Same XY extent as workpiece → skip.
        const bool isOuter =
            std::abs(lxMin - xMin) < 1e-2 && std::abs(lxMax - xMax) < 1e-2 &&
            std::abs(lyMin - yMin) < 1e-2 && std::abs(lyMax - yMax) < 1e-2;
        if (isOuter) continue;

        // Dedupe waypoint corners — each top-Z vertex is visited twice
        // (once per adjacent wall).  Keep one entry per unique XY (tol 1e-3).
        std::vector<std::array<double, 2>> waypoints;
        for (const auto& c : corners) {
            bool dup = false;
            for (const auto& w : waypoints)
                if (std::hypot(c[0] - w[0], c[1] - w[1]) < 1e-3) { dup = true; break; }
            if (!dup) waypoints.push_back(c);
        }

        json contour = json::array();
        for (const auto& p : waypoints) contour.push_back({ p[0], p[1] });

        json recovered = {
            { "contour_waypoints", contour },
            { "axis_dir",          { 0.0, 0.0, -1.0 } },
        };
        json matched = {
            { "wall_face_ids",  json(loop) },
            { "polygon_size",   loop.size() },
        };
        out.push_back(RecognizedFeature{
            kSkillId, recovered, /*confidence*/ 0.70, matched
        });
    }
    return out;
}

}  // namespace koocadcam::skill::wire_edm
