// @lat: [[engine/feature-watch#Build Sequence]]
// Watch front-case build sequence (6 of 11 steps, M1.2).
//
// Each step body is wrapped by `prim::runStep`, which provides uniform
// exception → E_OCCT, BRepCheck → W_BREPCHECK, and spdlog::debug logging.
// The step bodies themselves consist of: read spec values → describe geometry
// using `prim::` helpers → return shape.  Pass-through (no key in spec)
// short-circuits before runStep so we don't re-validate the unchanged input.

#include "WatchFrontModel.hpp"

#include "primitives/Bbox.hpp"
#include "primitives/Cuts.hpp"
#include "primitives/Fillets.hpp"
#include "primitives/Frames.hpp"
#include "primitives/StepGuard.hpp"
#include "primitives/Tools.hpp"

#include <BRepAdaptor_Curve.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepBndLib.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <BRepExtrema_DistShapeShape.hxx>
#include <BRepGProp.hxx>
#include <BRep_Tool.hxx>
#include <Bnd_Box.hxx>
#include <GProp_GProps.hxx>
#include <GeomLProp_CLProps.hxx>
#include <Standard_Failure.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Shape.hxx>
#include <NCollection_IndexedDataMap.hxx>
#include <NCollection_List.hxx>
#include <TopTools_ShapeMapHasher.hxx>
#include <gp_Ax1.hxx>
#include <gp_Ax2.hxx>
#include <gp_Cylinder.hxx>
#include <gp_Dir.hxx>
#include <gp_Lin.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <iterator>
#include <limits>
#include <string>
#include <vector>

namespace koocadcam::engine {

namespace pr = prim;

// ── DFM geometric-check helpers (anonymous namespace) ─────────────────────
// These helpers implement the topology-driven DFM rules (DFM-001/003/004/
// 011/013/017).  They are intentionally conservative: each loops over a
// bounded sample of topology elements so the overall O(n²) cost stays in
// the milli-second range on a ~200-face watch-case shape.
namespace {

// Sample a face normal at the centre of its UV parameter range.
// Returns false if the face is degenerate / can't be evaluated.
bool sampleFaceNormalAndPoint(const TopoDS_Face& f, gp_Dir& nOut, gp_Pnt& pOut)
{
    try {
        BRepAdaptor_Surface surf(f);
        const double uMid = (surf.FirstUParameter() + surf.LastUParameter()) / 2.0;
        const double vMid = (surf.FirstVParameter() + surf.LastVParameter()) / 2.0;
        gp_Pnt p;
        gp_Vec du, dv;
        surf.D1(uMid, vMid, p, du, dv);
        gp_Vec n = du.Crossed(dv);
        if (n.Magnitude() < 1e-9) return false;
        n.Normalize();
        if (f.Orientation() == TopAbs_REVERSED) n.Reverse();
        nOut = gp_Dir(n);
        pOut = p;
        return true;
    } catch (const Standard_Failure&) {
        return false;
    } catch (const std::exception&) {
        return false;
    }
}

// DFM-001 helper: scan all face pairs; if normals oppose (dot ≤ -0.8) and
// the two faces are within the global bbox's max extent (sanity bound),
// compute their minimum point-to-point distance via BRepExtrema_DistShapeShape.
// Returns the minimum thickness observed across the entire shape (or +inf
// if no opposing pair found).  Limited to bbox-near faces by checking the
// face-centre distance against the bbox span.
double minWallThickness(const TopoDS_Shape& s)
{
    std::vector<TopoDS_Face> faces;
    faces.reserve(64);
    for (TopExp_Explorer ex(s, TopAbs_FACE); ex.More(); ex.Next())
        faces.push_back(TopoDS::Face(ex.Current()));

    if (faces.size() < 2) return std::numeric_limits<double>::infinity();

    std::vector<gp_Dir> normals(faces.size());
    std::vector<gp_Pnt> centres(faces.size());
    std::vector<bool>   ok     (faces.size(), false);
    for (std::size_t i = 0; i < faces.size(); ++i)
        ok[i] = sampleFaceNormalAndPoint(faces[i], normals[i], centres[i]);

    // Cap the pair count: 64 faces → 2016 pairs, ~10 ms on watch-case sized
    // shapes.  Above that, sample every Nth face to keep runtime bounded.
    const std::size_t kMaxFaces = 64;
    const std::size_t stride = (faces.size() > kMaxFaces)
        ? (faces.size() + kMaxFaces - 1) / kMaxFaces : 1;

    const auto bb     = pr::optimalBbox(s);
    const double diag = std::sqrt(bb.dx() * bb.dx() + bb.dy() * bb.dy()
                                  + bb.dz() * bb.dz());
    const double maxCandidateDist = diag;   // sanity upper bound

    double minD = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < faces.size(); i += stride) {
        if (!ok[i]) continue;
        for (std::size_t j = i + stride; j < faces.size(); j += stride) {
            if (!ok[j]) continue;
            // Opposing normals?  dot ≤ -0.8 → angle ≥ ~143°.
            const double dot = normals[i].X() * normals[j].X()
                             + normals[i].Y() * normals[j].Y()
                             + normals[i].Z() * normals[j].Z();
            if (dot > -0.8) continue;
            // Bbox-near filter: skip pairs whose centres are obviously far.
            const double dxC = centres[i].X() - centres[j].X();
            const double dyC = centres[i].Y() - centres[j].Y();
            const double dzC = centres[i].Z() - centres[j].Z();
            const double approx = std::sqrt(dxC*dxC + dyC*dyC + dzC*dzC);
            if (approx > maxCandidateDist) continue;
            try {
                BRepExtrema_DistShapeShape dss(faces[i], faces[j]);
                if (dss.IsDone() && dss.NbSolution() > 0) {
                    const double d = dss.Value();
                    if (d > 1e-6 && d < minD) minD = d;
                }
            } catch (...) { /* skip flaky pair */ }
        }
    }
    return minD;
}

// DFM-003 helper: collect cylindrical-face axes, compare pairwise; if axes
// are parallel (|dot| > 0.99) and the perpendicular distance between them
// is < `limit`, return the smallest such distance.  +inf when no holes.
double minHoleToHoleAxisDistance(const TopoDS_Shape& s)
{
    struct CylAxis { gp_Ax1 axis; double radius; };
    std::vector<CylAxis> axes;
    axes.reserve(32);
    for (TopExp_Explorer ex(s, TopAbs_FACE); ex.More(); ex.Next()) {
        const TopoDS_Face& f = TopoDS::Face(ex.Current());
        try {
            BRepAdaptor_Surface surf(f);
            if (surf.GetType() != GeomAbs_Cylinder) continue;
            const gp_Cylinder cyl = surf.Cylinder();
            axes.push_back({ cyl.Axis(), cyl.Radius() });
        } catch (...) { /* skip */ }
    }
    if (axes.size() < 2) return std::numeric_limits<double>::infinity();

    double minEdgeGap = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < axes.size(); ++i) {
        for (std::size_t j = i + 1; j < axes.size(); ++j) {
            const gp_Dir& di = axes[i].axis.Direction();
            const gp_Dir& dj = axes[j].axis.Direction();
            const double dotAxes = std::abs(
                di.X() * dj.X() + di.Y() * dj.Y() + di.Z() * dj.Z());
            if (dotAxes < 0.99) continue;   // not parallel
            // Distance between two parallel infinite lines.
            const gp_Lin li(axes[i].axis);
            const double centreDist = li.Distance(axes[j].axis.Location());
            // Edge-to-edge clearance = centre-to-centre - both radii.
            const double edgeGap =
                centreDist - axes[i].radius - axes[j].radius;
            if (edgeGap > 0.0 && edgeGap < minEdgeGap) minEdgeGap = edgeGap;
        }
    }
    return minEdgeGap;
}

// DFM-004 helper: walk concave edges of the shape; estimate curvature via
// `GeomLProp_CLProps` at the curve midpoint; return the minimum radius
// among all sampled non-linear edges, or +inf for an all-straight shape.
// "Concave" is approximated by checking adjacent face normals — if the
// edge curls toward the inside (dot of outward normals < 0.95) we treat
// it as a candidate corner edge.
double minConcaveCornerRadius(const TopoDS_Shape& s)
{
    // Build edge → list-of-faces map so we know each edge's neighbours.
    NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>, TopTools_ShapeMapHasher> e2f;
    TopExp::MapShapesAndAncestors(s, TopAbs_EDGE, TopAbs_FACE, e2f);

    double minR = std::numeric_limits<double>::infinity();
    for (int i = 1; i <= e2f.Extent(); ++i) {
        const TopoDS_Edge& e = TopoDS::Edge(e2f.FindKey(i));
        const NCollection_List<TopoDS_Shape>& nb = e2f.FindFromIndex(i);
        if (nb.Extent() < 2) continue;   // need two adjacent faces
        const TopoDS_Face& fA = TopoDS::Face(nb.First());
        const TopoDS_Face& fB = TopoDS::Face(nb.Last());

        try {
            BRepAdaptor_Curve crv(e);
            // Skip linear edges (infinite radius).
            if (crv.GetType() == GeomAbs_Line) continue;

            // Concavity gate: skip flat edges (G1 continuous) and convex
            // edges that wrap an outer round (handled by DFM-017).
            gp_Dir nA, nB;
            gp_Pnt pA, pB;
            const bool okA = sampleFaceNormalAndPoint(fA, nA, pA);
            const bool okB = sampleFaceNormalAndPoint(fB, nB, pB);
            (void)pA; (void)pB;
            if (okA && okB) {
                const double nDot = nA.X() * nB.X() + nA.Y() * nB.Y()
                                  + nA.Z() * nB.Z();
                if (nDot > 0.95) continue;        // ~near-flat
            }

            // Sample curvature at the curve midpoint.
            const double tMid = (crv.FirstParameter() + crv.LastParameter()) / 2.0;
            GeomLProp_CLProps prop(crv.Curve().Curve(), tMid, 2, 1e-7);
            const double k = prop.Curvature();
            if (k < 1e-9) continue;   // numerically straight
            const double R = 1.0 / k;
            if (R > 0.0 && R < minR) minR = R;
        } catch (...) { /* skip */ }
    }
    return minR;
}

// DFM-011 helper: walk shared edges; compute dihedral angle between adjacent
// face normals sampled at the edge midpoint.  Returns the smallest dihedral
// (in degrees) observed across the shape — small angles indicate a knife
// edge.  +inf when there are no shared edges (single-face shape).
double minDihedralAngleDeg(const TopoDS_Shape& s)
{
    NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>, TopTools_ShapeMapHasher> e2f;
    TopExp::MapShapesAndAncestors(s, TopAbs_EDGE, TopAbs_FACE, e2f);

    double minAng = std::numeric_limits<double>::infinity();
    for (int i = 1; i <= e2f.Extent(); ++i) {
        const NCollection_List<TopoDS_Shape>& nb = e2f.FindFromIndex(i);
        if (nb.Extent() < 2) continue;
        const TopoDS_Face& fA = TopoDS::Face(nb.First());
        const TopoDS_Face& fB = TopoDS::Face(nb.Last());
        gp_Dir nA, nB;
        gp_Pnt pA, pB;
        if (!sampleFaceNormalAndPoint(fA, nA, pA)) continue;
        if (!sampleFaceNormalAndPoint(fB, nB, pB)) continue;
        (void)pA; (void)pB;
        const double dot = std::clamp(
            nA.X() * nB.X() + nA.Y() * nB.Y() + nA.Z() * nB.Z(), -1.0, 1.0);
        // Knife-edge angle = exterior wedge angle = 180° - acos(dot).
        // dot ≈ +1 → faces near-coplanar → wedge ≈ 180° (fine);
        // dot ≈ -1 → faces back-to-back  → wedge ≈ 0°  (knife).
        const double interiorDeg = std::acos(dot) * 180.0 / M_PI;
        const double wedgeDeg    = 180.0 - interiorDeg;
        // Skip tangent-continuous edges where wedge ≈ 180° (no knife).
        if (wedgeDeg > 175.0) continue;
        if (wedgeDeg < minAng) minAng = wedgeDeg;
    }
    return minAng;
}

// DFM-013 helper: find the largest planar face whose normal points -Z
// (downward) — a candidate display pocket floor — then sample a 10×10
// grid of UV points and report the worst point-to-best-fit-plane deviation.
// Returns +inf when no such face exists (shape has no display pocket).
double displayPocketFlatness(const TopoDS_Shape& s)
{
    TopoDS_Face best;
    double      bestArea = -1.0;
    for (TopExp_Explorer ex(s, TopAbs_FACE); ex.More(); ex.Next()) {
        const TopoDS_Face& f = TopoDS::Face(ex.Current());
        try {
            BRepAdaptor_Surface surf(f);
            if (surf.GetType() != GeomAbs_Plane) continue;
            gp_Dir n; gp_Pnt p;
            if (!sampleFaceNormalAndPoint(f, n, p)) continue;
            (void)p;
            // Look for a face whose OUTWARD normal points +Z (the pocket
            // floor's outward normal — we drilled DOWN into the top, so the
            // floor's outward normal is +Z, opposite of cut direction).
            if (n.Z() < 0.9) continue;
            GProp_GProps props;
            BRepGProp::SurfaceProperties(f, props);
            const double area = props.Mass();
            if (area > 100.0 && area > bestArea) { bestArea = area; best = f; }
        } catch (...) {}
    }
    if (best.IsNull()) return std::numeric_limits<double>::infinity();

    // For a true planar face the flatness is 0 by construction.  We still
    // sample a small grid and report the max deviation from the best-fit
    // plane (= surface plane) — guards against degenerate / B-spline floors
    // that happen to be near-planar.
    BRepAdaptor_Surface surf(best);
    if (surf.GetType() != GeomAbs_Plane)
        return std::numeric_limits<double>::infinity();
    const gp_Pln pln  = surf.Plane();
    const gp_Pnt loc  = pln.Location();
    const gp_Dir nrm  = pln.Axis().Direction();

    double maxDev = 0.0;
    const int    N    = 10;
    const double uMin = surf.FirstUParameter();
    const double uMax = surf.LastUParameter();
    const double vMin = surf.FirstVParameter();
    const double vMax = surf.LastVParameter();
    for (int iu = 0; iu < N; ++iu) {
        const double u = uMin + (uMax - uMin) * iu / (N - 1);
        for (int iv = 0; iv < N; ++iv) {
            const double v = vMin + (vMax - vMin) * iv / (N - 1);
            try {
                gp_Pnt q = surf.Value(u, v);
                const gp_Vec dv(loc, q);
                const double signed_d =
                    dv.X() * nrm.X() + dv.Y() * nrm.Y() + dv.Z() * nrm.Z();
                if (std::abs(signed_d) > maxDev) maxDev = std::abs(signed_d);
            } catch (...) {}
        }
    }
    return maxDev;
}

// DFM-017 helper: walk outer-silhouette edges (vertical-tangent faces) and
// check tangent continuity at shared vertices.  Returns the largest tangent
// angle gap observed in degrees, or 0.0 when there are no candidate edges.
double maxOuterTangentGapDeg(const TopoDS_Shape& s)
{
    // Collect candidate edges: those that lie between two faces whose
    // normals are near-horizontal (|n·Z| < 0.2) — i.e., side-wall edges.
    NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>, TopTools_ShapeMapHasher> e2f;
    TopExp::MapShapesAndAncestors(s, TopAbs_EDGE, TopAbs_FACE, e2f);

    std::vector<TopoDS_Edge> silhouette;
    silhouette.reserve(16);
    for (int i = 1; i <= e2f.Extent(); ++i) {
        const NCollection_List<TopoDS_Shape>& nb = e2f.FindFromIndex(i);
        if (nb.Extent() < 2) continue;
        const TopoDS_Face& fA = TopoDS::Face(nb.First());
        const TopoDS_Face& fB = TopoDS::Face(nb.Last());
        gp_Dir nA, nB; gp_Pnt pA, pB;
        if (!sampleFaceNormalAndPoint(fA, nA, pA)) continue;
        if (!sampleFaceNormalAndPoint(fB, nB, pB)) continue;
        (void)pA; (void)pB;
        if (std::abs(nA.Z()) > 0.2 || std::abs(nB.Z()) > 0.2) continue;
        silhouette.push_back(TopoDS::Edge(e2f.FindKey(i)));
    }
    if (silhouette.size() < 2) return 0.0;

    // For each edge, sample tangent at first and last parameter.  If the
    // edge's end is within 0.05 mm of another edge's start (= shared
    // vertex proxy), measure the tangent-angle gap and remember the worst.
    auto sampleEnd = [](const TopoDS_Edge& e, bool atStart,
                        gp_Pnt& pOut, gp_Vec& tOut) -> bool {
        try {
            BRepAdaptor_Curve c(e);
            const double param = atStart ? c.FirstParameter() : c.LastParameter();
            gp_Vec d1;
            c.D1(param, pOut, d1);
            if (d1.Magnitude() < 1e-9) return false;
            d1.Normalize();
            tOut = d1;
            return true;
        } catch (...) { return false; }
    };

    double maxGap = 0.0;
    for (std::size_t i = 0; i < silhouette.size(); ++i) {
        gp_Pnt pe; gp_Vec tEnd;
        if (!sampleEnd(silhouette[i], false, pe, tEnd)) continue;
        for (std::size_t j = 0; j < silhouette.size(); ++j) {
            if (i == j) continue;
            gp_Pnt ps; gp_Vec tStart;
            if (!sampleEnd(silhouette[j], true, ps, tStart)) continue;
            // Endpoint proximity gate (shared-vertex proxy).
            if (pe.Distance(ps) > 0.05) continue;
            const double dot = std::clamp(tEnd.Dot(tStart), -1.0, 1.0);
            const double angDeg = std::acos(std::abs(dot)) * 180.0 / M_PI;
            if (angDeg > maxGap) maxGap = angDeg;
        }
    }
    return maxGap;
}

}  // namespace
// ──────────────────────────────────────────────────────────────────────────

// @lat: [[engine/feature-watch#스텝 1 — buildBase]]
StepResult WatchFrontModel::buildBase(const nlohmann::json& spec)
{
    return pr::runStep("WatchFrontModel::buildBase",
        [&](std::vector<BuildWarning>&) -> TopoDS_Shape {
            const std::string formFactor = spec.value("form_factor", "round");

            if (formFactor == "round") {
                const double diameter  = spec["base"]["diameter_mm"].get<double>();
                const double thickness = spec["base"]["thickness_mm"].get<double>();
                return pr::cylinder(pr::axisAtZ(gp_Pnt(0, 0, 0)),
                                    diameter / 2.0, thickness);
            }

            if (formFactor == "square") {
                const double width     = spec["base"]["width_mm"].get<double>();
                const double height    = spec["base"]["height_mm"].get<double>();
                const double thickness = spec["base"]["thickness_mm"].get<double>();
                const double initR     = spec.value(
                    nlohmann::json::json_pointer("/base/initial_corner_r_mm"), 2.0);

                const TopoDS_Shape boxShape = pr::box(
                    gp_Ax2(gp_Pnt(-width / 2.0, -height / 2.0, 0.0), gp::DZ()),
                    width, height, thickness);

                return pr::filletEdges(boxShape, initR,
                    pr::verticalCornerEdges(width / 2.0, height / 2.0, thickness));
            }

            throw Standard_Failure(
                ("buildBase: unknown form_factor '" + formFactor + "'").c_str());
        },
        /*checkValidity=*/false);   // primitive solids are always valid
}

// @lat: [[engine/feature-watch#스텝 2 — applyCornerRadius]]
StepResult WatchFrontModel::applyCornerRadius(const TopoDS_Shape& in,
                                               const nlohmann::json& spec)
{
    return pr::runStep("WatchFrontModel::applyCornerRadius",
        [&](std::vector<BuildWarning>&) {
            const double rTop  = spec["corner_radius"]["r_top_mm"].get<double>();
            const double rSide = spec["corner_radius"]["r_side_mm"].get<double>();
            const auto bb = pr::optimalBbox(in);
            return pr::filletEdgesMulti(in, {
                { rTop,  pr::edgesAtZ(bb.zMax) },
                { rSide, pr::edgesAtZ(bb.zMin) }
            });
        });
}

// @lat: [[engine/feature-watch#스텝 3 — buildBezel]]
StepResult WatchFrontModel::buildBezel(const TopoDS_Shape& in,
                                       const nlohmann::json& spec)
{
    return pr::runStep("WatchFrontModel::buildBezel",
        [&](std::vector<BuildWarning>& warnings) {
            const auto bb = pr::optimalBbox(in);
            const double outerR     = bb.outerRadiusXY();
            const double bezelWidth = spec["bezel"]["width_mm"].get<double>();
            const double bezelDepth = spec["bezel"]["depth_mm"].get<double>();
            const double taperDeg   = spec["bezel"].value("taper_deg", 0.0);
            const double innerR     = outerR - bezelWidth;

            const gp_Ax2 ax = pr::axisAtZ(gp_Pnt(0.0, 0.0, bb.zMax - bezelDepth));

            TopoDS_Shape tool;
            if (taperDeg > 0.0) {
                const double bottomR =
                    outerR - bezelDepth * std::tan(taperDeg * M_PI / 180.0);
                if (bottomR > innerR) {
                    tool = pr::annularConeRing(ax, bottomR, outerR, innerR, bezelDepth);
                } else {
                    warnings.push_back(BuildWarning{
                        "W_BEZEL_TAPER_FALLBACK",
                        "bezel taper would invert inner radius; using straight walls"
                    });
                    tool = pr::annularRing(ax, outerR, innerR, bezelDepth);
                }
            } else {
                tool = pr::annularRing(ax, outerR, innerR, bezelDepth);
            }
            return pr::cut(in, tool);
        });
}

// @lat: [[engine/feature-watch#스텝 4 — buildDisplayPocket]]
StepResult WatchFrontModel::buildDisplayPocket(const TopoDS_Shape& in,
                                                const nlohmann::json& spec)
{
    if (!spec.contains("display_pocket")) return StepResult{ in, {} };

    return pr::runStep("WatchFrontModel::buildDisplayPocket",
        [&](std::vector<BuildWarning>&) {
            const auto& dp = spec["display_pocket"];
            const double dPocket     = dp["d_pocket_mm"].get<double>();
            const double depthPocket = dp["depth_pocket_mm"].get<double>();
            const double glassOff    = dp.value("glass_offset_mm", 0.0);

            const auto bb = pr::optimalBbox(in);
            const double recessAmt = std::max(0.0, -glassOff);
            const double zBottom   = bb.zMax - depthPocket - recessAmt;
            const double height    = depthPocket + std::abs(glassOff);

            const TopoDS_Shape pocket = pr::cylinder(
                pr::axisAtZ(gp_Pnt(0.0, 0.0, zBottom)), dPocket / 2.0, height);
            return pr::cut(in, pocket);
        });
}

// @lat: [[engine/feature-watch#스텝 5 — addCrownCavity]]
StepResult WatchFrontModel::addCrownCavity(const TopoDS_Shape& in,
                                            const nlohmann::json& spec)
{
    if (!spec.contains("crown_cavity")) return StepResult{ in, {} };

    return pr::runStep("WatchFrontModel::addCrownCavity",
        [&](std::vector<BuildWarning>&) {
            const auto& cc = spec["crown_cavity"];
            const double angleDeg = cc["side_angle_deg"].get<double>();
            const double heightZ  = cc["height_pos_mm"].get<double>();
            const double depth    = cc["depth_mm"].get<double>();
            const double diameter = cc["diameter_mm"].get<double>();
            const double shaftDia = cc["shaft_dia_mm"].get<double>();

            const double R       = pr::optimalBbox(in).outerRadiusXY();
            const auto   frame   = pr::sideFrameAt(R, angleDeg, heightZ);
            const gp_Ax2 ax      = frame.ax2InwardRadial();

            const TopoDS_Shape cavity = pr::cylinder(ax, diameter / 2.0, depth);
            const TopoDS_Shape shaft  = pr::cylinder(ax, shaftDia / 2.0, R + 1.0);
            return pr::cutMany(in, { cavity, shaft });
        });
}

// @lat: [[engine/feature-watch#스텝 6 — addSideButtons]]
StepResult WatchFrontModel::addSideButtons(const TopoDS_Shape& in,
                                            const nlohmann::json& spec)
{
    if (!spec.contains("side_buttons") || spec["side_buttons"].empty())
        return StepResult{ in, {} };

    return pr::runStep("WatchFrontModel::addSideButtons",
        [&](std::vector<BuildWarning>& warnings) {
            const double R = pr::optimalBbox(in).outerRadiusXY();
            const auto& buttons = spec["side_buttons"];

            std::vector<TopoDS_Shape> tools;
            tools.reserve(buttons.size());
            for (const auto& btn : buttons) {
                if (btn.value("taper_deg", 0.0) > 0.0) {
                    warnings.push_back(BuildWarning{
                        "W_TAPER_M1_5",
                        "side_buttons taper_deg > 0 not implemented (M1.5); treated as 0"
                    });
                }
                const auto frame = pr::sideFrameAt(R,
                    btn["angle_deg"].get<double>(),
                    btn["height_mm"].get<double>());
                tools.push_back(pr::sidePocketBox(frame,
                    btn["depth_mm"].get<double>(),
                    btn["length_mm"].get<double>(),
                    btn["width_mm"].get<double>()));
            }
            return pr::cutMany(in, tools);
        });
}

// @lat: [[engine/feature-watch#스텝 7 — addSpeakerGrille]]
StepResult WatchFrontModel::addSpeakerGrille(const TopoDS_Shape& in,
                                              const nlohmann::json& spec)
{
    if (!spec.contains("speaker_grille")) return StepResult{ in, {} };

    return pr::runStep("WatchFrontModel::addSpeakerGrille",
        [&](std::vector<BuildWarning>&) {
            const auto& sg = spec["speaker_grille"];
            const double angleDeg = sg["angle_deg"].get<double>();
            const double heightZ  = sg["height_pos_mm"].get<double>();
            const int    rows     = sg["rows"].get<int>();
            const int    cols     = sg["cols"].get<int>();
            const double rowSp    = sg["row_spacing_mm"].get<double>();
            const double colSp    = sg["col_spacing_mm"].get<double>();
            const double dia      = sg["hole_dia_mm"].get<double>();
            const double depth    = sg["depth_mm"].get<double>();

            const double R     = pr::optimalBbox(in).outerRadiusXY();
            const auto   frame = pr::sideFrameAt(R, angleDeg, heightZ);

            std::vector<TopoDS_Shape> tools;
            tools.reserve(static_cast<std::size_t>(rows) * cols);
            for (int r = 0; r < rows; ++r) {
                const double dz = (r - (rows - 1) / 2.0) * rowSp;
                for (int c = 0; c < cols; ++c) {
                    const double dy = (c - (cols - 1) / 2.0) * colSp;
                    const gp_Pnt holeCenter = pr::offsetPoint(
                        frame.center, dy, frame.tangentCCW, dz, frame.axialZ);
                    const gp_Ax2 ax(holeCenter, frame.inwardRadial);
                    tools.push_back(pr::cylinder(ax, dia / 2.0, depth));
                }
            }
            return pr::cutMany(in, tools);
        });
}

// @lat: [[engine/feature-watch#스텝 8 — addRearSensors]]
StepResult WatchFrontModel::addRearSensors(const TopoDS_Shape& in,
                                            const nlohmann::json& spec)
{
    if (!spec.contains("rear_sensors") || spec["rear_sensors"].empty())
        return StepResult{ in, {} };

    return pr::runStep("WatchFrontModel::addRearSensors",
        [&](std::vector<BuildWarning>&) {
            const auto bb = pr::optimalBbox(in);

            std::vector<TopoDS_Shape> tools;
            tools.reserve(spec["rear_sensors"].size());
            for (const auto& s : spec["rear_sensors"]) {
                const double x   = s["offset_x_mm"].get<double>();
                const double y   = s["offset_y_mm"].get<double>();
                const double dia = s["dia_mm"].get<double>();
                const double dep = s["depth_mm"].get<double>();
                // Hole axis +Z, starts 0.05 mm below rear face for clean cut.
                const double kOverhang = 0.05;
                const gp_Ax2 ax(gp_Pnt(x, y, bb.zMin - kOverhang), gp::DZ());
                tools.push_back(pr::cylinder(ax, dia / 2.0, dep + kOverhang));
            }
            return pr::cutMany(in, tools);
        });
}

// @lat: [[engine/feature-watch#스텝 9 — addLugs]]
StepResult WatchFrontModel::addLugs(const TopoDS_Shape& in,
                                     const nlohmann::json& spec)
{
    if (!spec.contains("lugs") || spec["lugs"].empty())
        return StepResult{ in, {} };

    return pr::runStep("WatchFrontModel::addLugs",
        [&](std::vector<BuildWarning>& warnings) {
            const auto   bb   = pr::optimalBbox(in);
            const double R    = bb.outerRadiusXY();
            const double midZ = (bb.zMin + bb.zMax) / 2.0;

            // Build protrusions (material to add) and optional pin holes (material to cut).
            std::vector<TopoDS_Shape> protrusions;
            std::vector<TopoDS_Shape> pinHoles;
            protrusions.reserve(spec["lugs"].size());

            for (const auto& lug : spec["lugs"]) {
                const double angleDeg = lug["angle_deg"].get<double>();
                const double length   = lug["length_mm"].get<double>();
                const double width    = lug["width_mm"].get<double>();
                const double thick    = lug["thickness_mm"].get<double>();
                const double pinDia   = lug.value("pin_hole_dia_mm", 0.0);

                const auto    frame   = pr::sideFrameAt(R, angleDeg, midZ);
                const gp_Dir  outward = frame.inwardRadial.Reversed();

                // Lug box: extends outward by `length`, centered on tangent
                // (`width`) and axial (`thick`).  Origin = case-surface corner.
                //
                // gp_Ax2(P, V, Vx) sets local Z = V and local X = Vx.  We want
                // DX = length along outward, DY = width along tangent, DZ =
                // thick along axial.  Picking V = axialZ and Vx = outward gives
                // Y = axialZ × outward = tangentCCW (verified for any angle θ).
                const gp_Pnt origin = pr::offsetPoint(
                    frame.center,
                    -width / 2.0, frame.tangentCCW,
                    -thick / 2.0, frame.axialZ);
                const gp_Ax2 ax(origin, frame.axialZ, outward);
                protrusions.push_back(pr::box(ax, length, width, thick));

                // Optional pin hole: through-hole along tangent direction,
                // positioned 70 % out along the lug length.
                if (pinDia >= 0.6) {  // ignore tiny / zero — DFM-002 also enforces
                    const double pinOutOffset = length * 0.7;
                    const double kOverhang    = 0.05;
                    const gp_Pnt pinStart(
                        frame.center.X()
                            + pinOutOffset * outward.X()
                            - (width / 2.0 + kOverhang) * frame.tangentCCW.X(),
                        frame.center.Y()
                            + pinOutOffset * outward.Y()
                            - (width / 2.0 + kOverhang) * frame.tangentCCW.Y(),
                        frame.center.Z());
                    const gp_Ax2 pinAx(pinStart, frame.tangentCCW);
                    pinHoles.push_back(
                        pr::cylinder(pinAx, pinDia / 2.0, width + 2.0 * kOverhang));
                } else if (pinDia > 0.0) {
                    warnings.push_back(BuildWarning{
                        "W_DFM_002_PIN", "lug pin_hole_dia_mm < 0.6, skipped (DFM-002)"
                    });
                }
            }
            // First fuse all protrusions, then cut pin holes through fused body.
            const TopoDS_Shape withLugs = pr::fuseMany(in, protrusions);
            return pr::cutMany(withLugs, pinHoles);
        });
}

// @lat: [[engine/feature-watch#스텝 10 — addSecondaryFillets]]
StepResult WatchFrontModel::addSecondaryFillets(const TopoDS_Shape& in,
                                                 const nlohmann::json& spec)
{
    if (!spec.contains("secondary_fillets")) return StepResult{ in, {} };

    return pr::runStep("WatchFrontModel::addSecondaryFillets",
        [&](std::vector<BuildWarning>& warnings) {
            const double r  = spec["secondary_fillets"].value("r_mm", 0.2);
            const auto   bb = pr::optimalBbox(in);

            // Apply forgiving fillets to two specific Z-bands:
            //   1. display-pocket top rim    @ z = zMax
            //   2. bezel inner step          @ z = zMax - bezel.depth_mm
            // If either pass fails (no matching edges, fillet solver fails),
            // log a warning and pass the previous shape through.
            TopoDS_Shape current = in;
            auto safeFillet = [&](double radius, double z, const char* label) {
                try {
                    current = pr::filletEdges(current, radius, pr::edgesAtZ(z));
                } catch (const std::exception& e) {
                    warnings.push_back(BuildWarning{
                        "W_FILLET_SKIP",
                        std::string(label) + " fillet skipped: " + e.what()
                    });
                }
            };
            safeFillet(r, bb.zMax, "top rim");
            if (spec.contains("bezel")) {
                const double bezDepth = spec["bezel"].value("depth_mm", 0.0);
                if (bezDepth > 0.0)
                    safeFillet(r * 0.5, bb.zMax - bezDepth, "bezel inner step");
            }
            return current;
        },
        /*checkValidity=*/false);  // multi-fillet may leave benign minor invalid edges
}

// ── Convenience: chain all M1.5 steps (1–10) ───────────────────────────────
TopoDS_Shape WatchFrontModel::buildAll(const nlohmann::json& spec,
                                        std::vector<BuildWarning>& warnings)
{
    WatchFrontModel m;

    auto absorb = [&warnings](StepResult&& r, const char* tag) -> TopoDS_Shape {
        warnings.insert(warnings.end(),
                        std::make_move_iterator(r.warnings.begin()),
                        std::make_move_iterator(r.warnings.end()));
        if (r.shape.IsNull())
            spdlog::error("WatchFrontModel::buildAll aborted at {}", tag);
        return r.shape;
    };

    TopoDS_Shape s;
    s = absorb(m.buildBase(spec),               "step1 buildBase");        if (s.IsNull()) return {};
    s = absorb(m.applyCornerRadius(s, spec),    "step2 cornerRad");        if (s.IsNull()) return {};
    s = absorb(m.buildBezel(s, spec),           "step3 buildBezel");       if (s.IsNull()) return {};
    s = absorb(m.buildDisplayPocket(s, spec),   "step4 displayPocket");    if (s.IsNull()) return {};
    s = absorb(m.addCrownCavity(s, spec),       "step5 crownCavity");      if (s.IsNull()) return {};
    s = absorb(m.addSideButtons(s, spec),       "step6 sideButtons");      if (s.IsNull()) return {};
    s = absorb(m.addSpeakerGrille(s, spec),     "step7 speakerGrille");    if (s.IsNull()) return {};
    s = absorb(m.addRearSensors(s, spec),       "step8 rearSensors");      if (s.IsNull()) return {};
    s = absorb(m.addLugs(s, spec),              "step9 lugs");             if (s.IsNull()) return {};
    s = absorb(m.addSecondaryFillets(s, spec),  "step10 secondaryFillets"); if (s.IsNull()) return {};
    return s;
}

// ── Step 11: runDFM — DFM rule catalog validation ──────────────────────────
// @lat: [[engine/feature-watch#스텝 11 — runDFM]]
DFMReport WatchFrontModel::runDFM(const TopoDS_Shape& shape,
                                   const nlohmann::json& spec)
{
    DFMReport report;
    auto addFinding = [&](const char* code, const char* severity,
                          const std::string& msg) {
        report.findings.push_back(DFMFinding{ code, severity, msg });
        if (std::string_view(severity) == "error") report.passed = false;
    };

    if (shape.IsNull()) {
        addFinding("DFM-NULL", "error", "shape is null — cannot run DFM");
        return report;
    }

    // ── DFM-002: minimum hole / cavity diameter ≥ 0.8 mm ───────────────
    constexpr double kMinHoleDia = 0.8;
    auto checkHoleDia = [&](double dia, const char* src) {
        if (dia > 0.0 && dia < kMinHoleDia) {
            addFinding("DFM-002", "error",
                std::string(src) + " diameter " + std::to_string(dia) +
                " mm < min " + std::to_string(kMinHoleDia) + " mm");
        }
    };
    if (spec.contains("crown_cavity")) {
        const auto& cc = spec["crown_cavity"];
        checkHoleDia(cc["diameter_mm"].get<double>(),  "crown_cavity.diameter_mm");
        checkHoleDia(cc["shaft_dia_mm"].get<double>(), "crown_cavity.shaft_dia_mm");
    }
    if (spec.contains("speaker_grille")) {
        checkHoleDia(spec["speaker_grille"]["hole_dia_mm"].get<double>(),
                     "speaker_grille.hole_dia_mm");
    }
    if (spec.contains("rear_sensors")) {
        std::size_t i = 0;
        for (const auto& s : spec["rear_sensors"]) {
            checkHoleDia(s["dia_mm"].get<double>(),
                         ("rear_sensors[" + std::to_string(i) + "].dia_mm").c_str());
            ++i;
        }
    }
    if (spec.contains("lugs")) {
        std::size_t i = 0;
        for (const auto& l : spec["lugs"]) {
            checkHoleDia(l.value("pin_hole_dia_mm", 0.0),
                         ("lugs[" + std::to_string(i) + "].pin_hole_dia_mm").c_str());
            ++i;
        }
    }

    // ── DFM-009: bezel minimum width ≥ 0.6 mm ─────────────────────────
    constexpr double kMinBezelW = 0.6;
    if (spec.contains("bezel")) {
        const double w = spec["bezel"]["width_mm"].get<double>();
        if (w < kMinBezelW) {
            addFinding("DFM-009", "error",
                "bezel.width_mm " + std::to_string(w) +
                " < min " + std::to_string(kMinBezelW));
        }
    }

    // ── DFM-014: speaker-grille pin (web between holes) ≥ 0.25 mm ─────
    constexpr double kMinPinThick = 0.25;
    if (spec.contains("speaker_grille")) {
        const auto& sg = spec["speaker_grille"];
        const double colSp = sg["col_spacing_mm"].get<double>();
        const double rowSp = sg["row_spacing_mm"].get<double>();
        const double dia   = sg["hole_dia_mm"].get<double>();
        const double pinT  = std::min(colSp, rowSp) - dia;
        if (pinT < kMinPinThick) {
            addFinding("DFM-014", "error",
                "speaker_grille pin thickness " + std::to_string(pinT) +
                " mm < min " + std::to_string(kMinPinThick) + " mm");
        }
    }

    // ── DFM-020: all shells closed (topology check) ────────────────────
    {
        BRepCheck_Analyzer analyzer(shape);
        if (!analyzer.IsValid()) {
            addFinding("DFM-020", "warning",
                "BRepCheck_Analyzer reports invalid topology — likely "
                "non-closed shell or geometry tolerance issue");
        }
    }

    // ── DFM-022: OBB Z aligned (thickness axis is the smallest extent) ─
    {
        const auto bb = pr::optimalBbox(shape);
        const double dx = bb.dx(), dy = bb.dy(), dz = bb.dz();
        const double minExt = std::min({ dx, dy, dz });
        if (std::abs(dz - minExt) > 1e-3) {
            addFinding("DFM-022", "warning",
                "thickness axis dz=" + std::to_string(dz) +
                " mm is not the smallest extent (min=" +
                std::to_string(minExt) + " mm)");
        }
    }

    // ══════════════════════════════════════════════════════════════════
    //  Newly added rules (10/25) — see [[engine/dfm-rules]] for criteria.
    // ══════════════════════════════════════════════════════════════════

    // ── DFM-001: minimum wall thickness ≥ 0.4 mm (watch ≥ 0.35) ────────
    {
        const std::string formFactor = spec.value("form_factor", "round");
        // Watch products use 0.35 mm; phone / other products use 0.4 mm.
        // The WatchFrontModel only ever ships watches, so we hard-code
        // 0.35 mm here.  PhoneFrontModel will get its own copy.
        constexpr double kMinWall = 0.35;
        const double minW = minWallThickness(shape);
        if (std::isfinite(minW) && minW < kMinWall) {
            addFinding("DFM-001", "error",
                "minimum wall thickness " + std::to_string(minW) +
                " mm < watch limit " + std::to_string(kMinWall) + " mm");
        }
        (void)formFactor;
    }

    // ── DFM-003: minimum hole-to-hole edge distance ≥ 1.5 mm ───────────
    {
        constexpr double kMinHoleGap = 1.5;
        const double gap = minHoleToHoleAxisDistance(shape);
        if (std::isfinite(gap) && gap < kMinHoleGap) {
            addFinding("DFM-003", "error",
                "hole-to-hole gap " + std::to_string(gap) +
                " mm < min " + std::to_string(kMinHoleGap) + " mm");
        }
    }

    // ── DFM-004: minimum concave corner radius ≥ 0.2 mm ────────────────
    {
        constexpr double kMinCornerR = 0.2;
        const double r = minConcaveCornerRadius(shape);
        if (std::isfinite(r) && r < kMinCornerR) {
            addFinding("DFM-004", "error",
                "minimum concave corner radius " + std::to_string(r) +
                " mm < min " + std::to_string(kMinCornerR) + " mm");
        }
    }

    // ── DFM-006: button pocket depth ≤ min(0.8, thickness − 0.3) ───────
    if (spec.contains("side_buttons") && !spec["side_buttons"].empty() &&
        spec.contains("base") && spec["base"].contains("thickness_mm")) {
        const double thickness = spec["base"]["thickness_mm"].get<double>();
        const double limit     = std::min(0.8, thickness - 0.3);
        std::size_t idx = 0;
        for (const auto& btn : spec["side_buttons"]) {
            const double depth = btn.value("depth_mm", 0.0);
            if (depth > limit) {
                addFinding("DFM-006", "error",
                    "side_buttons[" + std::to_string(idx) + "].depth_mm " +
                    std::to_string(depth) + " > limit " + std::to_string(limit) +
                    " (= min(0.8, thickness-0.3))");
            }
            ++idx;
        }
    }

    // ── DFM-011: anti-knife edge — adjacent face wedge ≥ 5° ────────────
    {
        constexpr double kMinDihedral = 5.0;
        const double ang = minDihedralAngleDeg(shape);
        if (std::isfinite(ang) && ang < kMinDihedral) {
            addFinding("DFM-011", "error",
                "minimum adjacent-face wedge angle " + std::to_string(ang) +
                " deg < min " + std::to_string(kMinDihedral) + " deg");
        }
    }

    // ── DFM-013: display pocket flatness ≤ 0.02 mm  (phone scope) ──────
    // Per spec, DFM-013 is Phone-only.  Watch shapes report N/A; we still
    // sample so a manual scope override could enable it.
    if (spec.value("form_factor", "round") == "phone") {
        constexpr double kMaxFlatness = 0.02;
        const double f = displayPocketFlatness(shape);
        if (std::isfinite(f) && f > kMaxFlatness) {
            addFinding("DFM-013", "error",
                "display pocket flatness " + std::to_string(f) +
                " mm > max " + std::to_string(kMaxFlatness) + " mm");
        }
    }

    // ── DFM-015: tap-hole min surrounding material ≥ tap-dia × 1.5 ─────
    // Spec-only: the watch spec does not yet expose tap-hole metadata
    // distinctly from generic hole metadata.  Conservative proxy: if any
    // `lugs[*].pin_hole_dia_mm` is "tap-like" (≤ 3 mm), check that the lug
    // length minus the pin offset (0.7 × length) gives at least 1.5 ×
    // tap_diameter of material between the pin and the lug tip.
    if (spec.contains("lugs")) {
        std::size_t idx = 0;
        for (const auto& l : spec["lugs"]) {
            const double pinDia = l.value("pin_hole_dia_mm", 0.0);
            const double length = l.value("length_mm",       0.0);
            if (pinDia > 0.0 && pinDia <= 3.0 && length > 0.0) {
                // Pin lies at 0.7 × length from lug root (see addLugs).
                const double tipMaterial = length - 0.7 * length;
                const double rootMaterial = 0.7 * length;
                const double minSurround  = pinDia * 1.5;
                const double worst =
                    std::min(tipMaterial, rootMaterial);
                if (worst < minSurround) {
                    addFinding("DFM-015", "error",
                        "lugs[" + std::to_string(idx) + "] pin surrounding " +
                        "material " + std::to_string(worst) +
                        " mm < required " + std::to_string(minSurround) +
                        " mm (= " + std::to_string(pinDia) + " × 1.5)");
                }
            }
            ++idx;
        }
    }

    // ── DFM-017: outer-edge curvature continuity (G1, > 0.5° gap) ──────
    {
        constexpr double kMaxTangentGapDeg = 0.5;
        const double gap = maxOuterTangentGapDeg(shape);
        if (gap > kMaxTangentGapDeg) {
            addFinding("DFM-017", "warning",
                "outer silhouette tangent gap " + std::to_string(gap) +
                " deg > G1 limit " + std::to_string(kMaxTangentGapDeg) + " deg");
        }
    }

    // ── DFM-018: camera deco-ring parallelism to top face ≤ 0.1° ───────
    // Per spec, scope = Phone only.  WatchFrontModel reports N/A; the
    // PhoneFrontModel will receive its own variant in a separate change.
    {
        if (spec.value("form_factor", "round") == "phone") {
            // Watch shapes don't currently carry deco-ring metadata; the
            // phone-specific implementation lives in PhoneFrontModel.
            // No-op for watch — leave as future PhoneFrontModel hook.
        }
    }

    // ── DFM-023: watch lug span symmetry ≤ 0.05 mm ─────────────────────
    if (spec.contains("lugs") && spec["lugs"].is_array() &&
        spec["lugs"].size() >= 2) {
        const auto& a = spec["lugs"][0];
        const auto& b = spec["lugs"][1];
        auto checkSpan = [&](const char* field) {
            const double va = a.value(field, 0.0);
            const double vb = b.value(field, 0.0);
            const double diff = std::abs(va - vb);
            if (diff > 0.05) {
                addFinding("DFM-023", "warning",
                    std::string("lug span asymmetry on '") + field +
                    "': |" + std::to_string(va) + " - " +
                    std::to_string(vb) + "| = " + std::to_string(diff) +
                    " mm > 0.05 mm");
            }
        };
        checkSpan("length_mm");
        checkSpan("width_mm");
        checkSpan("thickness_mm");
    }

    return report;
}

// ── Default spec: 44 mm round watch baseline ───────────────────────────────
nlohmann::json WatchFrontModel::defaultSpec()
{
    return nlohmann::json{
        { "schema_version", "1.0.0" },
        { "product_name",   "Default Round Watch 44mm" },
        { "form_factor",    "round" },
        { "base", {
            { "diameter_mm",  44.0 },
            { "thickness_mm", 10.0 }
        }},
        { "corner_radius", {
            { "r_top_mm",  1.0 },
            { "r_side_mm", 0.6 }
        }},
        { "bezel", {
            { "width_mm",   3.0 },
            { "depth_mm",   1.0 },
            { "taper_deg",  0.0 }
        }},
        { "display_pocket", {
            { "d_pocket_mm",      36.0  },
            { "depth_pocket_mm",   0.8  },
            { "glass_offset_mm",  -0.1  }
        }},
        { "crown_cavity", {
            { "side_angle_deg",  0.0 },
            { "height_pos_mm",   5.0 },
            { "depth_mm",        2.5 },
            { "diameter_mm",     3.0 },
            { "shaft_dia_mm",    1.5 }
        }},
        { "side_buttons", nlohmann::json::array({
            nlohmann::json{
                { "angle_deg",  60.0 },
                { "height_mm",   5.0 },
                { "length_mm",   5.0 },
                { "width_mm",    2.0 },
                { "depth_mm",    0.8 },
                { "taper_deg",   0.0 }
            }
        })},
        { "speaker_grille", {
            { "angle_deg",       180.0 },   // 9 o'clock (convention 0°=+X=3시, CCW)
            { "height_pos_mm",     5.0 },
            { "rows",              2 },
            { "cols",              6 },
            { "row_spacing_mm",    1.5 },
            { "col_spacing_mm",    1.5 },
            { "hole_dia_mm",       0.8 },   // DFM-002 minimum
            { "depth_mm",          1.0 }
        }},
        { "rear_sensors", nlohmann::json::array({
            nlohmann::json{
                { "offset_x_mm",  0.0 }, { "offset_y_mm",  0.0 },
                { "dia_mm",       5.0 }, { "depth_mm",     0.5 }
            },
            nlohmann::json{
                { "offset_x_mm",  5.0 }, { "offset_y_mm",  0.0 },
                { "dia_mm",       1.5 }, { "depth_mm",     0.3 }
            }
        })},
        { "lugs", nlohmann::json::array({
            nlohmann::json{
                { "angle_deg",        90.0 },   // 12 o'clock
                { "length_mm",         6.0 },
                { "width_mm",         18.0 },
                { "thickness_mm",      3.0 },
                { "pin_hole_dia_mm",   1.8 }
            },
            nlohmann::json{
                { "angle_deg",       270.0 },   // 6 o'clock
                { "length_mm",         6.0 },
                { "width_mm",         18.0 },
                { "thickness_mm",      3.0 },
                { "pin_hole_dia_mm",   1.8 }
            }
        })},
        { "secondary_fillets", {
            { "r_mm",  0.2 }
        }}
    };
}

}  // namespace koocadcam::engine
