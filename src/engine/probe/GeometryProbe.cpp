// @lat: [[engine/dfm-rules#GeometryProbe]]
// Product-agnostic DFM geometry probes (B5.2-B5.3).
//
// The dihedral logic is ported verbatim from the former file-local
// WatchFrontModel.cpp helper so DFM-011 behaviour is bit-for-bit
// unchanged: same midpoint normal sampling, same planar-planar gate,
// same 175° tangency skip.

#include "GeometryProbe.hpp"

#include <BRepAdaptor_Surface.hxx>
#include <BRepExtrema_DistShapeShape.hxx>
#include <NCollection_IndexedDataMap.hxx>
#include <NCollection_List.hxx>
#include <Standard_Failure.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <NCollection_IndexedMap.hxx>
#include <TopTools_ShapeMapHasher.hxx>
#include <TopoDS.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>

#include <algorithm>
#include <cmath>
#include <limits>

namespace koocadcam::engine::probe {

namespace {

// Sample a face normal at the centre of its UV parameter range.
// Returns false if the face is degenerate / can't be evaluated.
// (Verbatim port of the WatchFrontModel.cpp helper of the same name.)
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

}  // namespace

double dihedralWedgeDeg(const TopoDS_Face& fA, const TopoDS_Face& fB)
{
    constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
    // The midpoint-normal wedge below is only EXACT for two planar faces
    // (a plane's normal is constant, so the midpoint normal equals the
    // normal at the shared edge).  For curved faces (fillets, cylinders)
    // the midpoint normal diverges from the edge normal, producing
    // spurious sub-5° "knife edges".  Restrict the probe to planar-planar
    // pairs — the genuine anti-knife-edge concern for machined flats.
    // (Curved-face dihedrals need edge-evaluated normals: future work.)
    try {
        if (BRepAdaptor_Surface(fA).GetType() != GeomAbs_Plane) return kNaN;
        if (BRepAdaptor_Surface(fB).GetType() != GeomAbs_Plane) return kNaN;
    } catch (...) { return kNaN; }
    gp_Dir nA, nB;
    gp_Pnt pA, pB;
    if (!sampleFaceNormalAndPoint(fA, nA, pA)) return kNaN;
    if (!sampleFaceNormalAndPoint(fB, nB, pB)) return kNaN;
    (void)pA; (void)pB;
    const double dot = std::clamp(
        nA.X() * nB.X() + nA.Y() * nB.Y() + nA.Z() * nB.Z(), -1.0, 1.0);
    // Knife-edge angle = exterior wedge angle = 180° - acos(dot).
    // dot ≈ +1 → faces near-coplanar → wedge ≈ 180° (fine);
    // dot ≈ -1 → faces back-to-back  → wedge ≈ 0°  (knife).
    const double interiorDeg = std::acos(dot) * 180.0 / M_PI;
    return 180.0 - interiorDeg;
}

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
        const double wedgeDeg = dihedralWedgeDeg(fA, fB);
        if (std::isnan(wedgeDeg)) continue;   // curved pair / degenerate face
        // Skip tangent-continuous edges where wedge ≈ 180° (no knife).
        if (wedgeDeg > 175.0) continue;
        if (wedgeDeg < minAng) minAng = wedgeDeg;
    }
    return minAng;
}

double holeToOuterMinDistance(const TopoDS_Shape& shape,
                              const TopoDS_Face& holeFace)
{
    constexpr double kInf = std::numeric_limits<double>::infinity();
    // Only cylindrical hole walls are measurable in B5.3 scope.
    try {
        if (BRepAdaptor_Surface(holeFace).GetType() != GeomAbs_Cylinder)
            return kInf;
    } catch (...) { return kInf; }

    // Edges of the hole wall — used to exclude faces that merely border
    // the hole (the pierced top/bottom faces, chamfer rings, pocket
    // floors): their distance to the wall is trivially 0 and says nothing
    // about the surrounding stock.
    NCollection_IndexedMap<TopoDS_Shape, TopTools_ShapeMapHasher> holeEdges;
    TopExp::MapShapes(holeFace, TopAbs_EDGE, holeEdges);

    double minD = kInf;
    for (TopExp_Explorer ex(shape, TopAbs_FACE); ex.More(); ex.Next()) {
        const TopoDS_Face& f = TopoDS::Face(ex.Current());
        if (f.IsSame(holeFace)) continue;
        bool adjacent = false;
        for (TopExp_Explorer ee(f, TopAbs_EDGE); ee.More() && !adjacent; ee.Next())
            if (holeEdges.Contains(ee.Current())) adjacent = true;
        if (adjacent) continue;
        try {
            BRepExtrema_DistShapeShape dss(holeFace, f);
            if (dss.IsDone() && dss.NbSolution() > 0) {
                const double d = dss.Value();
                // >1e-6 guards against tangent-contact faces that touch the
                // wall without sharing an edge (same filter as DFM-001).
                if (d > 1e-6 && d < minD) minD = d;
            }
        } catch (...) { /* skip flaky pair */ }
    }
    return minD;
}

}  // namespace koocadcam::engine::probe
