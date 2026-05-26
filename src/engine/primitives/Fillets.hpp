#pragma once
// @lat: [[engine/parametric-templates#primitives — Fillets]]
//
// Edge-predicate fillet helpers.  Single-pass `filletEdgesMulti` lets a step
// apply different radii to different edge groups (e.g. r_top vs r_side) in
// one BRepFilletAPI_MakeFillet build, preserving the OCCT-native behaviour
// of the original WatchFrontModel::applyCornerRadius.

#include <BRepAdaptor_Curve.hxx>
#include <BRepFilletAPI_MakeFillet.hxx>
#include <Standard_Failure.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Shape.hxx>
#include <gp_Pnt.hxx>

#include <cmath>
#include <functional>
#include <initializer_list>
#include <vector>

namespace koocadcam::engine::prim {

// Predicate over (edge, mid-point on edge).  true → edge is filleted.
using EdgePredicate = std::function<bool(const TopoDS_Edge&, const gp_Pnt& midPt)>;

struct FilletSpec
{
    double        radius;
    EdgePredicate predicate;
};

// Single-radius fillet.  No-op if no edge matches.
inline TopoDS_Shape filletEdges(const TopoDS_Shape& shape, double radius,
                                const EdgePredicate& predicate)
{
    BRepFilletAPI_MakeFillet f(shape);
    int added = 0;
    for (TopExp_Explorer e(shape, TopAbs_EDGE); e.More(); e.Next()) {
        const TopoDS_Edge& edge = TopoDS::Edge(e.Current());
        BRepAdaptor_Curve curve(edge);
        const double mid = (curve.FirstParameter() + curve.LastParameter()) / 2.0;
        const gp_Pnt midPt = curve.Value(mid);
        if (predicate(edge, midPt)) {
            f.Add(radius, edge);
            ++added;
        }
    }
    if (added == 0) return shape;
    f.Build();
    if (!f.IsDone()) throw Standard_Failure("prim::filletEdges: build failed");
    return f.Shape();
}

// Multi-radius fillet — applies each spec in one fillet pass; first matching
// predicate wins per edge.  Use this when you need top-rim and bottom-rim
// fillets in the same build (the OCCT solver may relax differently than two
// sequential passes).
inline TopoDS_Shape filletEdgesMulti(const TopoDS_Shape& shape,
                                     std::initializer_list<FilletSpec> specs)
{
    BRepFilletAPI_MakeFillet f(shape);
    int added = 0;
    for (TopExp_Explorer e(shape, TopAbs_EDGE); e.More(); e.Next()) {
        const TopoDS_Edge& edge = TopoDS::Edge(e.Current());
        BRepAdaptor_Curve curve(edge);
        const double mid = (curve.FirstParameter() + curve.LastParameter()) / 2.0;
        const gp_Pnt midPt = curve.Value(mid);
        for (const auto& s : specs) {
            if (s.predicate(edge, midPt)) {
                f.Add(s.radius, edge);
                ++added;
                break;
            }
        }
    }
    if (added == 0) return shape;
    f.Build();
    if (!f.IsDone()) throw Standard_Failure("prim::filletEdgesMulti: build failed");
    return f.Shape();
}

// ── Common predicates ────────────────────────────────────────────────────

// Edges whose mid-point Z is within `tol` of `z`.  Top-rim / bottom-rim.
inline EdgePredicate edgesAtZ(double z, double tol = 1e-3)
{
    return [z, tol](const TopoDS_Edge&, const gp_Pnt& mp) {
        return std::abs(mp.Z() - z) < tol;
    };
}

// Vertical edges running along Z at the XY corners of a rectangular box.
// `hx` / `hy` = half-width / half-height (box is centred on Z axis).
inline EdgePredicate verticalCornerEdges(double hx, double hy, double thickness,
                                         double tol = 1e-3)
{
    return [hx, hy, thickness, tol](const TopoDS_Edge&, const gp_Pnt& mp) {
        const bool nearMidZ  = (mp.Z() > tol) && (mp.Z() < thickness - tol);
        const bool atCornerX = (std::abs(mp.X()) > hx - tol);
        const bool atCornerY = (std::abs(mp.Y()) > hy - tol);
        return nearMidZ && atCornerX && atCornerY;
    };
}

}  // namespace koocadcam::engine::prim
