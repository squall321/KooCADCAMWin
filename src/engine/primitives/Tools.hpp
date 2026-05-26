#pragma once
// @lat: [[engine/parametric-templates#primitives — Tool shapes]]
//
// Factory functions for the OCCT primitive shapes most commonly used as
// Boolean "tools" against the workpiece: cylinders, boxes, cone frusta,
// annular rings.  Each function throws `Standard_Failure` on build failure
// so that `prim::runStep` can convert it to a BuildWarning.

#include "Fillets.hpp"
#include "Frames.hpp"

#include <BRepAlgoAPI_Cut.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeCone.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <Standard_Failure.hxx>
#include <TopoDS_Shape.hxx>
#include <gp.hxx>
#include <gp_Ax2.hxx>
#include <gp_Pnt.hxx>

#include <algorithm>

namespace koocadcam::engine::prim {

inline TopoDS_Shape cylinder(const gp_Ax2& axis, double radius, double height)
{
    BRepPrimAPI_MakeCylinder m(axis, radius, height);
    m.Build();
    if (!m.IsDone()) throw Standard_Failure("prim::cylinder: build failed");
    return m.Shape();
}

inline TopoDS_Shape coneFrustum(const gp_Ax2& axis, double r1, double r2, double height)
{
    BRepPrimAPI_MakeCone m(axis, r1, r2, height);
    m.Build();
    if (!m.IsDone()) throw Standard_Failure("prim::coneFrustum: build failed");
    return m.Shape();
}

inline TopoDS_Shape box(const gp_Ax2& axis, double dx, double dy, double dz)
{
    BRepPrimAPI_MakeBox m(axis, dx, dy, dz);
    m.Build();
    if (!m.IsDone()) throw Standard_Failure("prim::box: build failed");
    return m.Shape();
}

// Annular cylindrical ring = outer cylinder − inner cylinder (both straight).
inline TopoDS_Shape annularRing(const gp_Ax2& axis,
                                double outerR, double innerR, double height)
{
    if (innerR >= outerR)
        throw Standard_Failure("prim::annularRing: innerR >= outerR");
    const TopoDS_Shape outer = cylinder(axis, outerR, height);
    const TopoDS_Shape inner = cylinder(axis, innerR, height);
    BRepAlgoAPI_Cut op(outer, inner);
    op.Build();
    if (!op.IsDone()) throw Standard_Failure("prim::annularRing: cut failed");
    return op.Shape();
}

// Annular ring whose outer wall is a cone frustum (r1 at bottom, r2 at top),
// inner wall a straight cylinder of `innerR`.  Used for tapered bezels.
inline TopoDS_Shape annularConeRing(const gp_Ax2& axis,
                                    double outerR1Bottom, double outerR2Top,
                                    double innerR, double height)
{
    if (innerR >= std::min(outerR1Bottom, outerR2Top))
        throw Standard_Failure("prim::annularConeRing: innerR collides with outer");
    const TopoDS_Shape outer = coneFrustum(axis, outerR1Bottom, outerR2Top, height);
    const TopoDS_Shape inner = cylinder   (axis, innerR, height);
    BRepAlgoAPI_Cut op(outer, inner);
    op.Build();
    if (!op.IsDone()) throw Standard_Failure("prim::annularConeRing: cut failed");
    return op.Shape();
}

// Rectangular pocket centred on a side-frame's `center` in the tangent
// (length) and axial (width) directions, extending `depth` inward.  Side
// buttons, lateral speaker grilles, lateral SIM trays all reuse this.
//
// gp_Ax2(P, V, Vx) interprets V as the LOCAL Z direction (the "main"
// direction along which DZ extends in BRepPrimAPI_MakeBox(P, DX, DY, DZ))
// and Vx as the LOCAL X direction.  We pick MainDir = axialZ so that DZ =
// width extrudes along the case axis, XDir = inwardRadial so that DX =
// depth extrudes into the case, leaving YDir = MainDir × XDir = tangentCCW
// so DY = length extends along the side surface.
inline TopoDS_Shape sidePocketBox(const SideFrame& frame,
                                  double depth, double length, double width)
{
    const gp_Pnt origin = offsetPoint(
        frame.center,
        -length / 2.0, frame.tangentCCW,
        -width  / 2.0, frame.axialZ);
    const gp_Ax2 ax(origin, frame.axialZ, frame.inwardRadial);
    return box(ax, depth, length, width);
}

// Rounded-rectangle pocket cut tool, axis-aligned, going UP from `bottomCenter`
// to `bottomCenter.Z() + depth`.  The 4 vertical corner edges are filleted by
// `cornerR` (skipped if cornerR <= 0).  Used for phone/tablet screen pockets,
// large rectangular recesses on any flat top face.
//
// `bottomCenter` is the XY centre of the pocket footprint at the cut tool's
// bottom Z; callers typically pass `(0, 0, topZ - depth)` for a centred pocket
// on a top face, or non-zero XY for an offset pocket.
inline TopoDS_Shape roundedRectPocketTool(const gp_Pnt& bottomCenter,
                                          double sx, double sy, double depth,
                                          double cornerR)
{
    const gp_Pnt origin(bottomCenter.X() - sx / 2.0,
                        bottomCenter.Y() - sy / 2.0,
                        bottomCenter.Z());
    const TopoDS_Shape baseBox = box(gp_Ax2(origin, gp::DZ()), sx, sy, depth);
    if (cornerR <= 0.0) return baseBox;

    return filletEdges(baseBox, cornerR,
        verticalCornerEdges(
            bottomCenter.X(), bottomCenter.Y(),
            sx / 2.0,         sy / 2.0,
            bottomCenter.Z(), bottomCenter.Z() + depth));
}

}  // namespace koocadcam::engine::prim
