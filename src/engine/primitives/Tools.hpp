#pragma once
// @lat: [[engine/parametric-templates#primitives — Tool shapes]]
//
// Factory functions for the OCCT primitive shapes most commonly used as
// Boolean "tools" against the workpiece: cylinders, boxes, cone frusta,
// annular rings.  Each function throws `Standard_Failure` on build failure
// so that `prim::runStep` can convert it to a BuildWarning.

#include "Frames.hpp"

#include <BRepAlgoAPI_Cut.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeCone.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <Standard_Failure.hxx>
#include <TopoDS_Shape.hxx>
#include <gp_Ax2.hxx>

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
inline TopoDS_Shape sidePocketBox(const SideFrame& frame,
                                  double depth, double length, double width)
{
    const gp_Pnt origin = offsetPoint(
        frame.center,
        -length / 2.0, frame.tangentCCW,
        -width  / 2.0, frame.axialZ);
    const gp_Ax2 ax(origin, frame.inwardRadial, frame.tangentCCW);
    return box(ax, depth, length, width);
}

}  // namespace koocadcam::engine::prim
