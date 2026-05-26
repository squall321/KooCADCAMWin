#pragma once
// @lat: [[engine/parametric-templates#primitives — Bbox3d]]
//
// Optimal bounding-box helper.  `BRepBndLib::AddOptimal` walks the actual
// surface (vs the NURBS control polygon `BRepBndLib::Add` gives), so callers
// get the geometric extents seen in the viewer — see process/occt8-migration
// -cookbook BT-6.

#include <Bnd_Box.hxx>
#include <BRepBndLib.hxx>
#include <TopoDS_Shape.hxx>
#include <gp_Pnt.hxx>

#include <algorithm>

namespace koocadcam::engine::prim {

struct Bbox3d
{
    double xMin = 0.0, yMin = 0.0, zMin = 0.0;
    double xMax = 0.0, yMax = 0.0, zMax = 0.0;

    double dx() const { return xMax - xMin; }
    double dy() const { return yMax - yMin; }
    double dz() const { return zMax - zMin; }

    // Largest XY half-extent — the outer radius of any axisymmetric body
    // whose footprint fits in this bbox.
    double outerRadiusXY() const { return std::max(dx(), dy()) / 2.0; }

    gp_Pnt center() const
    {
        return gp_Pnt((xMin + xMax) / 2.0, (yMin + yMax) / 2.0, (zMin + zMax) / 2.0);
    }
    gp_Pnt topCenter() const
    {
        return gp_Pnt((xMin + xMax) / 2.0, (yMin + yMax) / 2.0, zMax);
    }
    gp_Pnt bottomCenter() const
    {
        return gp_Pnt((xMin + xMax) / 2.0, (yMin + yMax) / 2.0, zMin);
    }
};

inline Bbox3d optimalBbox(const TopoDS_Shape& s)
{
    Bbox3d b;
    if (s.IsNull()) return b;
    Bnd_Box box;
    BRepBndLib::AddOptimal(s, box);
    if (box.IsVoid()) return b;
    box.Get(b.xMin, b.yMin, b.zMin, b.xMax, b.yMax, b.zMax);
    return b;
}

}  // namespace koocadcam::engine::prim
