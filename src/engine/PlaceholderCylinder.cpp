#include "PlaceholderCylinder.hpp"

#include <BRepPrimAPI_MakeCylinder.hxx>
#include <Standard_Failure.hxx>
#include <gp_Ax2.hxx>
#include <gp_Pnt.hxx>
#include <gp_Dir.hxx>

#include <spdlog/spdlog.h>

namespace koocadcam::engine {

TopoDS_Shape PlaceholderCylinder::make(double diameterMm, double heightMm)
{
    if (diameterMm <= 0.0 || heightMm <= 0.0) {
        throw Standard_Failure("PlaceholderCylinder::make: diameter and height must be positive");
    }
    // Cylinder centered at origin, axis along +Z.
    gp_Ax2 axis(gp_Pnt(0.0, 0.0, -heightMm / 2.0), gp_Dir(0.0, 0.0, 1.0));
    BRepPrimAPI_MakeCylinder mk(axis, diameterMm / 2.0, heightMm);
    spdlog::debug("PlaceholderCylinder: ø{:.2f} mm × {:.2f} mm", diameterMm, heightMm);
    return mk.Shape();
}

}  // namespace koocadcam::engine
