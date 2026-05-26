#pragma once
// @lat: [[engine/parametric-templates#primitives — SideFrame]]
//
// Local coordinate frames for placing features on a body.
//
// Angle convention (KooCADCAM standard): 0° = +X (3 o'clock), CCW.  Watch
// 12-o'clock corresponds to 90°.  Phone-camera holes, watch crowns, side
// buttons all share this convention.

#include <gp.hxx>
#include <gp_Ax2.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>

#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::engine::prim {

// Local frame on the side surface of an axisymmetric body of radius `radius`.
struct SideFrame
{
    gp_Pnt center;        // (R cosθ, R sinθ, z) — point on side surface
    gp_Dir inwardRadial;  // x_loc — into the body (depth direction)
    gp_Dir tangentCCW;    // y_loc — along the side surface, CCW
    gp_Dir axialZ;        // z_loc — world +Z (case-thickness direction)

    // gp_Ax2 with origin at `center`, xDir = inwardRadial.
    // Use this for cylinders / cones that poke into the side surface.
    gp_Ax2 ax2InwardRadial() const { return gp_Ax2(center, inwardRadial); }
};

inline SideFrame sideFrameAt(double radius, double angleDeg, double z)
{
    const double a = angleDeg * M_PI / 180.0;
    const double c = std::cos(a);
    const double s = std::sin(a);

    SideFrame f;
    f.center       = gp_Pnt(radius * c, radius * s, z);
    f.inwardRadial = gp_Dir(-c, -s, 0.0);
    f.tangentCCW   = gp_Dir(-s,  c, 0.0);
    f.axialZ       = gp_Dir( 0.0, 0.0, 1.0);
    return f;
}

// Axis at (point, +Z) — for top-face features (display pocket, screw holes).
inline gp_Ax2 axisAtZ(const gp_Pnt& origin) { return gp_Ax2(origin, gp::DZ()); }

// 3-D linear combination: base + dx * xDir + dy * yDir.  Useful for offsetting
// a corner from a center point within a local 2-direction frame.
inline gp_Pnt offsetPoint(const gp_Pnt& base,
                          double dx, const gp_Dir& xDir,
                          double dy, const gp_Dir& yDir)
{
    return gp_Pnt(
        base.X() + dx * xDir.X() + dy * yDir.X(),
        base.Y() + dx * xDir.Y() + dy * yDir.Y(),
        base.Z() + dx * xDir.Z() + dy * yDir.Z());
}

}  // namespace koocadcam::engine::prim
