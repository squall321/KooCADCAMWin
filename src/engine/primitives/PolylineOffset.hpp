#pragma once
// @lat: [[engine/parametric-templates#primitives — PolylineOffset]]
//
// Polyline channel cutter — builds a swept-prismatic shape that, when cut
// from a workpiece, leaves a channel of `channel_width_mm` width and
// `depth_mm` depth along a 2D XY-plane polyline.  Used by `engrave_path`.
//
// Implementation strategy (slice 4):
//   We use the STADIUM CHAIN approach, which is robust against any
//   polyline shape (sharp corners, near-collinear runs, very short
//   segments) and produces a valid solid via BRepAlgoAPI_Fuse.  For each
//   pair of consecutive waypoints we union:
//     - one cylinder at each endpoint (radius = channel_width/2)
//     - one box connecting the two cylinders (a "stadium")
//
//   This is geometrically equivalent to an offset wire extruded down by
//   depth_mm, with the bonus of natural end-cap rounding (matching a real
//   end-mill footprint).  It is what `engrave_path` did inline before this
//   primitive existed; consolidating it here means the skill body is short.
//
// A theoretically cleaner approach would be BRepOffsetAPI_MakeOffset on
// the 2D polyline + BRepPrimAPI_MakePrism, but OCCT 8.0's MakeOffset is
// notoriously fragile for short/sharp polylines (it can produce
// self-intersecting offset wires).  The stadium chain has no such failure
// mode and is the right primitive for our use case (engraving channels
// rarely have wide-radius elbows that would benefit from a single offset
// wire).

#include "Cuts.hpp"
#include "Tools.hpp"

#include <Standard_Failure.hxx>
#include <TopoDS_Shape.hxx>
#include <gp.hxx>
#include <gp_Ax2.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>

#include <algorithm>
#include <array>
#include <cmath>
#include <vector>

namespace koocadcam::engine::prim {

// Build a channel cutter along a 2D polyline at z = base_z.
//   waypoints:        (x, y) pairs in mm; size must be >= 2.
//   channel_width_mm: width of the engraved channel (= tool diameter), > 0.
//   depth_mm:         depth of the cutter (extending downward from base_z), > 0.
//   base_z:           Z coordinate of the cutter's TOP face.  The cutter
//                     extends from base_z down to base_z - depth_mm.
//
// Returns a single TopoDS_Shape (compound or solid) suitable for use as
// the `tool` shape in `prim::cut`.
inline TopoDS_Shape offsetPolylineCutter(const std::vector<std::array<double, 2>>& waypoints,
                                         double channel_width_mm,
                                         double depth_mm,
                                         double base_z)
{
    if (waypoints.size() < 2)
        throw Standard_Failure("offsetPolylineCutter: need at least 2 waypoints");
    if (channel_width_mm <= 0.0)
        throw Standard_Failure("offsetPolylineCutter: channel_width_mm must be > 0");
    if (depth_mm <= 0.0)
        throw Standard_Failure("offsetPolylineCutter: depth_mm must be > 0");

    const double radius = channel_width_mm / 2.0;
    const gp_Dir downZ(0.0, 0.0, -1.0);

    std::vector<TopoDS_Shape> parts;
    parts.reserve(waypoints.size() * 2);

    auto addCylinder = [&](double x, double y) {
        const gp_Pnt top(x, y, base_z);
        const gp_Ax2 ax(top, downZ);
        parts.push_back(cylinder(ax, radius, depth_mm));
    };

    // 1) End-cap cylinder at every waypoint.
    for (const auto& w : waypoints) addCylinder(w[0], w[1]);

    // 2) Box (stadium body) connecting each consecutive pair.
    for (size_t i = 0; i + 1 < waypoints.size(); ++i) {
        const double x0 = waypoints[i][0];
        const double y0 = waypoints[i][1];
        const double x1 = waypoints[i + 1][0];
        const double y1 = waypoints[i + 1][1];
        const double segLen = std::hypot(x1 - x0, y1 - y0);
        if (segLen < 1e-9) continue;   // zero-length segment → cylinders suffice

        // Box orientation: local X = unit(end - start), local Y = perpendicular
        // in XY plane, local Z = downward (so DZ extends down by depth_mm).
        //
        // gp_Ax2(P, mainDir, xDir): mainDir = local Z, xDir = local X.
        // BRepPrimAPI_MakeBox(P, dx, dy, dz) where dx, dy, dz extend along
        // (xDir, yDir = mainDir × xDir, mainDir).
        //
        // We want: dx = segLen (along the polyline direction), dy = width,
        // dz = depth.  Origin = (x0, y0, base_z) shifted by -width/2 along yDir.
        const gp_Dir xDir(static_cast<double>((x1 - x0) / segLen),
                          static_cast<double>((y1 - y0) / segLen),
                          0.0);
        // yDir = downZ × xDir = (-xDir.Y, xDir.X, 0) for downZ = (0,0,-1)? Let
        // me derive: (0,0,-1) × (a,b,0) = (0·0 - (-1)·b, (-1)·a - 0·0, 0·b - 0·a)
        //          = (b, -a, 0).  We want a +Y-like in-plane direction so we
        // build the origin by shifting by -width/2 along that perpendicular.
        const gp_Dir yDir(static_cast<double>(xDir.Y()),
                          static_cast<double>(-xDir.X()),
                          0.0);

        const gp_Pnt origin(
            x0 - (channel_width_mm / 2.0) * yDir.X(),
            y0 - (channel_width_mm / 2.0) * yDir.Y(),
            base_z);
        const gp_Ax2 ax(origin, downZ, xDir);
        parts.push_back(box(ax, segLen, channel_width_mm, depth_mm));
    }

    if (parts.empty())
        throw Standard_Failure("offsetPolylineCutter: no parts produced");

    // Fuse all parts into one solid.  We could return a compound and let
    // BRepAlgoAPI_Cut handle it directly, but a fused single solid is much
    // cleaner for downstream operations (and the fuse cost is negligible
    // for typical engrave paths).
    std::vector<TopoDS_Shape> tail(parts.begin() + 1, parts.end());
    return fuseMany(parts.front(), tail);
}

}  // namespace koocadcam::engine::prim
