#pragma once
// @lat: [[engine/parametric-templates#primitives — HelicalSweep]]
//
// Helical sweep primitive — builds a swept solid that follows a helix.
// Used by `tap_thread` (subtract from a pilot bore to leave an internal
// thread) and `thread_mill` (subtract for internal threads, or fuse onto
// an external cylinder for an external thread).
//
// Strategy:
//   1. Sample N points along the helix `(R·cosθ, R·sinθ, pitch·θ/2π)`
//      transformed into the caller's `axis` frame.
//   2. Build a single C^2 B-spline through those points → the spine wire.
//   3. Build a triangular V-profile (60° ISO metric thread cross-section)
//      perpendicular to the spine tangent at the spine start point.
//   4. `BRepOffsetAPI_MakePipeShell` sweeps the profile along the spine.
//
// Fallback:
//   `BRepOffsetAPI_MakePipeShell` is notoriously fragile when the spine
//   is highly curved (which a thin tight helix is).  If the pipe build
//   fails, we fall back to `prim::annularRing` — a coarse straight ring
//   of identical major/minor radius and axial length.  This still records
//   the thread intent; downstream tooling treats the geometry as a
//   placeholder.  The cpp file logs whether the swept path or the fallback
//   was used.

#include "Tools.hpp"

#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <BRepOffsetAPI_MakePipeShell.hxx>
#include <GeomAPI_PointsToBSpline.hxx>
#include <Geom_BSplineCurve.hxx>
#include <NCollection_Array1.hxx>
#include <Standard_Failure.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Shape.hxx>
#include <TopoDS_Wire.hxx>
#include <gp_Ax1.hxx>
#include <gp_Ax2.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>

#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::engine::prim {

namespace detail {

// Build the helical spine wire: a B-spline interpolating N samples taken
// along `(R cosθ, R sinθ, pitch·θ/2π)` in axis local coords, then
// transformed into world coords via the axis frame.
inline TopoDS_Wire buildHelicalSpine(const gp_Ax1& axis,
                                     double pitch_mm,
                                     double length_mm,
                                     double radius_mm,
                                     int    samples)
{
    // Build local frame: x_loc, y_loc perpendicular to axis.Direction(), z_loc.
    const gp_Dir z_loc = axis.Direction();
    gp_Dir x_loc;
    {
        // Pick an arbitrary direction not parallel to z_loc.
        const gp_Dir seed = (std::abs(z_loc.Z()) < 0.9) ? gp_Dir(0, 0, 1) : gp_Dir(1, 0, 0);
        gp_Vec vx = gp_Vec(seed).Crossed(gp_Vec(z_loc));
        if (vx.Magnitude() < 1e-9) {
            // Fallback: any axis transformation should not actually trigger this.
            vx = gp_Vec(1, 0, 0);
        }
        x_loc = gp_Dir(vx);
    }
    const gp_Dir y_loc(gp_Vec(z_loc).Crossed(gp_Vec(x_loc)));

    const double thetaMax = 2.0 * M_PI * (length_mm / pitch_mm);
    const int N = std::max(samples, 8);

    NCollection_Array1<gp_Pnt> pts(1, N);
    for (int i = 0; i < N; ++i) {
        const double t = static_cast<double>(i) / static_cast<double>(N - 1);
        const double th = t * thetaMax;
        const double c  = std::cos(th);
        const double s  = std::sin(th);
        const double z  = pitch_mm * th / (2.0 * M_PI);
        // Point in world coords: axis.Location() + R·c·x_loc + R·s·y_loc + z·z_loc
        const gp_Pnt& O = axis.Location();
        const gp_Pnt p(
            O.X() + radius_mm * (c * x_loc.X() + s * y_loc.X()) + z * z_loc.X(),
            O.Y() + radius_mm * (c * x_loc.Y() + s * y_loc.Y()) + z * z_loc.Y(),
            O.Z() + radius_mm * (c * x_loc.Z() + s * y_loc.Z()) + z * z_loc.Z());
        pts.SetValue(i + 1, p);
    }

    GeomAPI_PointsToBSpline bs(pts);
    Handle(Geom_BSplineCurve) curve = bs.Curve();
    if (curve.IsNull())
        throw Standard_Failure("HelicalSweep: PointsToBSpline produced null curve");

    BRepBuilderAPI_MakeEdge mkEdge(curve);
    mkEdge.Build();
    if (!mkEdge.IsDone())
        throw Standard_Failure("HelicalSweep: spine edge build failed");

    BRepBuilderAPI_MakeWire mkWire(mkEdge.Edge());
    mkWire.Build();
    if (!mkWire.IsDone())
        throw Standard_Failure("HelicalSweep: spine wire build failed");

    return mkWire.Wire();
}

// Build a small triangular V-profile face perpendicular to the spine's
// tangent at parameter θ = 0 (the first sample).  V-profile vertices:
//   - p_apex at the major radius (R, 0, 0) in local coords (the bore wall)
//   - p_lower at (R + thread_depth, -pitch/4, 0)  (lower outer V vertex)
//   - p_upper at (R + thread_depth, +pitch/4, 0)  (upper outer V vertex)
//
// The V-profile's inner apex sits AT the bore wall (radius R); the two
// outer vertices poke OUTWARD into the surrounding material by
// `thread_depth_mm`.  Sweeping this triangle along the helix produces
// a helical "ridge" of material that, when SUBTRACTED from the workpiece
// (around an existing pilot bore), carves a helical V-groove into the
// bore wall — the canonical tap-thread groove.
//
// The face lies in the plane spanned by the axial direction (= local Z
// of the V) and the radial outward direction (= local X of the V), i.e.
// the plane containing the helix axis at θ = 0.
inline TopoDS_Shape buildVProfileFace(const gp_Ax1& axis,
                                      double pitch_mm,
                                      double radius_mm,
                                      double thread_depth_mm)
{
    // Same local frame as the spine.
    const gp_Dir z_loc = axis.Direction();
    gp_Dir x_loc;
    {
        const gp_Dir seed = (std::abs(z_loc.Z()) < 0.9) ? gp_Dir(0, 0, 1) : gp_Dir(1, 0, 0);
        gp_Vec vx = gp_Vec(seed).Crossed(gp_Vec(z_loc));
        if (vx.Magnitude() < 1e-9) vx = gp_Vec(1, 0, 0);
        x_loc = gp_Dir(vx);
    }

    // The V-profile lives at θ = 0, so its apex is at `axis.Location() +
    // R · x_loc`.  Vertices (in world coords):
    const gp_Pnt& O = axis.Location();
    auto worldPoint = [&](double dr, double dz) -> gp_Pnt {
        // dr = signed offset along x_loc (positive = outward from axis)
        // dz = signed offset along z_loc (positive = up the helix)
        return gp_Pnt(
            O.X() + (radius_mm + dr) * x_loc.X() + dz * z_loc.X(),
            O.Y() + (radius_mm + dr) * x_loc.Y() + dz * z_loc.Y(),
            O.Z() + (radius_mm + dr) * x_loc.Z() + dz * z_loc.Z());
    };

    const gp_Pnt pOuter = worldPoint(0.0, 0.0);                         // apex on bore wall
    const gp_Pnt pLower = worldPoint(+thread_depth_mm, -pitch_mm / 4.0); // outer −Z vertex
    const gp_Pnt pUpper = worldPoint(+thread_depth_mm, +pitch_mm / 4.0); // outer +Z vertex

    BRepBuilderAPI_MakeWire wireMk;
    wireMk.Add(BRepBuilderAPI_MakeEdge(pOuter, pLower).Edge());
    wireMk.Add(BRepBuilderAPI_MakeEdge(pLower, pUpper).Edge());
    wireMk.Add(BRepBuilderAPI_MakeEdge(pUpper, pOuter).Edge());
    if (!wireMk.IsDone())
        throw Standard_Failure("HelicalSweep: V-profile wire build failed");

    BRepBuilderAPI_MakeFace faceMk(wireMk.Wire(), /*OnlyPlane=*/true);
    if (!faceMk.IsDone())
        throw Standard_Failure("HelicalSweep: V-profile face build failed");
    return faceMk.Face();
}

}  // namespace detail

// Build a helical thread solid:
//   - axis: defines the helix axis (origin + Z direction)
//   - pitch_mm: vertical distance per full turn (must be > 0)
//   - length_mm: total axial length of the helix (must be > 0)
//   - major_radius_mm: outer thread radius (must be > 0)
//   - thread_depth_mm: depth of the V-profile valley (must be > 0,
//                      typically pitch_mm * 0.65)
//
// Returns a solid representing the swept V-thread.  Subtract it from a
// pilot bore for an internal tap; fuse it onto a cylinder for an external
// thread mill.
//
// Implementation: B-spline helical spine + triangular V-profile +
// BRepOffsetAPI_MakePipeShell.  If the pipe build fails (which can happen
// for very tight helices in OCCT 8.0), falls back to a coarse annular
// ring of (major_radius, major_radius - thread_depth) over length_mm.
inline TopoDS_Shape helicalSweep(const gp_Ax1& axis,
                                 double pitch_mm,
                                 double length_mm,
                                 double major_radius_mm,
                                 double thread_depth_mm)
{
    if (pitch_mm <= 0.0)
        throw Standard_Failure("helicalSweep: pitch_mm must be > 0");
    if (length_mm <= 0.0)
        throw Standard_Failure("helicalSweep: length_mm must be > 0");
    if (major_radius_mm <= 0.0)
        throw Standard_Failure("helicalSweep: major_radius_mm must be > 0");
    if (thread_depth_mm <= 0.0)
        throw Standard_Failure("helicalSweep: thread_depth_mm must be > 0");

    // Sample density: 32 per turn, minimum 24 overall (coarse but stable for tests).
    const int turns   = std::max(1, static_cast<int>(std::ceil(length_mm / pitch_mm)));
    const int samples = std::max(24, turns * 32);

    try {
        const TopoDS_Wire spine = detail::buildHelicalSpine(
            axis, pitch_mm, length_mm, major_radius_mm, samples);
        const TopoDS_Shape profile = detail::buildVProfileFace(
            axis, pitch_mm, major_radius_mm, thread_depth_mm);

        BRepOffsetAPI_MakePipeShell pipe(spine);
        // Auxiliary frenet-frame mode: orient the section along the spine's
        // tangent.  CorrectedFrenet is the most robust mode in OCCT 8.0 for
        // curved spines.
        pipe.SetMode(true);
        pipe.Add(profile, false, true);
        pipe.Build();
        if (!pipe.IsDone())
            throw Standard_Failure("helicalSweep: PipeShell.Build failed");
        if (!pipe.MakeSolid())
            throw Standard_Failure("helicalSweep: PipeShell.MakeSolid failed");

        const TopoDS_Shape result = pipe.Shape();
        if (result.IsNull())
            throw Standard_Failure("helicalSweep: PipeShell produced null shape");
        // Verify the swept solid is geometrically valid — PipeShell can
        // succeed yet leave self-intersecting / non-manifold geometry for
        // tight helices, in which case downstream Booleans crash.
        BRepCheck_Analyzer chk(result);
        if (!chk.IsValid())
            throw Standard_Failure("helicalSweep: PipeShell produced invalid BRep");
        spdlog::debug("helicalSweep ok (swept) pitch={} length={} R={} depth={}",
                      pitch_mm, length_mm, major_radius_mm, thread_depth_mm);
        return result;
    }
    catch (const Standard_Failure& e) {
        spdlog::warn("helicalSweep: swept-pipe build failed ({}), "
                     "falling back to annular ring", e.what());
    }
    catch (const std::exception& e) {
        spdlog::warn("helicalSweep: swept-pipe build threw ({}), "
                     "falling back to annular ring", e.what());
    }

    // ── Fallback: annular ring of (R, R + thread_depth) over `length_mm` ──
    // The "thread material" sits between the bore wall (R) and a slightly
    // larger outer cylinder (R + thread_depth).  Subtracting this from the
    // workpiece widens the bore by `thread_depth_mm` over the thread length
    // — a coarse but valid placeholder for the helical groove.
    //
    // The ring's axis frame: origin at axis.Location(), main direction =
    // axis.Direction().  gp_Ax2 takes (P, mainDir) — that's what we want.
    const gp_Ax2 ax2(axis.Location(), axis.Direction());
    return annularRing(ax2, major_radius_mm + thread_depth_mm, major_radius_mm, length_mm);
}

}  // namespace koocadcam::engine::prim
