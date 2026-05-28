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
//   3. Build an ISO 60° metric thread cross-section profile (with crest /
//      root fillet radii where possible) perpendicular to the spine
//      tangent at the spine start point.
//   4. `BRepOffsetAPI_MakePipeShell` sweeps the profile along the spine.
//
// ISO 60° profile reference:
//   - Flank angle    : 60° (each flank 30° off the perpendicular)
//   - Thread depth   : (5/8) · cos(30°) · pitch ≈ 0.541 · pitch
//   - Root radius    : ~0.144 · pitch   (rounded valley, optional)
//   - Crest radius   : ~0.125 · pitch   (rounded peak, optional)
//   - In this implementation the depth is taken from the caller's
//     `thread_depth_mm` argument (so it stays compatible with the slice-4
//     pitch×0.65 rule-of-thumb used by tap_thread / thread_mill); the
//     pitch×0.541 ISO depth is documented for reference.
//
// Fallback cascade:
//   `BRepOffsetAPI_MakePipeShell` is notoriously fragile when the spine
//   is highly curved (which a thin tight helix is).  We try the rounded
//   ISO 60° profile first; if that PipeShell build fails (or the rounded
//   profile itself fails to build), we retry with the simpler closed-V
//   profile (no fillets); if THAT fails too, we fall back to
//   `prim::annularRing` — a coarse straight ring of identical major/minor
//   radius and axial length.  The returned `HelicalSweepProfile` tag tells
//   callers which path succeeded:
//     Iso60WithRadii   → "iso_60_with_radii"   (rounded crest + root fillets)
//     Iso60Simplified  → "iso_60_simplified"   (closed-V, no fillets)
//     AnnularFallback  → "annular_fallback"    (straight ring placeholder)

#include "Tools.hpp"

#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <BRepOffsetAPI_MakePipeShell.hxx>
#include <GeomAPI_PointsToBSpline.hxx>
#include <Geom_BSplineCurve.hxx>
#include <Geom_Circle.hxx>
#include <NCollection_Array1.hxx>
#include <Standard_Failure.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Shape.hxx>
#include <TopoDS_Wire.hxx>
#include <gp_Ax1.hxx>
#include <gp_Ax2.hxx>
#include <gp_Circ.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>

#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <string>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::engine {

// Returned alongside the swept shape so callers can record which profile
// path actually succeeded.  See helicalSweep() doc comment.
enum class HelicalSweepProfile
{
    Iso60WithRadii,    // rounded ISO 60° (4 segments: line + arc + line + arc)
    Iso60Simplified,   // closed-V (3 lines + closing edge, no fillets)
    AnnularFallback    // BRepOffsetAPI failed entirely; using straight ring
};

inline const char* helicalSweepProfileTag(HelicalSweepProfile p)
{
    switch (p) {
        case HelicalSweepProfile::Iso60WithRadii:  return "iso_60_with_radii";
        case HelicalSweepProfile::Iso60Simplified: return "iso_60_simplified";
        case HelicalSweepProfile::AnnularFallback: return "annular_fallback";
    }
    return "annular_fallback";   // unreachable; satisfies /WX exhaustiveness.
}

namespace prim {

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
    const gp_Dir z_loc = axis.Direction();
    gp_Dir x_loc;
    {
        const gp_Dir seed = (std::abs(z_loc.Z()) < 0.9) ? gp_Dir(0, 0, 1) : gp_Dir(1, 0, 0);
        gp_Vec vx = gp_Vec(seed).Crossed(gp_Vec(z_loc));
        if (vx.Magnitude() < 1e-9) vx = gp_Vec(1, 0, 0);
        x_loc = gp_Dir(vx);
    }
    const gp_Dir y_loc(gp_Vec(z_loc).Crossed(gp_Vec(x_loc)));

    const double thetaMax = 2.0 * M_PI * (length_mm / pitch_mm);
    const int N = std::max(samples, 8);

    NCollection_Array1<gp_Pnt> pts(1, N);
    for (int i = 0; i < N; ++i) {
        const double t  = static_cast<double>(i) / static_cast<double>(N - 1);
        const double th = t * thetaMax;
        const double c  = std::cos(th);
        const double s  = std::sin(th);
        const double z  = pitch_mm * th / (2.0 * M_PI);
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

// ── ISO 60° thread profile builders ──────────────────────────────────────
//
// The profile lives in the plane spanned by:
//   - local Z (= the helix axis direction)  — "axial" coordinate
//   - local X (radial outward from axis)    — "radial-depth" coordinate
//
// Profile sketch (radial outward to the right; axis vertical):
//
//                          (radial 0 = bore wall)         radial → +
//                                │
//   axial +pitch/4   ◄══valley══╕                          ← root corner
//                                │
//                                │  ↓  upper flank line
//                                │
//   axial +pitch/8   ──flank────╝                          ← peak corner
//                                │
//                                │  )  crest fillet (Rc ≈ 0.125 pitch)
//                                │
//   axial −pitch/8   ──flank────╗                          ← peak corner
//                                │
//                                │  ↑  lower flank line
//                                │
//   axial −pitch/4   ◄══valley══╛                          ← root corner
//                                │
//                                │  )  root fillet (Rr ≈ 0.144 pitch)
//                                       closing the wire across the valley
//
// Simplified V (fallback):
//   3-line triangle (valley → flank-left → flank-right → close)
//
// Rounded ISO 60°:
//   For the crest fillet to pass through both peak corners (flank_left at
//   axial = -pitch/8 and flank_right at axial = +pitch/8) on a circle of
//   radius Rc, we would need Rc ≥ pitch/8.  For root fillets, an arc
//   through both valley corners (at axial = ±pitch/4) on a circle of
//   radius Rr needs Rr ≥ pitch/4.  Standard ISO values (Rc = 0.125·pitch,
//   Rr = 0.144·pitch) do NOT satisfy these constraints — the standard
//   geometry has the fillet tangent-clip the flanks at a point inboard of
//   the corners.  We therefore RELAX the flank/valley anchor points:
//     - crest arc endpoints: axial = ±cMax where cMax = Rc·sin(60°)
//       (= 0.866·Rc ≈ 0.108·pitch) — the arc subtends 120°.
//     - root  arc endpoints: axial = ±aMax where aMax = Rr·sin(60°)
//       (= 0.866·Rr ≈ 0.125·pitch) — the arc subtends 120°.
//   The flanks then run from valley-relaxed → flank-relaxed (straight
//   lines).  This produces a faithful 60°-tangent ISO profile.

struct LocalFrame {
    gp_Dir x_loc;   // outward radial
    gp_Dir y_loc;   // in-plane tangent (profile plane normal)
    gp_Dir z_loc;   // axial (helix progression)
    gp_Pnt origin;  // axis.Location()
};

inline LocalFrame profileFrame(const gp_Ax1& axis)
{
    LocalFrame f;
    f.z_loc = axis.Direction();
    const gp_Dir seed = (std::abs(f.z_loc.Z()) < 0.9) ? gp_Dir(0, 0, 1) : gp_Dir(1, 0, 0);
    gp_Vec vx = gp_Vec(seed).Crossed(gp_Vec(f.z_loc));
    if (vx.Magnitude() < 1e-9) vx = gp_Vec(1, 0, 0);
    f.x_loc  = gp_Dir(vx);
    f.y_loc  = gp_Dir(gp_Vec(f.z_loc).Crossed(gp_Vec(f.x_loc)));
    f.origin = axis.Location();
    return f;
}

// Map a (radial, axial) profile-plane coord to a world point.
//   `radial`: signed outward from axis along x_loc (positive = outward)
//   `axial` : signed along z_loc            (positive = up the helix)
inline gp_Pnt profilePoint(const LocalFrame& f, double radius_mm,
                           double radial, double axial)
{
    return gp_Pnt(
        f.origin.X() + (radius_mm + radial) * f.x_loc.X() + axial * f.z_loc.X(),
        f.origin.Y() + (radius_mm + radial) * f.x_loc.Y() + axial * f.z_loc.Y(),
        f.origin.Z() + (radius_mm + radial) * f.x_loc.Z() + axial * f.z_loc.Z());
}

// Closed-V (no fillets) — 4 line edges forming a closed quadrilateral.
// Endpoint axial positions match the standard ±pitch/8 (flank peaks) and
// ±pitch/4 (valley corners).  Geometrically a planar quad whose two
// "outer" sides (the valley edge and the flank edge between the peaks)
// are vertical lines parallel to the helix axis.
inline TopoDS_Shape buildVProfileSimplified(const gp_Ax1& axis,
                                            double pitch_mm,
                                            double radius_mm,
                                            double thread_depth_mm)
{
    const LocalFrame f = profileFrame(axis);

    const gp_Pnt pValL = profilePoint(f, radius_mm, +thread_depth_mm, -pitch_mm / 4.0);
    const gp_Pnt pFlkL = profilePoint(f, radius_mm, 0.0,              -pitch_mm / 8.0);
    const gp_Pnt pFlkR = profilePoint(f, radius_mm, 0.0,              +pitch_mm / 8.0);
    const gp_Pnt pValR = profilePoint(f, radius_mm, +thread_depth_mm, +pitch_mm / 4.0);

    BRepBuilderAPI_MakeWire wireMk;
    wireMk.Add(BRepBuilderAPI_MakeEdge(pValL, pFlkL).Edge());   // lower flank
    wireMk.Add(BRepBuilderAPI_MakeEdge(pFlkL, pFlkR).Edge());   // crest    (no fillet)
    wireMk.Add(BRepBuilderAPI_MakeEdge(pFlkR, pValR).Edge());   // upper flank
    wireMk.Add(BRepBuilderAPI_MakeEdge(pValR, pValL).Edge());   // valley   (no fillet)
    if (!wireMk.IsDone())
        throw Standard_Failure("HelicalSweep: V-profile (simplified) wire build failed");

    BRepBuilderAPI_MakeFace faceMk(wireMk.Wire(), /*OnlyPlane=*/true);
    if (!faceMk.IsDone())
        throw Standard_Failure("HelicalSweep: V-profile (simplified) face build failed");
    return faceMk.Face();
}

// Rounded ISO 60° profile.  See header docstring for the relaxation rule.
//   4 boundary segments:
//     1. Line  pValLrel → pFlkLrel  (lower flank, straight)
//     2. Arc   pFlkLrel → pFlkRrel  (crest fillet, radius Rc, subtends 120°)
//     3. Line  pFlkRrel → pValRrel  (upper flank, straight)
//     4. Arc   pValRrel → pValLrel  (root  fillet, radius Rr, subtends 120°)
inline TopoDS_Shape buildIso60Profile(const gp_Ax1& axis,
                                      double pitch_mm,
                                      double radius_mm,
                                      double thread_depth_mm)
{
    const LocalFrame f = profileFrame(axis);

    const double Rc  = 0.125 * pitch_mm;            // crest fillet radius
    const double Rr  = 0.144 * pitch_mm;            // root  fillet radius
    const double k60 = std::sqrt(3.0) / 2.0;        // sin(60°)
    const double cMax = Rc * k60;                   // crest arc half-axial chord
    const double aMax = Rr * k60;                   // root  arc half-axial chord

    // Relaxed (tangent-respecting) profile vertices.
    const gp_Pnt pFlkLrel = profilePoint(f, radius_mm, 0.0,               -cMax);
    const gp_Pnt pFlkRrel = profilePoint(f, radius_mm, 0.0,               +cMax);
    const gp_Pnt pValLrel = profilePoint(f, radius_mm, +thread_depth_mm,  -aMax);
    const gp_Pnt pValRrel = profilePoint(f, radius_mm, +thread_depth_mm,  +aMax);

    // Plane normal for the in-plane arcs.  Sign doesn't matter for circle
    // definition; the arc endpoints pick the short arc automatically.
    const gp_Dir planeNormal = f.y_loc;

    // Crest fillet circle: center is INWARD of the bore wall (radial < 0)
    // at axial = 0.  dc = √(Rc² − cMax²) = Rc · cos(60°) = Rc / 2.
    const gp_Pnt crestCenter = profilePoint(f, radius_mm, -Rc / 2.0, 0.0);
    Handle(Geom_Circle) crestCircle = new Geom_Circle(
        gp_Ax2(crestCenter, planeNormal), Rc);

    // Root fillet circle: center is OUTWARD beyond the valley (radial =
    // depth + Rr/2) at axial = 0.  Same cos(60°) geometry.
    const gp_Pnt rootCenter = profilePoint(
        f, radius_mm, thread_depth_mm + Rr / 2.0, 0.0);
    Handle(Geom_Circle) rootCircle = new Geom_Circle(
        gp_Ax2(rootCenter, planeNormal), Rr);

    BRepBuilderAPI_MakeEdge mkFlankL  (pValLrel,    pFlkLrel);
    if (!mkFlankL.IsDone())
        throw Standard_Failure("HelicalSweep: ISO60 lower-flank line failed");

    BRepBuilderAPI_MakeEdge mkCrestArc(crestCircle, pFlkLrel, pFlkRrel);
    if (!mkCrestArc.IsDone())
        throw Standard_Failure("HelicalSweep: ISO60 crest arc failed");

    BRepBuilderAPI_MakeEdge mkFlankR  (pFlkRrel,    pValRrel);
    if (!mkFlankR.IsDone())
        throw Standard_Failure("HelicalSweep: ISO60 upper-flank line failed");

    BRepBuilderAPI_MakeEdge mkRootArc (rootCircle,  pValRrel, pValLrel);
    if (!mkRootArc.IsDone())
        throw Standard_Failure("HelicalSweep: ISO60 root arc failed");

    BRepBuilderAPI_MakeWire wireMk;
    wireMk.Add(mkFlankL  .Edge());
    wireMk.Add(mkCrestArc.Edge());
    wireMk.Add(mkFlankR  .Edge());
    wireMk.Add(mkRootArc .Edge());
    if (!wireMk.IsDone())
        throw Standard_Failure("HelicalSweep: ISO60 wire build failed");

    BRepBuilderAPI_MakeFace faceMk(wireMk.Wire(), /*OnlyPlane=*/true);
    if (!faceMk.IsDone())
        throw Standard_Failure("HelicalSweep: ISO60 face build failed");
    return faceMk.Face();
}

// Sweep helper — takes spine and profile, returns the swept solid or
// throws.  Validates the result via BRepCheck_Analyzer.
inline TopoDS_Shape sweepProfile(const TopoDS_Wire&  spine,
                                 const TopoDS_Shape& profile)
{
    BRepOffsetAPI_MakePipeShell pipe(spine);
    pipe.SetMode(true);                  // Frenet frame
    pipe.Add(profile, false, true);
    pipe.Build();
    if (!pipe.IsDone())
        throw Standard_Failure("helicalSweep: PipeShell.Build failed");
    if (!pipe.MakeSolid())
        throw Standard_Failure("helicalSweep: PipeShell.MakeSolid failed");
    const TopoDS_Shape result = pipe.Shape();
    if (result.IsNull())
        throw Standard_Failure("helicalSweep: PipeShell produced null shape");
    BRepCheck_Analyzer chk(result);
    if (!chk.IsValid())
        throw Standard_Failure("helicalSweep: PipeShell produced invalid BRep");
    return result;
}

}  // namespace detail

// Build a helical thread solid.  See file header docstring for the fallback
// cascade and the meaning of `outProfile`.
//
//   - axis: defines the helix axis (origin + Z direction)
//   - pitch_mm: vertical distance per full turn (must be > 0)
//   - length_mm: total axial length of the helix (must be > 0)
//   - major_radius_mm: outer thread radius (must be > 0)
//   - thread_depth_mm: depth of the V-profile valley (must be > 0,
//                      typically pitch_mm * 0.65)
//   - outProfile: if non-null, receives the profile flavour that succeeded
inline TopoDS_Shape helicalSweep(const gp_Ax1& axis,
                                 double pitch_mm,
                                 double length_mm,
                                 double major_radius_mm,
                                 double thread_depth_mm,
                                 HelicalSweepProfile* outProfile = nullptr)
{
    if (pitch_mm <= 0.0)
        throw Standard_Failure("helicalSweep: pitch_mm must be > 0");
    if (length_mm <= 0.0)
        throw Standard_Failure("helicalSweep: length_mm must be > 0");
    if (major_radius_mm <= 0.0)
        throw Standard_Failure("helicalSweep: major_radius_mm must be > 0");
    if (thread_depth_mm <= 0.0)
        throw Standard_Failure("helicalSweep: thread_depth_mm must be > 0");

    // Sample density: 32 per turn, minimum 24 overall.
    const int turns   = std::max(1, static_cast<int>(std::ceil(length_mm / pitch_mm)));
    const int samples = std::max(24, turns * 32);

    auto setProfile = [&](HelicalSweepProfile p) {
        if (outProfile) *outProfile = p;
    };

    TopoDS_Wire spine;
    try {
        spine = detail::buildHelicalSpine(
            axis, pitch_mm, length_mm, major_radius_mm, samples);
    }
    catch (const Standard_Failure& e) {
        spdlog::warn("helicalSweep: spine build failed ({}); using annular fallback",
                     e.what());
        setProfile(HelicalSweepProfile::AnnularFallback);
        const gp_Ax2 ax2(axis.Location(), axis.Direction());
        return annularRing(ax2,
                           major_radius_mm + thread_depth_mm,
                           major_radius_mm, length_mm);
    }

    // ── Attempt 1: rounded ISO 60° profile with crest + root fillets ─────
    try {
        const TopoDS_Shape profile = detail::buildIso60Profile(
            axis, pitch_mm, major_radius_mm, thread_depth_mm);
        const TopoDS_Shape result  = detail::sweepProfile(spine, profile);
        setProfile(HelicalSweepProfile::Iso60WithRadii);
        spdlog::debug("helicalSweep ok (iso60_with_radii) pitch={} length={} R={} depth={}",
                      pitch_mm, length_mm, major_radius_mm, thread_depth_mm);
        return result;
    }
    catch (const Standard_Failure& e) {
        spdlog::debug("helicalSweep: rounded ISO60 path failed ({}); "
                      "retrying with simplified closed-V", e.what());
    }
    catch (const std::exception& e) {
        spdlog::debug("helicalSweep: rounded ISO60 path threw ({}); "
                      "retrying with simplified closed-V", e.what());
    }

    // ── Attempt 2: simplified closed-V (no fillets) ──────────────────────
    try {
        const TopoDS_Shape profile = detail::buildVProfileSimplified(
            axis, pitch_mm, major_radius_mm, thread_depth_mm);
        const TopoDS_Shape result  = detail::sweepProfile(spine, profile);
        setProfile(HelicalSweepProfile::Iso60Simplified);
        spdlog::debug("helicalSweep ok (iso60_simplified) pitch={} length={} R={} depth={}",
                      pitch_mm, length_mm, major_radius_mm, thread_depth_mm);
        return result;
    }
    catch (const Standard_Failure& e) {
        spdlog::warn("helicalSweep: simplified V-profile path failed ({}); "
                     "falling back to annular ring", e.what());
    }
    catch (const std::exception& e) {
        spdlog::warn("helicalSweep: simplified V-profile path threw ({}); "
                     "falling back to annular ring", e.what());
    }

    // ── Fallback: annular ring (R, R + thread_depth) over `length_mm` ────
    setProfile(HelicalSweepProfile::AnnularFallback);
    const gp_Ax2 ax2(axis.Location(), axis.Direction());
    return annularRing(ax2,
                       major_radius_mm + thread_depth_mm,
                       major_radius_mm, length_mm);
}

}  // namespace prim

}  // namespace koocadcam::engine
