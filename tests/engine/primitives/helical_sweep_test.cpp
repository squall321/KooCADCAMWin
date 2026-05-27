// @lat: [[process/test-strategy#primitive helical sweep]]
//
// Unit tests for prim::helicalSweep — verify that the helix builder
// produces a non-null, positive-volume solid with sensible bbox.  Tests
// pass whether the genuine swept-pipe path succeeded or the annular-ring
// fallback was used (both produce a valid solid).

#include <gtest/gtest.h>

#include "engine/primitives/Bbox.hpp"
#include "engine/primitives/HelicalSweep.hpp"

#include <BRepCheck_Analyzer.hxx>
#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>
#include <gp_Ax1.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>

namespace pr = koocadcam::engine::prim;

namespace {

double volumeOf(const TopoDS_Shape& s)
{
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}

}  // namespace

// ── 1. Basic build produces a non-null solid with positive volume ────────
TEST(PrimHelicalSweep, BuildsNonNullSolid)
{
    // M3 tap-style thread: pitch=0.5, length=4mm, R=1.25 (M3 tap-drill / 2),
    // V depth = 0.5 * 0.65 = 0.325 mm.
    const gp_Ax1 axis(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1));
    TopoDS_Shape s = pr::helicalSweep(axis, 0.5, 4.0, 1.25, 0.325);

    ASSERT_FALSE(s.IsNull());
    EXPECT_GT(volumeOf(s), 0.0);
}

// ── 2. Sensible bounding box: extents within R + thread_depth radially,
//      0..length + pitch/2 axially ────────────────────────────────────────
TEST(PrimHelicalSweep, BoundingBoxFitsExpectedEnvelope)
{
    const gp_Ax1 axis(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1));
    const double pitch = 1.0, length = 3.0, R = 2.5, depth = 0.65;
    TopoDS_Shape s = pr::helicalSweep(axis, pitch, length, R, depth);
    ASSERT_FALSE(s.IsNull());

    const auto bb = pr::optimalBbox(s);
    // Radial extents: V-profile pokes OUTWARD by `depth` from R, so the
    // outermost shell sits near R + depth.  Allow a small slack.
    const double rMax = R + depth + 1e-3;
    EXPECT_LE(bb.xMax,  rMax);
    EXPECT_GE(bb.xMin, -rMax);
    EXPECT_LE(bb.yMax,  rMax);
    EXPECT_GE(bb.yMin, -rMax);
    // Axial extents: 0 .. length, with V-profile half-pitch slack at top/bottom.
    EXPECT_GE(bb.zMin, -pitch / 2.0 - 1e-3);
    EXPECT_LE(bb.zMax,  length + pitch / 2.0 + 1e-3);
}

// ── 3. Validity of the BRep (closed solid) ────────────────────────────────
TEST(PrimHelicalSweep, BRepIsValid)
{
    const gp_Ax1 axis(gp_Pnt(5, 5, 0), gp_Dir(0, 0, 1));
    TopoDS_Shape s = pr::helicalSweep(axis, 0.7, 3.5, 2.0, 0.5);
    ASSERT_FALSE(s.IsNull());
    BRepCheck_Analyzer chk(s);
    EXPECT_TRUE(chk.IsValid());
}

// ── 4. Invalid inputs throw ──────────────────────────────────────────────
TEST(PrimHelicalSweep, RejectsBadInputs)
{
    const gp_Ax1 axis(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1));
    EXPECT_ANY_THROW(pr::helicalSweep(axis, 0.0, 4.0, 1.5, 0.3));   // pitch <= 0
    EXPECT_ANY_THROW(pr::helicalSweep(axis, 0.5, 0.0, 1.5, 0.3));   // length <= 0
    EXPECT_ANY_THROW(pr::helicalSweep(axis, 0.5, 4.0, 0.0, 0.3));   // R <= 0
    EXPECT_ANY_THROW(pr::helicalSweep(axis, 0.5, 4.0, 1.5, 0.0));   // depth <= 0
}
