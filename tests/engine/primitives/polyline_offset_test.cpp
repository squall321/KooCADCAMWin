// @lat: [[process/test-strategy#primitive polyline offset]]
//
// Unit tests for prim::offsetPolylineCutter — verify it produces a
// reasonable stadium-chain solid for a simple polyline.

#include <gtest/gtest.h>

#include "engine/primitives/Bbox.hpp"
#include "engine/primitives/PolylineOffset.hpp"

#include <BRepCheck_Analyzer.hxx>
#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

#include <array>
#include <cmath>
#include <vector>

namespace pr = koocadcam::engine::prim;

namespace {

double volumeOf(const TopoDS_Shape& s)
{
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}

}  // namespace

// ── 1. Straight 2-point polyline produces volume ≈ stadium volume ────────
//   Stadium = (length × width × depth) + (π × r² × depth)  [two half-disks]
//   Use very loose tolerance: stadium overlaps half-cylinders with the box
//   ends, so the actual volume is slightly less than the naive sum.
TEST(PrimPolylineOffset, StraightSegmentVolume)
{
    std::vector<std::array<double, 2>> wps = {
        { 0.0, 0.0 },
        { 10.0, 0.0 },
    };
    const double width = 0.5;
    const double depth = 0.2;
    const double base  = 5.0;
    TopoDS_Shape s = pr::offsetPolylineCutter(wps, width, depth, base);
    ASSERT_FALSE(s.IsNull());

    const double r = width / 2.0;
    const double length = 10.0;
    // Stadium area ≈ length × width + π × r² (two half-circles at the ends).
    const double area = length * width + M_PI * r * r;
    const double approx = area * depth;
    EXPECT_NEAR(volumeOf(s), approx, approx * 0.05);
}

// ── 2. Bounding box: cutter spans z = base_z - depth_mm .. base_z ────────
TEST(PrimPolylineOffset, BoundingBoxZSpan)
{
    std::vector<std::array<double, 2>> wps = {
        { 0.0, 0.0 }, { 5.0, 5.0 },
    };
    const double base = 10.0, depth = 0.3, width = 0.4;
    TopoDS_Shape s = pr::offsetPolylineCutter(wps, width, depth, base);
    ASSERT_FALSE(s.IsNull());

    const auto bb = pr::optimalBbox(s);
    EXPECT_NEAR(bb.zMax, base,         1e-3);
    EXPECT_NEAR(bb.zMin, base - depth, 1e-3);
}

// ── 3. Multi-segment polyline (L-shape) builds a valid solid ─────────────
TEST(PrimPolylineOffset, MultiSegmentLShape)
{
    std::vector<std::array<double, 2>> wps = {
        { 0.0, 0.0 },
        { 5.0, 0.0 },
        { 5.0, 5.0 },
    };
    TopoDS_Shape s = pr::offsetPolylineCutter(wps, 0.5, 0.2, 1.0);
    ASSERT_FALSE(s.IsNull());
    EXPECT_GT(volumeOf(s), 0.0);
    BRepCheck_Analyzer chk(s);
    EXPECT_TRUE(chk.IsValid());
}

// ── 4. Invalid inputs throw ──────────────────────────────────────────────
TEST(PrimPolylineOffset, RejectsBadInputs)
{
    std::vector<std::array<double, 2>> oneOnly = { { 0.0, 0.0 } };
    EXPECT_ANY_THROW(pr::offsetPolylineCutter(oneOnly, 0.5, 0.2, 1.0));

    std::vector<std::array<double, 2>> two = { { 0.0, 0.0 }, { 1.0, 0.0 } };
    EXPECT_ANY_THROW(pr::offsetPolylineCutter(two, 0.0, 0.2, 1.0));   // width <= 0
    EXPECT_ANY_THROW(pr::offsetPolylineCutter(two, 0.5, 0.0, 1.0));   // depth <= 0
}
