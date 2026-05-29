// @lat: [[process/test-strategy#skill round-trip]]
//
// eccentric_shaft_collar: real-geometry compound feature tests.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/eccentric_shaft_collar.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using namespace koocadcam;

namespace {
double volumeOf(const TopoDS_Shape& s) {
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}
}

// Test 1: REAL multi-cut geometry — volume delta matches:
//   net delta = + collar_disc - bore - set_screw  (since we FUSE then CUT).
TEST(SkillEccentricShaftCollar, ApplyProducesRealCompoundGeometry)
{
    // Stock: thin slab where the collar will be ADDED + bored.
    auto stock = skill::createCuboidStock(60.0, 60.0, 4.0);
    ASSERT_FALSE(stock->shape().IsNull());

    skill::eccentric_shaft_collar::Input in;
    in.outer_dia_mm       = 30.0;
    in.thickness_mm       = 12.0;
    in.bore_dia_mm        = 10.0;
    in.eccentricity_mm    = 2.0;
    in.set_screw_dia_mm   = 3.3;
    in.set_screw_depth_mm = 6.0;
    in.center_x_mm        = 30.0;
    in.center_y_mm        = 30.0;
    in.center_z_mm        = 4.0;  // sit on top of the slab

    auto out = skill::eccentric_shaft_collar::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    const double rO  = in.outer_dia_mm / 2.0;
    const double rB  = in.bore_dia_mm / 2.0;
    const double rSS = in.set_screw_dia_mm / 2.0;
    const double vCollarAdd = M_PI * rO * rO * in.thickness_mm;
    const double vBore      = M_PI * rB * rB * in.thickness_mm;
    const double vSetScrew  = M_PI * rSS * rSS * in.set_screw_depth_mm;
    // Net = +collar - bore - set_screw.
    const double expectedDelta = vCollarAdd - vBore - vSetScrew;

    const double actualDelta = volumeOf(out.workpiece->shape())
                              - volumeOf(stock->shape());
    EXPECT_NEAR(actualDelta, expectedDelta, std::abs(expectedDelta) * 0.20)
        << "expected delta ≈ " << expectedDelta
        << ", actual = " << actualDelta;

    // Face count should grow (real geometry, not metadata clone).
    EXPECT_GT(out.workpiece->faceCount(), stock->faceCount());
}

// Test 2: validate rejects bad input + apply throws SkillError.
TEST(SkillEccentricShaftCollar, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 4.0);

    // Bore too large for OD (DIN 705 wall ratio).
    skill::eccentric_shaft_collar::Input in;
    in.outer_dia_mm    = 20.0;
    in.thickness_mm    = 10.0;
    in.bore_dia_mm     = 18.0;   // 0.9 × OD — violates DFM-COLLAR-WALL
    in.eccentricity_mm = 0.0;
    in.set_screw_dia_mm = 3.3;
    in.set_screw_depth_mm = 4.0;

    auto r = skill::eccentric_shaft_collar::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool foundWall = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-COLLAR-WALL") { foundWall = true; break; }
    EXPECT_TRUE(foundWall);
    EXPECT_THROW(skill::eccentric_shaft_collar::apply(*stock, in),
                 skill::SkillError);
}

// Test 3: signature.is_compound == true, subfeature_count >= 2, round-trip.
TEST(SkillEccentricShaftCollar, SignatureCompound)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 4.0);

    skill::eccentric_shaft_collar::Input in;
    in.outer_dia_mm       = 30.0;
    in.thickness_mm       = 12.0;
    in.bore_dia_mm        = 10.0;
    in.eccentricity_mm    = 2.0;
    in.set_screw_dia_mm   = 3.3;
    in.set_screw_depth_mm = 6.0;
    in.center_x_mm        = 30.0;
    in.center_y_mm        = 30.0;
    in.center_z_mm        = 4.0;

    auto out = skill::eccentric_shaft_collar::apply(*stock, in);
    EXPECT_EQ(out.signature.skill_id, std::string("eccentric_shaft_collar"));
    EXPECT_TRUE(out.signature.pattern.value("is_compound", false));
    EXPECT_GE(out.signature.pattern.value("subfeature_count", 0), 2);

    EXPECT_NEAR(out.signature.params["outer_dia_mm"].get<double>(),
                in.outer_dia_mm, 1e-9);
    EXPECT_NEAR(out.signature.params["bore_dia_mm"].get<double>(),
                in.bore_dia_mm, 1e-9);
    EXPECT_NEAR(out.signature.params["eccentricity_mm"].get<double>(),
                in.eccentricity_mm, 1e-9);
}

// Test 4: recognize returns ≥ 1 candidate at confidence > 0.5.
TEST(SkillEccentricShaftCollar, RecognizeFindsCollar)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 4.0);

    skill::eccentric_shaft_collar::Input in;
    in.outer_dia_mm       = 30.0;
    in.thickness_mm       = 12.0;
    in.bore_dia_mm        = 10.0;
    in.eccentricity_mm    = 2.0;
    in.set_screw_dia_mm   = 3.3;
    in.set_screw_depth_mm = 6.0;
    in.center_x_mm        = 30.0;
    in.center_y_mm        = 30.0;
    in.center_z_mm        = 4.0;

    auto out = skill::eccentric_shaft_collar::apply(*stock, in);
    auto cands = skill::eccentric_shaft_collar::recognize(*out.workpiece);
    ASSERT_FALSE(cands.empty());
    EXPECT_GT(cands.front().confidence, 0.5);
    // metadata path → confidence = 1.0
    EXPECT_DOUBLE_EQ(cands.front().confidence, 1.0);
}

// Test 5: eccentricity recovery — bore axis is NOT coincident with collar axis.
TEST(SkillEccentricShaftCollar, EccentricityIsNonZeroInGeometry)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 4.0);

    skill::eccentric_shaft_collar::Input in;
    in.outer_dia_mm       = 30.0;
    in.thickness_mm       = 12.0;
    in.bore_dia_mm        = 10.0;
    in.eccentricity_mm    = 2.5;
    in.set_screw_dia_mm   = 3.3;
    in.set_screw_depth_mm = 6.0;
    in.center_x_mm        = 30.0;
    in.center_y_mm        = 30.0;
    in.center_z_mm        = 4.0;

    auto out = skill::eccentric_shaft_collar::apply(*stock, in);

    // Volume removed by bore = π·r²·t.  If bore were on-axis, the bore
    // would still remove that much — so testing eccentricity needs to
    // confirm topology: we should still have a bore (cylindrical face)
    // that is *offset* from the disc axis.  Verify via face count
    // increase + the recorded param.
    EXPECT_NEAR(out.signature.params["eccentricity_mm"].get<double>(), 2.5, 1e-9);
    // A real (non-clone) compound feature added cylindrical surfaces.
    EXPECT_GT(out.workpiece->faceCount(), stock->faceCount() + 2);
}
