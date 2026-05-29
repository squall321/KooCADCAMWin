// @lat: [[process/test-strategy#skill round-trip]]
//
// cam_follower_roller_seat: real-geometry compound feature tests.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/cam_follower_roller_seat.hpp"

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

// Test 1: real multi-cut geometry — pocket + 2 slots + retention bore.
TEST(SkillCamFollowerRollerSeat, ApplyProducesRealCompoundGeometry)
{
    // Wide stock so all features fit safely.
    auto stock = skill::createCuboidStock(60.0, 60.0, 30.0);
    ASSERT_FALSE(stock->shape().IsNull());

    skill::cam_follower_roller_seat::Input in;
    in.roller_dia_mm           = 12.0;
    in.roller_length_mm        = 8.0;
    in.radial_clearance_mm     = 0.1;
    in.arm_clearance_width_mm  = 4.0;
    in.arm_clearance_depth_mm  = 6.0;
    in.retention_dia_mm        = 3.3;
    in.retention_depth_mm      = 20.0;
    in.center_x_mm             = 30.0;
    in.center_y_mm             = 30.0;
    in.center_z_mm             = 15.0;

    auto out = skill::cam_follower_roller_seat::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    const double rPocket = in.roller_dia_mm / 2.0 + in.radial_clearance_mm;
    const double lPocket = in.roller_length_mm + 2.0 * in.radial_clearance_mm;
    const double rRet    = in.retention_dia_mm / 2.0;

    const double vPocket = M_PI * rPocket * rPocket * lPocket;
    const double vSlot   = in.arm_clearance_depth_mm
                          * (lPocket) * in.arm_clearance_width_mm;
    const double vRet    = M_PI * rRet * rRet * in.retention_depth_mm;

    // The Boolean cut subtracts the FUSED tool — overlap of pocket and
    // retention bore is small (their intersection has volume ~
    // (πrRet² · 2·rPocket)) but we allow 25% tolerance for cross-feature
    // overlaps (slot ∩ pocket, retention ∩ pocket).
    const double expectedMax = vPocket + 2.0 * vSlot + vRet;
    const double actual = volumeOf(stock->shape()) - volumeOf(out.workpiece->shape());
    // Removed must be > the smallest single feature and < the unioned sum.
    EXPECT_GT(actual, vPocket * 0.8) << "should remove at least the pocket";
    EXPECT_LT(actual, expectedMax * 1.05);

    EXPECT_GT(out.workpiece->faceCount(), stock->faceCount());
}

// Test 2: validate rejects retention_dia too big.
TEST(SkillCamFollowerRollerSeat, ValidateRejectsAxleTooLarge)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 30.0);

    skill::cam_follower_roller_seat::Input in;
    in.roller_dia_mm           = 10.0;
    in.roller_length_mm        = 8.0;
    in.radial_clearance_mm     = 0.1;
    in.arm_clearance_width_mm  = 4.0;
    in.arm_clearance_depth_mm  = 6.0;
    in.retention_dia_mm        = 8.0;    // 0.8 × roller_dia — too big
    in.retention_depth_mm      = 10.0;

    auto r = skill::cam_follower_roller_seat::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-FOLLOWER-AXLE") { found = true; break; }
    EXPECT_TRUE(found);
    EXPECT_THROW(skill::cam_follower_roller_seat::apply(*stock, in),
                 skill::SkillError);
}

// Test 3: signature compound + round-trip.
TEST(SkillCamFollowerRollerSeat, SignatureCompound)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 30.0);

    skill::cam_follower_roller_seat::Input in;
    in.roller_dia_mm           = 12.0;
    in.roller_length_mm        = 8.0;
    in.radial_clearance_mm     = 0.1;
    in.arm_clearance_width_mm  = 4.0;
    in.arm_clearance_depth_mm  = 6.0;
    in.retention_dia_mm        = 3.3;
    in.retention_depth_mm      = 20.0;
    in.center_x_mm             = 30.0;
    in.center_y_mm             = 30.0;
    in.center_z_mm             = 15.0;

    auto out = skill::cam_follower_roller_seat::apply(*stock, in);
    EXPECT_EQ(out.signature.skill_id, std::string("cam_follower_roller_seat"));
    EXPECT_TRUE(out.signature.pattern.value("is_compound", false));
    EXPECT_GE(out.signature.pattern.value("subfeature_count", 0), 2);

    EXPECT_NEAR(out.signature.params["roller_dia_mm"].get<double>(),
                in.roller_dia_mm, 1e-9);
    EXPECT_NEAR(out.signature.params["retention_dia_mm"].get<double>(),
                in.retention_dia_mm, 1e-9);
}

// Test 4: recognize returns ≥ 1 candidate.
TEST(SkillCamFollowerRollerSeat, RecognizeFindsSeat)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 30.0);

    skill::cam_follower_roller_seat::Input in;
    in.roller_dia_mm           = 12.0;
    in.roller_length_mm        = 8.0;
    in.radial_clearance_mm     = 0.1;
    in.arm_clearance_width_mm  = 4.0;
    in.arm_clearance_depth_mm  = 6.0;
    in.retention_dia_mm        = 3.3;
    in.retention_depth_mm      = 20.0;
    in.center_x_mm             = 30.0;
    in.center_y_mm             = 30.0;
    in.center_z_mm             = 15.0;

    auto out = skill::cam_follower_roller_seat::apply(*stock, in);
    auto cands = skill::cam_follower_roller_seat::recognize(*out.workpiece);
    ASSERT_FALSE(cands.empty());
    EXPECT_GT(cands.front().confidence, 0.5);
    EXPECT_DOUBLE_EQ(cands.front().confidence, 1.0);
}

// Test 5: pocket fit clearance — the recovered roller_dia + 2·clearance
// should equal the actual pocket diameter.
TEST(SkillCamFollowerRollerSeat, ClearanceAccountedFor)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 30.0);

    skill::cam_follower_roller_seat::Input in;
    in.roller_dia_mm           = 12.0;
    in.roller_length_mm        = 8.0;
    in.radial_clearance_mm     = 0.25;
    in.arm_clearance_width_mm  = 4.0;
    in.arm_clearance_depth_mm  = 6.0;
    in.retention_dia_mm        = 3.3;
    in.retention_depth_mm      = 20.0;
    in.center_x_mm             = 30.0;
    in.center_y_mm             = 30.0;
    in.center_z_mm             = 15.0;

    auto out = skill::cam_follower_roller_seat::apply(*stock, in);

    // The recorded params encode roller_dia + clearance separately.
    EXPECT_NEAR(out.signature.params["roller_dia_mm"].get<double>(), 12.0, 1e-9);
    EXPECT_NEAR(out.signature.params["radial_clearance_mm"].get<double>(),
                0.25, 1e-9);
    // Larger clearance ⇒ larger pocket ⇒ more volume removed than with zero
    // clearance (sanity).
    auto stock2 = skill::createCuboidStock(60.0, 60.0, 30.0);
    in.radial_clearance_mm = 0.05;
    auto out2 = skill::cam_follower_roller_seat::apply(*stock2, in);

    const double vRemovedBig   = volumeOf(stock->shape())  - volumeOf(out.workpiece->shape());
    const double vRemovedSmall = volumeOf(stock2->shape()) - volumeOf(out2.workpiece->shape());
    EXPECT_GT(vRemovedBig, vRemovedSmall);
}
