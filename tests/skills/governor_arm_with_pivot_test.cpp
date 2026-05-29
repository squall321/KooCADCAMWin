// @lat: [[process/test-strategy#skill round-trip]]
//
// governor_arm_with_pivot: real-geometry compound feature tests.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/governor_arm_with_pivot.hpp"

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

// Test 1: real compound geometry — L-arm fused + 3 bores cut.
TEST(SkillGovernorArmWithPivot, ApplyProducesRealCompoundGeometry)
{
    // Use a thin slab as the workpiece "base" the arm sits on.
    auto stock = skill::createCuboidStock(80.0, 80.0, 4.0);
    ASSERT_FALSE(stock->shape().IsNull());

    skill::governor_arm_with_pivot::Input in;
    in.arm_length_mm      = 30.0;
    in.arm_width_mm       = 8.0;
    in.arm_thickness_mm   = 6.0;
    in.pivot_bore_dia_mm  = 4.0;
    in.ball_seat_dia_mm   = 5.0;
    in.ball_seat_depth_mm = 2.5;
    in.corner_x_mm        = 10.0;
    in.corner_y_mm        = 10.0;
    in.corner_z_mm        = 4.0;

    auto out = skill::governor_arm_with_pivot::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    const double w = in.arm_width_mm;
    const double t = in.arm_thickness_mm;
    // L-arm area = two perpendicular rectangles overlapping at corner w×w.
    // legA: (arm_length + w/2) × w; legB: w × (arm_length + w/2).
    const double areaL = (in.arm_length_mm + w / 2.0) * w
                       + w * (in.arm_length_mm + w / 2.0)
                       - w * w;     // subtract corner overlap
    const double vArmAdd = areaL * t;
    const double rP = in.pivot_bore_dia_mm / 2.0;
    const double rB = in.ball_seat_dia_mm / 2.0;
    const double vPivot = M_PI * rP * rP * t;
    const double vBall  = M_PI * rB * rB * in.ball_seat_depth_mm;
    const double expected = vArmAdd - vPivot - 2.0 * vBall;

    const double actual = volumeOf(out.workpiece->shape()) - volumeOf(stock->shape());
    EXPECT_NEAR(actual, expected, std::abs(expected) * 0.20)
        << "expected " << expected << ", actual " << actual;

    EXPECT_GT(out.workpiece->faceCount(), stock->faceCount());
}

// Test 2: DFM rejects pivot bore too large.
TEST(SkillGovernorArmWithPivot, ValidateRejectsPivotTooBig)
{
    auto stock = skill::createCuboidStock(80.0, 80.0, 4.0);

    skill::governor_arm_with_pivot::Input in;
    in.arm_length_mm      = 30.0;
    in.arm_width_mm       = 6.0;
    in.arm_thickness_mm   = 6.0;
    in.pivot_bore_dia_mm  = 5.5;   // > 0.7 × arm_width
    in.ball_seat_dia_mm   = 4.0;
    in.ball_seat_depth_mm = 2.5;
    in.corner_x_mm        = 10.0;
    in.corner_y_mm        = 10.0;

    auto r = skill::governor_arm_with_pivot::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-GOVARM-WALL") { found = true; break; }
    EXPECT_TRUE(found);
    EXPECT_THROW(skill::governor_arm_with_pivot::apply(*stock, in),
                 skill::SkillError);
}

// Test 3: signature compound + round-trip.
TEST(SkillGovernorArmWithPivot, SignatureCompound)
{
    auto stock = skill::createCuboidStock(80.0, 80.0, 4.0);

    skill::governor_arm_with_pivot::Input in;
    in.arm_length_mm      = 30.0;
    in.arm_width_mm       = 8.0;
    in.arm_thickness_mm   = 6.0;
    in.pivot_bore_dia_mm  = 4.0;
    in.ball_seat_dia_mm   = 5.0;
    in.ball_seat_depth_mm = 2.5;
    in.corner_x_mm        = 10.0;
    in.corner_y_mm        = 10.0;
    in.corner_z_mm        = 4.0;

    auto out = skill::governor_arm_with_pivot::apply(*stock, in);
    EXPECT_EQ(out.signature.skill_id, std::string("governor_arm_with_pivot"));
    EXPECT_TRUE(out.signature.pattern.value("is_compound", false));
    EXPECT_GE(out.signature.pattern.value("subfeature_count", 0), 2);

    EXPECT_NEAR(out.signature.params["arm_length_mm"].get<double>(),
                in.arm_length_mm, 1e-9);
    EXPECT_NEAR(out.signature.params["pivot_bore_dia_mm"].get<double>(),
                in.pivot_bore_dia_mm, 1e-9);
    EXPECT_NEAR(out.signature.params["ball_seat_dia_mm"].get<double>(),
                in.ball_seat_dia_mm, 1e-9);
}

// Test 4: recognize returns ≥ 1 candidate at confidence > 0.5.
TEST(SkillGovernorArmWithPivot, RecognizeFindsArm)
{
    auto stock = skill::createCuboidStock(80.0, 80.0, 4.0);

    skill::governor_arm_with_pivot::Input in;
    in.arm_length_mm      = 30.0;
    in.arm_width_mm       = 8.0;
    in.arm_thickness_mm   = 6.0;
    in.pivot_bore_dia_mm  = 4.0;
    in.ball_seat_dia_mm   = 5.0;
    in.ball_seat_depth_mm = 2.5;
    in.corner_x_mm        = 10.0;
    in.corner_y_mm        = 10.0;
    in.corner_z_mm        = 4.0;

    auto out = skill::governor_arm_with_pivot::apply(*stock, in);
    auto cands = skill::governor_arm_with_pivot::recognize(*out.workpiece);
    ASSERT_FALSE(cands.empty());
    EXPECT_GT(cands.front().confidence, 0.5);
    EXPECT_DOUBLE_EQ(cands.front().confidence, 1.0);
}

// Test 5: L-arm topology — pattern must record exactly 3 axial cylinders
// (pivot + 2 ball seats).
TEST(SkillGovernorArmWithPivot, PatternRecordsThreeAxialCyls)
{
    auto stock = skill::createCuboidStock(80.0, 80.0, 4.0);

    skill::governor_arm_with_pivot::Input in;
    in.arm_length_mm      = 30.0;
    in.arm_width_mm       = 8.0;
    in.arm_thickness_mm   = 6.0;
    in.pivot_bore_dia_mm  = 4.0;
    in.ball_seat_dia_mm   = 5.0;
    in.ball_seat_depth_mm = 2.5;
    in.corner_x_mm        = 10.0;
    in.corner_y_mm        = 10.0;
    in.corner_z_mm        = 4.0;

    auto out = skill::governor_arm_with_pivot::apply(*stock, in);
    EXPECT_EQ(out.signature.pattern.value("axial_cyl_face_count", 0), 3);
}
