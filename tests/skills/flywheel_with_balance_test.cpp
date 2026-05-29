// @lat: [[process/test-strategy#skill round-trip]]
//
// flywheel_with_balance: real-geometry compound feature tests.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/flywheel_with_balance.hpp"

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

// Test 1: real multi-cut geometry — disc + bore + 6 drills volume delta.
TEST(SkillFlywheelWithBalance, ApplyProducesRealCompoundGeometry)
{
    auto stock = skill::createCuboidStock(120.0, 120.0, 4.0);
    ASSERT_FALSE(stock->shape().IsNull());

    skill::flywheel_with_balance::Input in;
    in.outer_dia_mm           = 80.0;
    in.thickness_mm           = 10.0;
    in.hub_bore_dia_mm        = 12.0;
    in.balance_pcd_mm         = 60.0;
    in.balance_drill_dia_mm   = 3.0;
    in.balance_drill_depth_mm = 5.0;
    in.balance_count          = 6;
    in.chamfer_mm             = 0.0;   // no chamfer for clean delta math
    in.center_x_mm            = 60.0;
    in.center_y_mm            = 60.0;
    in.center_z_mm            = 4.0;

    auto out = skill::flywheel_with_balance::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    const double rO = in.outer_dia_mm / 2.0;
    const double rH = in.hub_bore_dia_mm / 2.0;
    const double rB = in.balance_drill_dia_mm / 2.0;
    const double vDiscAdd  = M_PI * rO * rO * in.thickness_mm;
    const double vHub      = M_PI * rH * rH * in.thickness_mm;
    const double vDrills   = M_PI * rB * rB * in.balance_drill_depth_mm
                             * in.balance_count;
    const double expected  = vDiscAdd - vHub - vDrills;

    const double actual = volumeOf(out.workpiece->shape()) - volumeOf(stock->shape());
    EXPECT_NEAR(actual, expected, std::abs(expected) * 0.20)
        << "expected " << expected << ", got " << actual;

    EXPECT_GT(out.workpiece->faceCount(), stock->faceCount());
}

// Test 2: DFM rejects bad PCD (drills would touch hub).
TEST(SkillFlywheelWithBalance, ValidateRejectsBadPcd)
{
    auto stock = skill::createCuboidStock(120.0, 120.0, 4.0);

    skill::flywheel_with_balance::Input in;
    in.outer_dia_mm           = 80.0;
    in.thickness_mm           = 10.0;
    in.hub_bore_dia_mm        = 12.0;
    in.balance_pcd_mm         = 14.0;   // too close to hub_bore_dia
    in.balance_drill_dia_mm   = 3.0;
    in.balance_drill_depth_mm = 5.0;
    in.balance_count          = 6;
    in.chamfer_mm             = 0.0;

    auto r = skill::flywheel_with_balance::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-FLYWHEEL-PCD") { found = true; break; }
    EXPECT_TRUE(found);
    EXPECT_THROW(skill::flywheel_with_balance::apply(*stock, in),
                 skill::SkillError);
}

// Test 3: signature compound + params round-trip.
TEST(SkillFlywheelWithBalance, SignatureCompound)
{
    auto stock = skill::createCuboidStock(120.0, 120.0, 4.0);

    skill::flywheel_with_balance::Input in;
    in.outer_dia_mm           = 80.0;
    in.thickness_mm           = 10.0;
    in.hub_bore_dia_mm        = 12.0;
    in.balance_pcd_mm         = 60.0;
    in.balance_drill_dia_mm   = 3.0;
    in.balance_drill_depth_mm = 5.0;
    in.balance_count          = 6;
    in.chamfer_mm             = 0.5;
    in.center_x_mm            = 60.0;
    in.center_y_mm            = 60.0;
    in.center_z_mm            = 4.0;

    auto out = skill::flywheel_with_balance::apply(*stock, in);
    EXPECT_EQ(out.signature.skill_id, std::string("flywheel_with_balance"));
    EXPECT_TRUE(out.signature.pattern.value("is_compound", false));
    EXPECT_GE(out.signature.pattern.value("subfeature_count", 0), 2);

    EXPECT_EQ(out.signature.params["balance_count"].get<int>(), 6);
    EXPECT_NEAR(out.signature.params["balance_pcd_mm"].get<double>(),
                in.balance_pcd_mm, 1e-9);
}

// Test 4: recognize returns ≥ 1 candidate at confidence > 0.5.
TEST(SkillFlywheelWithBalance, RecognizeFindsFlywheel)
{
    auto stock = skill::createCuboidStock(120.0, 120.0, 4.0);

    skill::flywheel_with_balance::Input in;
    in.outer_dia_mm           = 80.0;
    in.thickness_mm           = 10.0;
    in.hub_bore_dia_mm        = 12.0;
    in.balance_pcd_mm         = 60.0;
    in.balance_drill_dia_mm   = 3.0;
    in.balance_drill_depth_mm = 5.0;
    in.balance_count          = 6;
    in.chamfer_mm             = 0.0;
    in.center_x_mm            = 60.0;
    in.center_y_mm            = 60.0;
    in.center_z_mm            = 4.0;

    auto out = skill::flywheel_with_balance::apply(*stock, in);
    auto cands = skill::flywheel_with_balance::recognize(*out.workpiece);
    ASSERT_FALSE(cands.empty());
    EXPECT_GT(cands.front().confidence, 0.5);
    // metadata path is exact
    EXPECT_DOUBLE_EQ(cands.front().confidence, 1.0);
}

// Test 5: drill count consistency — pattern reports the right N, and the
// removed-volume increases linearly with balance_count.
TEST(SkillFlywheelWithBalance, DrillCountConsistency)
{
    skill::flywheel_with_balance::Input base;
    base.outer_dia_mm           = 80.0;
    base.thickness_mm           = 10.0;
    base.hub_bore_dia_mm        = 12.0;
    base.balance_pcd_mm         = 60.0;
    base.balance_drill_dia_mm   = 3.0;
    base.balance_drill_depth_mm = 5.0;
    base.chamfer_mm             = 0.0;
    base.center_x_mm            = 60.0;
    base.center_y_mm            = 60.0;
    base.center_z_mm            = 4.0;

    auto stockA = skill::createCuboidStock(120.0, 120.0, 4.0);
    skill::flywheel_with_balance::Input inA = base; inA.balance_count = 4;
    auto outA = skill::flywheel_with_balance::apply(*stockA, inA);

    auto stockB = skill::createCuboidStock(120.0, 120.0, 4.0);
    skill::flywheel_with_balance::Input inB = base; inB.balance_count = 8;
    auto outB = skill::flywheel_with_balance::apply(*stockB, inB);

    const double rB = base.balance_drill_dia_mm / 2.0;
    const double vPerDrill = M_PI * rB * rB * base.balance_drill_depth_mm;

    const double deltaA = volumeOf(stockA->shape()) - volumeOf(outA.workpiece->shape());
    const double deltaB = volumeOf(stockB->shape()) - volumeOf(outB.workpiece->shape());

    // outB - outA in removed volume = 4 extra drills.
    const double diffVol = deltaB - deltaA;
    EXPECT_NEAR(diffVol, 4.0 * vPerDrill, vPerDrill * 0.5)
        << "4 extra drills should remove ≈ 4·v_per_drill more";

    // pattern records reflect the input count.
    EXPECT_EQ(outA.signature.pattern.value("balance_drill_count", 0), 4);
    EXPECT_EQ(outB.signature.pattern.value("balance_drill_count", 0), 8);
}
