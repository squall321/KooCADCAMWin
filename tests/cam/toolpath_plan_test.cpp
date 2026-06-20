// @lat: [[engine/cam-3axis-verify]]
//
// CAM integration glue: planAndVerify wires the toolpath generator + collision
// checker into one call.  A machining feature on a workpiece yields a toolpath;
// a workpiece with no machining features yields none.  This is the entry point
// that makes the (previously separately-callable) CAM pieces usable as a unit.

#include <gtest/gtest.h>

#include "cam/ToolpathPlan.hpp"

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/drill_hole.hpp"
#include "skills/mill_rect_pocket.hpp"
#include "skills/mill_slot.hpp"

using namespace koocadcam;

// A drilled workpiece carries a drill_hole feature → one drill toolpath, with
// real segments; planAndVerify runs the collision check and returns a
// consistent report.
TEST(ToolpathPlan, GeneratesToolpathForDrilledFeature)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 20.0);
    skill::drill_hole::Input in;
    in.position_x_mm = 25.0;
    in.position_y_mm = 25.0;
    in.diameter_mm   = 6.0;
    in.depth_mm      = 10.0;
    const auto out = skill::drill_hole::apply(*stock, in);
    ASSERT_NE(out.workpiece, nullptr);
    ASSERT_EQ(out.workpiece->features().size(), 1u);

    const cam::ToolpathReport report = cam::planAndVerify(*out.workpiece);
    EXPECT_EQ(report.toolpaths.size(), 1u) << "one drill feature → one toolpath";
    EXPECT_GT(report.toolpaths.front().segments.size(), 0u)
        << "the drill toolpath must have segments";
    // ok must be consistent with the collision list (deterministic report).
    EXPECT_EQ(report.ok, report.collisions.empty());
}

// A milled rectangular pocket now gets a real perimeter-contour toolpath (it
// was silently skipped before — the recognizer knows it but slice-1 CAM didn't).
TEST(ToolpathPlan, GeneratesToolpathForRectPocket)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 20.0);
    skill::mill_rect_pocket::Input in;
    in.entry_face  = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.center_x_mm = 25.0; in.center_y_mm = 25.0;
    in.length_mm   = 16.0; in.width_mm = 10.0; in.depth_mm = 4.0;
    const auto out = skill::mill_rect_pocket::apply(*stock, in);
    ASSERT_NE(out.workpiece, nullptr);

    const cam::ToolpathReport report = cam::planAndVerify(*out.workpiece);
    EXPECT_EQ(report.toolpaths.size(), 1u) << "rect pocket → one toolpath";
    EXPECT_GE(report.toolpaths.front().segments.size(), 5u)
        << "a perimeter contour has plunge + 4 sides + retract";
    EXPECT_EQ(report.ok, report.collisions.empty());
}

// A milled slot gets a centreline-traverse toolpath.
TEST(ToolpathPlan, GeneratesToolpathForSlot)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 20.0);
    skill::mill_slot::Input in;
    in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.start_x_mm = 15.0; in.start_y_mm = 25.0;
    in.end_x_mm   = 35.0; in.end_y_mm   = 25.0;
    in.width_mm   = 4.0;  in.depth_mm   = 3.0;
    const auto out = skill::mill_slot::apply(*stock, in);
    ASSERT_NE(out.workpiece, nullptr);

    const cam::ToolpathReport report = cam::planAndVerify(*out.workpiece);
    EXPECT_EQ(report.toolpaths.size(), 1u) << "slot → one toolpath";
    EXPECT_GE(report.toolpaths.front().segments.size(), 3u)
        << "a slot traverse has plunge + cut + retract";
}

// A bare stock has no machining features → no toolpaths, trivially clear.
TEST(ToolpathPlan, NoMachiningFeaturesYieldsNoToolpaths)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 20.0);
    const cam::ToolpathReport report = cam::planAndVerify(*stock);
    EXPECT_TRUE(report.toolpaths.empty()) << "no machining features → no toolpaths";
    EXPECT_TRUE(report.collisions.empty());
    EXPECT_TRUE(report.ok);
}
