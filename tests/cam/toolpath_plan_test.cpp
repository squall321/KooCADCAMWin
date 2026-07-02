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

#include <algorithm>

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

    // WORK OFFSET: the stock top is at Z=20, so the contour must be cut at
    // Z ~ 20-depth = 16 and the rapid must clear Z=20 — not the old Z=0 frame.
    // (mill_rect_pocket::apply must emit center_z_mm for the generator to see
    // the offset; before that it silently fell back to 0.)
    double zHi = -1e9, zLo = 1e9;
    for (const auto& s : report.toolpaths.front().segments) {
        zHi = std::max(zHi, s.end_point.Z());
        zLo = std::min(zLo, s.end_point.Z());
    }
    EXPECT_GT(zHi, 20.0) << "rapid/retract must clear the real top surface (Z~20)";
    EXPECT_GT(zLo, 10.0) << "cut depth must be near entryZ-depth (~16), not ~ -4";
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

// toGCodeProgram turns a verified report into one runnable G-code program.
TEST(ToolpathPlan, EmitsRunnableGCodeProgram)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 20.0);
    skill::mill_rect_pocket::Input in;
    in.entry_face  = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.center_x_mm = 25.0; in.center_y_mm = 25.0;
    in.length_mm   = 16.0; in.width_mm = 10.0; in.depth_mm = 4.0;
    const auto out = skill::mill_rect_pocket::apply(*stock, in);
    ASSERT_NE(out.workpiece, nullptr);

    const cam::ToolpathReport report = cam::planAndVerify(*out.workpiece);
    const std::string gcode = cam::toGCodeProgram(report);

    EXPECT_NE(gcode.find("%"), std::string::npos)    << "program start";
    EXPECT_NE(gcode.find("G21"), std::string::npos)  << "metric units";
    EXPECT_NE(gcode.find("G90"), std::string::npos)  << "absolute mode";
    EXPECT_NE(gcode.find("G17"), std::string::npos)  << "arc plane select";
    EXPECT_NE(gcode.find("G1"),  std::string::npos)  << "a feed move";
    EXPECT_NE(gcode.find("M30"), std::string::npos)  << "program end";
    EXPECT_NE(gcode.find("mill_rect_pocket"), std::string::npos) << "feature comment";
}

// A MULTI-feature program must be terminated EXACTLY ONCE (a single M30 at the
// very end) — not one M30 per feature, which would halt the machine at the first
// one.  A drill + a pocket (two different tools) also gets a (TOOLCHANGE ...)
// marker between them.
TEST(ToolpathPlan, MultiFeatureProgramEndsExactlyOnce)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 20.0);
    skill::drill_hole::Input dh;
    dh.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    dh.position_x_mm = 15.0; dh.position_y_mm = 15.0;
    dh.axis_dir = gp_Dir(0, 0, -1); dh.diameter_mm = 6.0; dh.depth_mm = 8.0;
    auto w1 = skill::drill_hole::apply(*stock, dh);
    skill::mill_rect_pocket::Input mp;
    mp.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    mp.center_x_mm = 40.0; mp.center_y_mm = 40.0;
    mp.length_mm = 14.0; mp.width_mm = 10.0; mp.depth_mm = 4.0;
    auto w2 = skill::mill_rect_pocket::apply(*w1.workpiece, mp);

    const cam::ToolpathReport report = cam::planAndVerify(*w2.workpiece);
    ASSERT_GE(report.toolpaths.size(), 2u) << "two machining features → two toolpaths";
    const std::string g = cam::toGCodeProgram(report);

    // Exactly ONE M30 (program end), at the end — not one per feature.
    size_t m30count = 0;
    for (size_t p = g.find("M30"); p != std::string::npos; p = g.find("M30", p + 1)) ++m30count;
    EXPECT_EQ(m30count, 1u) << "a multi-feature program must end exactly once";
    // A tool change is marked between the drill and the mill.
    EXPECT_NE(g.find("TOOLCHANGE"), std::string::npos)
        << "a change of tool between features must be marked";
}

// WORK OFFSET: a hole drilled into a stock whose top is NOT at Z=0 must get a
// toolpath that plunges from that real surface — not the old hard-coded Z=0.
// The drill skill now emits the resolved entry-plane Z (zMax for a top-down
// hole) and the generator honours it.
TEST(ToolpathPlan, ToolpathFollowsEntrySurfaceZ)
{
    // Stock spans Z in [0, 20]; the top face (drill entry) is at Z=20.
    auto stock = skill::createCuboidStock(50.0, 50.0, 20.0);
    skill::drill_hole::Input in;
    in.position_x_mm = 25.0;
    in.position_y_mm = 25.0;
    in.diameter_mm   = 6.0;
    in.depth_mm      = 8.0;
    const auto out = skill::drill_hole::apply(*stock, in);
    ASSERT_NE(out.workpiece, nullptr);

    const cam::ToolpathReport report = cam::planAndVerify(*out.workpiece);
    ASSERT_EQ(report.toolpaths.size(), 1u);
    const auto& segs = report.toolpaths.front().segments;
    ASSERT_GT(segs.size(), 1u);

    // The highest segment Z (the safe/approach height) must sit ABOVE the
    // entry surface (Z~20), and the lowest (the drilled depth) must be near
    // 20 - depth = 12 — both well above the old Z=0 frame.
    double zHi = -1e9, zLo = 1e9;
    for (const auto& s : segs) {
        zHi = std::max(zHi, s.end_point.Z());
        zLo = std::min(zLo, s.end_point.Z());
    }
    EXPECT_GT(zHi, 20.0) << "approach must clear the real top surface (Z~20)";
    EXPECT_GT(zLo, 5.0)
        << "deepest cut should be near entryZ - depth (~12), not the old ~ -8";
    EXPECT_LT(zLo, 15.0) << "and it must actually descend into the part";
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
