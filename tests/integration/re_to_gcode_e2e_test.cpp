// @lat: [[engine/cam-3axis-verify]] [[process/test-strategy#re-roundtrip]]
//
// END-TO-END: a machined part travels the WHOLE chain in one test —
//   build -> recognize -> inferProcessPlan -> Executor::execute(fresh stock)
//        -> planAndVerify(re-synth) -> toGCodeProgram
// and we assert a RUNNABLE program comes out the far end.
//
// The integration_sketch_re_executor test stops at Executor::execute (geometry
// parity).  This one continues into CAM, so a break anywhere downstream — a
// generator that emits nothing for a recovered feature, a report that produces
// empty G-code — fails here instead of silently shipping a blank program.  It is
// the same class of gap an earlier discovery pass found between recognize and
// the Executor dispatch table: each newly-connected stage needs a test that
// crosses it, not just unit tests on either side.

#include <gtest/gtest.h>

#include "cam/ToolpathPlan.hpp"
#include "process/Executor.hpp"
#include "process/ProcessPlan.hpp"
#include "re/Recognizer.hpp"
#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/drill_hole.hpp"
#include "skills/mill_rect_pocket.hpp"
#include "skills/box_pocket.hpp"
#include "skills/bore_cylindrical.hpp"
#include "cam/Toolpath.hpp"

#include <string>

using namespace koocadcam;

namespace {
// Build a small machined part: a drilled hole + a milled rectangular pocket on
// the top face of a cuboid stock.  Both are recognised and both produce
// toolpaths, so the emitted program exercises more than one generator.
std::shared_ptr<skill::Workpiece> buildMachinedPart()
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 20.0);

    skill::drill_hole::Input hole;
    hole.position_x_mm = 18.0;
    hole.position_y_mm = 18.0;
    hole.diameter_mm   = 6.0;
    hole.depth_mm      = 10.0;
    auto w1 = skill::drill_hole::apply(*stock, hole);

    skill::mill_rect_pocket::Input pocket;
    pocket.entry_face  = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    pocket.center_x_mm = 40.0; pocket.center_y_mm = 40.0;
    pocket.length_mm   = 16.0; pocket.width_mm = 10.0; pocket.depth_mm = 4.0;
    auto w2 = skill::mill_rect_pocket::apply(*w1.workpiece, pocket);
    return w2.workpiece;
}
}  // namespace

// The full chain yields a runnable G-code program with a feed move and an end.
TEST(ReToGCodeE2E, MachinedPartProducesRunnableProgram)
{
    auto part = buildMachinedPart();
    ASSERT_NE(part, nullptr);

    // recognize -> plan
    const process::ProcessPlan plan = re::inferProcessPlan(*part, 0.7);
    ASSERT_FALSE(plan.empty()) << "the recovered part must yield a plan";

    // replay onto fresh stock
    auto fresh = skill::createCuboidStock(60.0, 60.0, 20.0);
    const auto result = process::Executor::execute(plan, fresh);
    ASSERT_TRUE(result.ok())
        << "the whole plan must replay (no unhandled feature): "
        << (result.errors.empty() ? "" : result.errors.front());
    ASSERT_NE(result.workpiece, nullptr);

    // CAM: generate + verify toolpaths, then emit one program.
    const cam::ToolpathReport report = cam::planAndVerify(*result.workpiece);
    EXPECT_GT(report.toolpaths.size(), 0u)
        << "the re-synth must carry machining features that generate toolpaths";

    const std::string gcode = cam::toGCodeProgram(report);
    EXPECT_NE(gcode.find("%"),   std::string::npos) << "program start/end marker";
    EXPECT_NE(gcode.find("G21"), std::string::npos) << "metric units";
    EXPECT_NE(gcode.find("G1"),  std::string::npos) << "at least one feed move";
    EXPECT_NE(gcode.find("M30"), std::string::npos) << "program end";
    // A real program has substantive body, not just headers.
    EXPECT_GT(gcode.size(), 200u) << "the program must contain real motion blocks";
}

// A TOP-FACE box_pocket (a phone camera island / watch top pocket) is machinable
// on the 3-axis-Z post: it must produce a real toolpath with cutting moves.
TEST(ReToGCodeE2E, TopFaceBoxPocketProducesToolpath)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 20.0);
    skill::box_pocket::Input in;
    in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1), 5.0, "largest" };  // +Z top
    in.length_mm = 14.0; in.width_mm = 10.0; in.depth_mm = 3.0;
    auto part = skill::box_pocket::apply(*stock, in).workpiece;

    // Generate toolpaths directly from the feature signature (a +Z box_pocket).
    const auto tps = cam::generateAllToolpaths(part->features());
    const cam::Toolpath* bp = nullptr;
    for (const auto& t : tps) if (t.feature_skill_id == "box_pocket") { bp = &t; break; }
    ASSERT_NE(bp, nullptr) << "a box_pocket feature must reach a toolpath generator";
    EXPECT_GT(bp->segments.size(), 2u)
        << "a +Z box_pocket must produce a real cutting path (not the empty fallback)";
}

// A RADIAL box_pocket (a side button, face_normal = +X) is NOT machinable on the
// 3-axis-Z post — it must yield an EMPTY toolpath (a loud deferral), NOT a
// plausible-but-wrong vertical plunge.
TEST(ReToGCodeE2E, RadialBoxPocketYieldsEmptyToolpath)
{
    auto stock = skill::createCuboidStock(40.0, 60.0, 30.0);
    skill::box_pocket::Input in;
    in.entry_face = skill::FaceByNormal{ gp_Dir(1, 0, 0), 5.0, "largest" };  // +X side
    in.length_mm = 10.0; in.width_mm = 5.0; in.depth_mm = 2.0;
    auto part = skill::box_pocket::apply(*stock, in).workpiece;

    const auto tps = cam::generateAllToolpaths(part->features());
    const cam::Toolpath* bp = nullptr;
    for (const auto& t : tps) if (t.feature_skill_id == "box_pocket") { bp = &t; break; }
    ASSERT_NE(bp, nullptr);
    EXPECT_EQ(bp->segments.size(), 0u)
        << "a radial (side-face) box_pocket must NOT be machined on the 3-axis-Z "
           "post — it must be an empty toolpath, not a wrong vertical plunge";
}

// A RADIAL bore_cylindrical (a crown side stem, axis -X) must likewise yield an
// empty toolpath — the axis guard stops the Z drill generator from emitting a
// wrong vertical plunge at the bore's XY.
TEST(ReToGCodeE2E, RadialBoreYieldsEmptyToolpath)
{
    auto stock = skill::createCuboidStock(40.0, 60.0, 30.0);
    skill::bore_cylindrical::Input in;
    in.entry_face    = skill::FaceByNormal{ gp_Dir(1, 0, 0), 5.0, "largest" };
    in.position_x_mm = 40.0; in.position_y_mm = 30.0; in.position_z_mm = 15.0;
    in.axis_dir      = gp_Dir(-1, 0, 0);
    in.diameter_mm   = 12.0; in.depth_mm = 10.0;
    auto part = skill::bore_cylindrical::apply(*stock, in).workpiece;

    const auto tps = cam::generateAllToolpaths(part->features());
    const cam::Toolpath* b = nullptr;
    for (const auto& t : tps) if (t.feature_skill_id == "bore_cylindrical") { b = &t; break; }
    ASSERT_NE(b, nullptr);
    EXPECT_EQ(b->segments.size(), 0u)
        << "a radial bore must be an empty toolpath (deferred), not a wrong plunge";
}

// A bare stock travels the same chain and yields an EMPTY-but-valid program
// (no machining features → no toolpaths → still a well-formed % ... M30 %).
TEST(ReToGCodeE2E, BareStockYieldsWellFormedEmptyProgram)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    const process::ProcessPlan plan = re::inferProcessPlan(*stock, 0.7);
    auto fresh = skill::createCuboidStock(40.0, 40.0, 10.0);
    const auto result = process::Executor::execute(plan, fresh);
    ASSERT_TRUE(result.ok());
    ASSERT_NE(result.workpiece, nullptr);

    const cam::ToolpathReport report = cam::planAndVerify(*result.workpiece);
    const std::string gcode = cam::toGCodeProgram(report);
    EXPECT_NE(gcode.find("%"),   std::string::npos);
    EXPECT_NE(gcode.find("M30"), std::string::npos)
        << "even with no toolpaths the program must be well-formed";
}
