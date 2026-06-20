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
