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
#include "skills/linear_pattern.hpp"
#include "skills/circular_pattern.hpp"
#include "skills/counterbore.hpp"
#include "skills/countersink.hpp"
#include "skills/ream.hpp"
#include "skills/bore_with_shelf.hpp"
#include "skills/multi_step_bore.hpp"
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

// A +Z linear hole pattern (a top-face drilled grid / mounting holes) must
// produce a toolpath that plunges at EVERY hole — not the empty fallback.
TEST(ReToGCodeE2E, TopFaceLinearPatternDrillsEveryHole)
{
    auto stock = skill::createCuboidStock(80.0, 60.0, 10.0);
    skill::linear_pattern::Input in;
    in.face          = skill::FaceByNormal{ gp_Dir(0, 0, 1), 5.0, "largest" };
    in.hole_dia_mm   = 4.0; in.hole_depth_mm = 5.0;
    in.count_x = 4; in.pitch_x_mm = 12.0; in.count_y = 1;
    in.start_x_mm = -18.0;
    auto part = skill::linear_pattern::apply(*stock, in).workpiece;

    const auto tps = cam::generateAllToolpaths(part->features());
    const cam::Toolpath* lp = nullptr;
    for (const auto& t : tps) if (t.feature_skill_id == "linear_pattern") { lp = &t; break; }
    ASSERT_NE(lp, nullptr) << "a linear_pattern must reach the pattern toolpath generator";
    // 4 holes x 3 segments (rapid over / plunge / retract) = 12 segments.
    EXPECT_EQ(lp->segments.size(), 12u) << "one plunge per hole (4 holes)";
}

// A +Z circular pattern (a bolt circle) must plunge at every hole.
TEST(ReToGCodeE2E, BoltCirclePatternDrillsEveryHole)
{
    auto stock = skill::createCylindricalStock(80.0, 10.0);
    skill::circular_pattern::Input in;
    in.axis_origin_xyz = { 0.0, 0.0, 10.0 };
    in.axis_dir_xyz    = { 0.0, 0.0, 1.0 };
    in.hole_dia_mm = 5.0; in.hole_depth_mm = 6.0;
    in.radial_offset_mm = 30.0; in.count = 6; in.total_angle_deg = 360.0;
    auto part = skill::circular_pattern::apply(*stock, in).workpiece;

    const auto tps = cam::generateAllToolpaths(part->features());
    const cam::Toolpath* cp = nullptr;
    for (const auto& t : tps) if (t.feature_skill_id == "circular_pattern") { cp = &t; break; }
    ASSERT_NE(cp, nullptr);
    EXPECT_EQ(cp->segments.size(), 18u) << "one plunge per hole (6 holes x 3 segments)";
}

// A RADIAL side grille (a linear_pattern about ±X) must yield an empty toolpath —
// the same 3-axis-Z limitation as the radial pocket/bore.
TEST(ReToGCodeE2E, RadialGrillePatternYieldsEmptyToolpath)
{
    auto stock = skill::createCuboidStock(40.0, 60.0, 30.0);
    skill::linear_pattern::Input build;
    build.use_world   = true;
    build.world_ox_mm = 40.0; build.world_oy_mm = 42.0; build.world_oz_mm = 15.0;
    build.world_nx = 1.0; build.world_ny = 0.0; build.world_nz = 0.0;   // +X side
    build.hole_dia_mm = 2.0; build.hole_depth_mm = 6.0;
    build.count_x = 4; build.pitch_x_mm = 8.0; build.count_y = 1;
    auto part = skill::linear_pattern::apply(*stock, build).workpiece;

    const auto tps = cam::generateAllToolpaths(part->features());
    const cam::Toolpath* lp = nullptr;
    for (const auto& t : tps) if (t.feature_skill_id == "linear_pattern") { lp = &t; break; }
    ASSERT_NE(lp, nullptr);
    EXPECT_EQ(lp->segments.size(), 0u)
        << "a radial side grille is not machinable on the 3-axis-Z post";
}

// A counterbore (socket-head-cap-screw seat) must produce a TWO-plunge toolpath
// (pilot to pilot_depth + seat to seat_depth) — not the empty fallback and not a
// single drill plunge.
TEST(ReToGCodeE2E, CounterboreProducesTwoCoaxialPlunges)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 20.0);
    skill::counterbore::Input in;
    in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 30; in.position_y_mm = 30; in.axis_dir = gp_Dir(0, 0, -1);
    in.pilot_dia_mm = 6; in.pilot_depth_mm = 12; in.seat_dia_mm = 12; in.seat_depth_mm = 4;
    auto part = skill::counterbore::apply(*stock, in).workpiece;

    const auto tps = cam::generateAllToolpaths(part->features());
    const cam::Toolpath* cb = nullptr;
    for (const auto& t : tps) if (t.feature_skill_id == "counterbore") { cb = &t; break; }
    ASSERT_NE(cb, nullptr) << "a counterbore must reach counterboreToolpath";
    // pilot (3 segs) + seat (3 segs) = 6.
    EXPECT_EQ(cb->segments.size(), 6u) << "pilot plunge + seat plunge";
    // A Linear (cutting) segment must reach the deeper pilot Z (20 - 12 = 8).
    bool reachesPilotDepth = false;
    for (const auto& s : cb->segments)
        if (s.move == cam::PathSegment::Move::Linear &&
            std::abs(s.end_point.Z() - (20.0 - 12.0)) < 1e-6) reachesPilotDepth = true;
    EXPECT_TRUE(reachesPilotDepth) << "one plunge must reach the full pilot depth";
}

// A countersink (flat-head screw seat) must produce a pilot plunge + a chamfer
// plunge to the cone depth.
TEST(ReToGCodeE2E, CountersinkProducesPilotPlusChamferPlunge)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 20.0);
    skill::countersink::Input in;
    in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 30; in.position_y_mm = 30; in.axis_dir = gp_Dir(0, 0, -1);
    in.pilot_dia_mm = 5; in.pilot_depth_mm = 10; in.cone_top_dia_mm = 10; in.cone_angle_deg = 90;
    auto part = skill::countersink::apply(*stock, in).workpiece;

    const auto tps = cam::generateAllToolpaths(part->features());
    const cam::Toolpath* cs = nullptr;
    for (const auto& t : tps) if (t.feature_skill_id == "countersink") { cs = &t; break; }
    ASSERT_NE(cs, nullptr) << "a countersink must reach countersinkToolpath";
    EXPECT_EQ(cs->segments.size(), 6u) << "pilot plunge + chamfer plunge";
}

// A ream (finishing an existing bore to a precise dia) must produce a single
// finishing plunge along the bore axis.
TEST(ReToGCodeE2E, ReamProducesSingleFinishingPlunge)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 20.0);
    // A BLIND bore (not through) so the reamed cylinder's span is exact and the
    // entry-Z assertion is precise: top at Z=20, floor at Z=20-12=8.
    skill::drill_hole::Input dh;
    dh.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    dh.position_x_mm = 30; dh.position_y_mm = 30;
    dh.axis_dir = gp_Dir(0, 0, -1); dh.diameter_mm = 10; dh.depth_mm = 12; dh.through_hole = false;
    auto drilled = skill::drill_hole::apply(*stock, dh).workpiece;

    skill::ream::Input in;
    in.existing_hole_datum = skill::FaceCylinderByAxis{
        gp_Ax1(gp_Pnt(30, 30, 20), gp_Dir(0, 0, -1)), 5.0 };
    in.enlarge_by_mm = 0.10;
    auto part = skill::ream::apply(*drilled, in).workpiece;

    const auto tps = cam::generateAllToolpaths(part->features());
    const cam::Toolpath* rm = nullptr;
    for (const auto& t : tps) if (t.feature_skill_id == "ream") { rm = &t; break; }
    ASSERT_NE(rm, nullptr) << "a ream must reach reamToolpath";
    EXPECT_EQ(rm->segments.size(), 3u) << "one finishing plunge (rapid/plunge/retract)";
    // The entry reference must be the REAL bore top (Z=20), not the OCCT
    // cylinder-axis BASE (Z=20.05, the cutter-overhang artefact).  segments[0] is
    // the rapid to safe_z = entryZ + 5, so entryZ = segments[0].Z - 5.
    ASSERT_GE(rm->segments.size(), 1u);
    const double entryZ = rm->segments[0].end_point.Z() - 5.0;  // safe_z − clearance
    EXPECT_NEAR(entryZ, 20.0, 0.02)
        << "ream entry must be the true bore top (20), not the axis-base artefact (20.05)";
}

// A RADIAL counterbore (a side-face screw seat, axis along +X) must yield an
// empty toolpath — the same 3-axis-Z limitation as the radial pocket/bore.
TEST(ReToGCodeE2E, RadialCounterboreYieldsEmptyToolpath)
{
    // Hand-build a counterbore signature with a +X (radial) axis and route it
    // through the dispatcher directly — the axis guard must refuse it.
    skill::FeatureSignature sig;
    sig.skill_id = "counterbore";
    sig.params = {
        { "position_x_mm", 0.0 }, { "position_y_mm", 30.0 }, { "position_z_mm", 10.0 },
        { "axis_dir", { 1.0, 0.0, 0.0 } },           // +X → radial
        { "pilot_dia_mm", 6.0 }, { "pilot_depth_mm", 12.0 },
        { "seat_dia_mm", 12.0 }, { "seat_depth_mm", 4.0 },
    };
    const auto tps = cam::generateAllToolpaths({ sig });
    ASSERT_EQ(tps.size(), 1u);
    EXPECT_EQ(tps[0].segments.size(), 0u)
        << "a radial counterbore is not machinable on the 3-axis-Z post";
}

// A bore_with_shelf (a stepped counterbore-class bore) must plunge to the shelf
// (upper_depth) and then to the full depth (upper + lower, since lower_depth is
// measured from the shelf).
TEST(ReToGCodeE2E, BoreWithShelfPlungesShelfThenFullDepth)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 30.0);
    skill::bore_with_shelf::Input in;
    in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 30; in.position_y_mm = 30; in.axis_dir = gp_Dir(0, 0, -1);
    in.upper_dia_mm = 8; in.upper_depth_mm = 10; in.lower_dia_mm = 16; in.lower_depth_mm = 8;
    auto part = skill::bore_with_shelf::apply(*stock, in).workpiece;

    const auto tps = cam::generateAllToolpaths(part->features());
    const cam::Toolpath* bs = nullptr;
    for (const auto& t : tps) if (t.feature_skill_id == "bore_with_shelf") { bs = &t; break; }
    ASSERT_NE(bs, nullptr) << "a bore_with_shelf must reach boreWithShelfToolpath";
    EXPECT_EQ(bs->segments.size(), 6u) << "shelf plunge + full-depth plunge";
    // Entry top = Z=30.  Shelf at 30-10=20, full depth at 30-(10+8)=12.
    bool reachesShelf = false, reachesFull = false;
    for (const auto& s : bs->segments)
        if (s.move == cam::PathSegment::Move::Linear) {
            if (std::abs(s.end_point.Z() - 20.0) < 1e-6) reachesShelf = true;
            if (std::abs(s.end_point.Z() - 12.0) < 1e-6) reachesFull  = true;
        }
    EXPECT_TRUE(reachesShelf) << "one plunge reaches the shelf (Z=20)";
    EXPECT_TRUE(reachesFull)  << "one plunge reaches the full depth (Z=12)";
}

// A multi_step_bore (N≥3 coaxial steps, cumulative depths) must plunge once per
// step at the CUMULATIVE depth.
TEST(ReToGCodeE2E, MultiStepBorePlungesEveryStepCumulatively)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 40.0);
    skill::multi_step_bore::Input in;
    in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 30; in.position_y_mm = 30; in.axis_dir = gp_Dir(0, 0, -1);
    in.steps = { { 20.0, 6.0 }, { 14.0, 6.0 }, { 9.0, 6.0 } };   // widest → narrowest
    auto part = skill::multi_step_bore::apply(*stock, in).workpiece;

    const auto tps = cam::generateAllToolpaths(part->features());
    const cam::Toolpath* ms = nullptr;
    for (const auto& t : tps) if (t.feature_skill_id == "multi_step_bore") { ms = &t; break; }
    ASSERT_NE(ms, nullptr) << "a multi_step_bore must reach multiStepBoreToolpath";
    EXPECT_EQ(ms->segments.size(), 9u) << "3 steps x 3 segments";
    // Entry top = Z=40.  Cumulative plunge floors at 40-6=34, 40-12=28, 40-18=22.
    bool z34 = false, z28 = false, z22 = false;
    for (const auto& s : ms->segments)
        if (s.move == cam::PathSegment::Move::Linear) {
            if (std::abs(s.end_point.Z() - 34.0) < 1e-6) z34 = true;
            if (std::abs(s.end_point.Z() - 28.0) < 1e-6) z28 = true;
            if (std::abs(s.end_point.Z() - 22.0) < 1e-6) z22 = true;
        }
    EXPECT_TRUE(z34 && z28 && z22) << "plunges at cumulative depths 34/28/22";
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
