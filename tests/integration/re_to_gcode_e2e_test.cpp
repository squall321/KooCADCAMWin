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
#include "skills/micro_drill.hpp"
#include "skills/gun_drill.hpp"
#include "skills/box_boss.hpp"
#include "skills/dome_boss.hpp"
#include "cam/Toolpath.hpp"

#include <cmath>
#include <set>
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

// A DEEP box_pocket is roughed in MULTIPLE axial passes: the cutting contour
// repeats at several descending Z levels (each a fraction of the tool dia deep),
// ending at the exact floor.  A shallow pocket collapses to a single pass.
TEST(ReToGCodeE2E, DeepBoxPocketRoughsInMultiplePasses)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 30.0);   // top Z=30
    skill::box_pocket::Input in;
    in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1), 5.0, "largest" };
    in.length_mm = 20.0; in.width_mm = 16.0; in.depth_mm = 12.0;   // deep
    auto part = skill::box_pocket::apply(*stock, in).workpiece;

    const auto tps = cam::generateAllToolpaths(part->features());
    const cam::Toolpath* bp = nullptr;
    for (const auto& t : tps) if (t.feature_skill_id == "box_pocket") { bp = &t; break; }
    ASSERT_NE(bp, nullptr);
    // Collect the distinct cutting Z levels (Linear segments), rounded.
    std::set<long> levels;
    double deepest = 1e9;
    for (const auto& s : bp->segments)
        if (s.move == cam::PathSegment::Move::Linear) {
            levels.insert(std::lround(s.end_point.Z() * 100.0));
            deepest = std::min(deepest, s.end_point.Z());
        }
    EXPECT_GE(levels.size(), 2u) << "a 12mm-deep pocket must rough in >=2 axial passes";
    // The last pass lands on the exact floor: 30 - 12 = 18.
    EXPECT_NEAR(deepest, 18.0, 1e-6) << "the final pass reaches the true pocket floor";
}

// A RADIAL box_pocket (a side button, face_normal = +X) is now machined in its
// OWN local work-plane frame: a non-empty rectangular contour flagged radial,
// with the local depth axis = -face_normal, NOT the empty fallback.
TEST(ReToGCodeE2E, RadialBoxPocketMachinedInLocalFrame)
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
    EXPECT_GT(bp->segments.size(), 2u)
        << "a radial box_pocket is machined in its local frame (a real contour), not empty";
    EXPECT_TRUE(bp->is_radial()) << "the toolpath is flagged radial";
    // depth axis = -face_normal = -X (into the +X side face).
    EXPECT_NEAR(bp->work_depth_axis.X(), -1.0, 1e-6) << "local depth axis is -X (into the side)";
    // Segment coords are LOCAL: the contour lies in the (u,v) plane at local depth
    // (Z_local > 0 into the material); the rectangle straddles the origin in u,v.
    bool hasPosU = false, hasNegU = false;
    for (const auto& s : bp->segments)
        if (s.move == cam::PathSegment::Move::Linear) {
            if (s.end_point.X() > 0.1)  hasPosU = true;
            if (s.end_point.X() < -0.1) hasNegU = true;
        }
    EXPECT_TRUE(hasPosU && hasNegU) << "the local contour straddles the pocket centre in u";
    // The G-code flags the radial setup.
    EXPECT_NE(cam::toGCode(*bp).find("SETUP: RADIAL"), std::string::npos);
}

// A RADIAL bore_cylindrical (a crown side stem, axis -X) built through the real
// apply()/recognize chain is now machined in its OWN local work-plane frame — a
// non-empty plunge flagged radial, NOT the empty fallback and NOT a wrong +Z
// vertical plunge at the bore's XY.
TEST(ReToGCodeE2E, RadialBoreMachinedInLocalFrame)
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
    EXPECT_EQ(b->segments.size(), 4u) << "a radial bore is a local-frame plunge, not empty";
    EXPECT_TRUE(b->is_radial()) << "the toolpath is flagged radial";
    EXPECT_NEAR(std::abs(b->work_depth_axis.X()), 1.0, 1e-6) << "depth axis is ±X (radial)";
    // No wrong global +Z plunge: none of the local-frame moves are a vertical
    // descent in world Z (all segment coords are LOCAL, u=v=0).
    for (const auto& s : b->segments) {
        EXPECT_NEAR(s.end_point.X(), 0.0, 1e-9);
        EXPECT_NEAR(s.end_point.Y(), 0.0, 1e-9);
    }
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

// A RADIAL side grille (a linear_pattern about ±X) is now machined in its local
// work-plane frame: a per-hole plunge for every hole (NOT the empty fallback).
TEST(ReToGCodeE2E, RadialGrillePatternMachinedInLocalFrame)
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
    // 4 holes x 3 segments (rapid/plunge/retract) = 12, in the local frame.
    EXPECT_EQ(lp->segments.size(), 12u) << "one plunge per hole (4 holes), local frame";
    EXPECT_TRUE(lp->is_radial()) << "the toolpath is flagged radial";
    // The 4 holes are spread along the local u-axis (distinct u offsets).
    std::set<long> uOffsets;
    for (const auto& s : lp->segments)
        if (s.move == cam::PathSegment::Move::Linear)
            uOffsets.insert(std::lround(s.end_point.X() * 100.0));
    EXPECT_GE(uOffsets.size(), 4u) << "4 holes at 4 distinct local-u positions";
}

// A RECOVERED-style radial grille signature (world_n* + face_normal + 3-D
// hole_centers, NO axis_dir — the shape linear_pattern::recognize emits for a
// foreign side grille) must be DETECTED as radial by the CAM axis guard and
// machined in the local frame.  Without the face_normal key, cutAxisIsZ (which
// only inspects axis_dir/face_normal/axis_dir_xyz, not world_n*) would miss the
// radial axis and route it to the wrong +Z generator.
TEST(ReToGCodeE2E, RecoveredRadialGrilleDetectedAsRadial)
{
    skill::FeatureSignature sig;
    sig.skill_id = "linear_pattern";
    // Four holes along +Y on the +X side face at world X=40 (a recovered grille).
    sig.params = {
        { "use_world",   true },
        { "world_nx",    1.0 }, { "world_ny", 0.0 }, { "world_nz", 0.0 },
        { "face_normal", { 1.0, 0.0, 0.0 } },   // the fix: recognize now emits this
        { "hole_dia_mm", 3.0 }, { "hole_depth_mm", 6.0 },
        { "hole_centers", {
            { 40.0, 20.0, 15.0 },
            { 40.0, 28.0, 15.0 },
            { 40.0, 36.0, 15.0 },
            { 40.0, 44.0, 15.0 } } },
    };
    const auto tps = cam::generateAllToolpaths({ sig });
    ASSERT_EQ(tps.size(), 1u);
    // Detected radial → local-frame per-hole plunges (4 holes × 3 = 12), NOT empty
    // and NOT a +Z generator (which would ignore the side face).
    EXPECT_TRUE(tps[0].is_radial()) << "recovered radial grille must be flagged radial";
    EXPECT_EQ(tps[0].segments.size(), 12u) << "4 holes machined in the local frame";
    EXPECT_NEAR(std::abs(tps[0].work_depth_axis.X()), 1.0, 1e-6) << "depth axis is ±X (into the side)";
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

// A RADIAL counterbore (a side-face screw seat, axis along +X) is now machined in
// its own local work-plane frame: two coaxial plunges (pilot + seat) at local
// (0,0,depth), flagged radial — NOT the empty fallback.
TEST(ReToGCodeE2E, RadialCounterboreMachinedInLocalFrame)
{
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
    const cam::Toolpath& tp = tps[0];
    // Two coaxial plunges → 2 × (rapid/rapid/plunge/rapid) = 8 segments.
    EXPECT_EQ(tp.segments.size(), 8u) << "pilot + seat coaxial plunges in the local frame";
    EXPECT_TRUE(tp.is_radial()) << "flagged radial";
    EXPECT_NEAR(std::abs(tp.work_depth_axis.X()), 1.0, 1e-6) << "depth axis ±X (radial)";
    // Local depths: a plunge reaches +12 (pilot) and another +4 (seat).
    bool pilot = false, seat = false;
    for (const auto& s : tp.segments)
        if (s.move == cam::PathSegment::Move::Linear) {
            if (std::abs(s.end_point.Z() - 12.0) < 1e-6) pilot = true;
            if (std::abs(s.end_point.Z() -  4.0) < 1e-6) seat  = true;
        }
    EXPECT_TRUE(pilot && seat) << "local-frame plunges reach pilot(12) and seat(4) depths";
}

// A RADIAL mill_circular_pocket (a round side recess, axis +X) is machined as an
// inward circular contour in its local frame (ArcCCW rings), NOT empty.
TEST(ReToGCodeE2E, RadialCircularPocketMachinedInLocalFrame)
{
    skill::FeatureSignature sig;
    sig.skill_id = "mill_circular_pocket";
    sig.params = {
        { "position_x_mm", 40.0 }, { "position_y_mm", 30.0 }, { "position_z_mm", 15.0 },
        { "axis_dir", { -1.0, 0.0, 0.0 } },          // into the +X side face
        { "diameter_mm", 12.0 }, { "depth_mm", 4.0 },
    };
    const auto tps = cam::generateAllToolpaths({ sig });
    ASSERT_EQ(tps.size(), 1u);
    const cam::Toolpath& tp = tps[0];
    EXPECT_TRUE(tp.is_radial()) << "flagged radial";
    int arcs = 0;
    for (const auto& s : tp.segments)
        if (s.move == cam::PathSegment::Move::ArcCCW) ++arcs;
    EXPECT_GT(arcs, 0) << "an inward circular contour (ArcCCW rings) in the local frame";
    EXPECT_EQ(arcs % 4, 0) << "each ring is four ArcCCW quarter-turns";
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

// A micro_drill (sub-mm hole) is a plain plunge — it must reuse the drill_hole
// toolpath (4 segments), plunging from the real entry surface.
TEST(ReToGCodeE2E, MicroDrillProducesPlungeToolpath)
{
    auto stock = skill::createCuboidStock(20.0, 20.0, 10.0);   // top face Z=10
    skill::micro_drill::Input in;
    in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 10; in.position_y_mm = 10; in.axis_dir = gp_Dir(0, 0, -1);
    in.diameter_mm = 0.5; in.depth_mm = 2.0; in.through_hole = false;
    auto part = skill::micro_drill::apply(*stock, in).workpiece;

    const auto tps = cam::generateAllToolpaths(part->features());
    const cam::Toolpath* md = nullptr;
    for (const auto& t : tps) if (t.feature_skill_id == "micro_drill") { md = &t; break; }
    ASSERT_NE(md, nullptr) << "a micro_drill must reach the drill toolpath, not empty";
    EXPECT_EQ(md->segments.size(), 4u) << "drill plunge (rapid/rapid/plunge/retract)";
    // Plunge floor = entry(10) - depth(2) = 8.
    bool reaches8 = false;
    for (const auto& s : md->segments)
        if (s.move == cam::PathSegment::Move::Linear &&
            std::abs(s.end_point.Z() - 8.0) < 1e-6) reaches8 = true;
    EXPECT_TRUE(reaches8) << "micro_drill plunge must reach the real depth (Z=8), not from Z=0";
}

// A gun_drill (deep-hole) is likewise a plain plunge reusing the drill toolpath.
TEST(ReToGCodeE2E, GunDrillProducesPlungeToolpath)
{
    auto stock = skill::createCuboidStock(30.0, 30.0, 50.0);   // top face Z=50
    skill::gun_drill::Input in;
    in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 15; in.position_y_mm = 15; in.axis_dir = gp_Dir(0, 0, -1);
    in.diameter_mm = 3.0; in.depth_mm = 40.0; in.through_hole = false;   // ratio 13.3
    auto part = skill::gun_drill::apply(*stock, in).workpiece;

    const auto tps = cam::generateAllToolpaths(part->features());
    const cam::Toolpath* gd = nullptr;
    for (const auto& t : tps) if (t.feature_skill_id == "gun_drill") { gd = &t; break; }
    ASSERT_NE(gd, nullptr) << "a gun_drill must reach the drill toolpath, not empty";
    EXPECT_EQ(gd->segments.size(), 4u);
    // Plunge floor = entry(50) - depth(40) = 10.
    bool reaches10 = false;
    for (const auto& s : gd->segments)
        if (s.move == cam::PathSegment::Move::Linear &&
            std::abs(s.end_point.Z() - 10.0) < 1e-6) reaches10 = true;
    EXPECT_TRUE(reaches10) << "gun_drill plunge must reach the real depth (Z=10)";
}

// A box_boss (raised rectangular island) is machined by clearing the field
// around it down to its base plane — a rectangular profile contour offset OUTWARD
// by the tool radius, at the base Z.
TEST(ReToGCodeE2E, BoxBossProducesOutwardContourAtBasePlane)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 20.0);   // top face Z=20
    skill::box_boss::Input in;
    in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.center_x_mm = 0.0; in.center_y_mm = 0.0;   // face-local: at the face centre
    in.length_mm = 20.0; in.width_mm = 12.0; in.height_mm = 6.0;
    auto part = skill::box_boss::apply(*stock, in).workpiece;

    const auto tps = cam::generateAllToolpaths(part->features());
    const cam::Toolpath* bb = nullptr;
    for (const auto& t : tps) if (t.feature_skill_id == "box_boss") { bb = &t; break; }
    ASSERT_NE(bb, nullptr) << "a box_boss must reach boxBossToolpath, not empty";
    // rapid-approach + plunge + 4 contour edges + retract = 7 segments.
    EXPECT_EQ(bb->segments.size(), 7u) << "rectangular contour (approach/plunge/4 edges/retract)";
    // The contour cuts at the base plane Z=20 (the top face the boss stands on).
    bool atBaseZ = false;
    for (const auto& s : bb->segments)
        if (s.move == cam::PathSegment::Move::Linear &&
            std::abs(s.end_point.Z() - 20.0) < 1e-6) atBaseZ = true;
    EXPECT_TRUE(atBaseZ) << "the field-clearing contour runs at the base plane (Z=20)";
}

// A dome_boss (raised spherical cap) is finished with a ball-nose 3-D pass: a
// stack of constant-Z rings from the apex down to the base (each a full circle at
// the dome's surface radius for its height), plus a base skirt contour.
TEST(ReToGCodeE2E, DomeBossProducesZLevelRingFinishing)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 20.0);   // top face Z=20
    skill::dome_boss::Input in;
    in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.center_x_mm = 0.0; in.center_y_mm = 0.0;
    in.base_radius_mm = 8.0; in.height_mm = 5.0;
    auto part = skill::dome_boss::apply(*stock, in).workpiece;

    const auto tps = cam::generateAllToolpaths(part->features());
    const cam::Toolpath* db = nullptr;
    for (const auto& t : tps) if (t.feature_skill_id == "dome_boss") { db = &t; break; }
    ASSERT_NE(db, nullptr) << "a dome_boss must reach domeBossToolpath, not empty";

    // Multiple Z-level rings → arcs are a multiple of 4 and well above one ring.
    int arcs = 0;
    double minArcZ = 1e9, maxArcZ = -1e9;
    for (const auto& s : db->segments)
        if (s.move == cam::PathSegment::Move::ArcCCW) {
            ++arcs;
            minArcZ = std::min(minArcZ, s.end_point.Z());
            maxArcZ = std::max(maxArcZ, s.end_point.Z());
        }
    EXPECT_GT(arcs, 4) << "a 3-D finishing pass has more than one ring";
    EXPECT_EQ(arcs % 4, 0) << "each ring is four ArcCCW quarter-turns";
    // Rings span from near the apex (Z≈20+5) down to the base plane (Z=20).
    EXPECT_NEAR(minArcZ, 20.0, 1e-6) << "the lowest ring is at the base plane";
    EXPECT_GT(maxArcZ, 20.0 + 2.0)   << "the highest ring climbs well up the dome";
    EXPECT_LE(maxArcZ, 20.0 + 5.0 + 1e-6) << "no ring exceeds the apex";
}

// A RADIAL box_boss (a lug on a side face, normal ±X) is not machinable on the
// 3-axis-Z post → empty toolpath via the axis guard.
TEST(ReToGCodeE2E, RadialBoxBossYieldsEmptyToolpath)
{
    skill::FeatureSignature sig;
    sig.skill_id = "box_boss";
    sig.params = {
        { "center_x_world_mm", 30.0 }, { "center_y_world_mm", 0.0 },
        { "center_z_world_mm", 10.0 },
        { "length_mm", 10.0 }, { "width_mm", 8.0 }, { "height_mm", 4.0 },
        { "face_normal", { 1.0, 0.0, 0.0 } },   // +X → radial
    };
    const auto tps = cam::generateAllToolpaths({ sig });
    ASSERT_EQ(tps.size(), 1u);
    EXPECT_EQ(tps[0].segments.size(), 0u)
        << "a radial (side-face) boss is not machinable on the 3-axis-Z post";
}

// A RADIAL side bore (a crown-stem bore, axis +X) is a plain axial plunge — it is
// now machined in the feature's OWN local work-plane frame (not the empty
// fallback).  The toolpath is flagged radial, records the frame, and its G-code
// carries a SETUP comment so a re-setup / post can reach it.
TEST(ReToGCodeE2E, RadialSideBoreMachinedInLocalFrame)
{
    skill::FeatureSignature sig;
    sig.skill_id = "bore_cylindrical";
    sig.params = {
        { "position_x_mm", 40.0 }, { "position_y_mm", 20.0 }, { "position_z_mm", 15.0 },
        { "axis_dir", { -1.0, 0.0, 0.0 } },   // bores in -X, into the +X side face
        { "diameter_mm", 6.0 }, { "depth_mm", 8.0 },
    };
    const auto tps = cam::generateAllToolpaths({ sig });
    ASSERT_EQ(tps.size(), 1u);
    const cam::Toolpath& tp = tps[0];
    // NON-empty: a plunge in the local frame (rapid/rapid/plunge/retract).
    EXPECT_EQ(tp.segments.size(), 4u) << "a radial bore is a local-frame plunge, not empty";
    EXPECT_TRUE(tp.is_radial()) << "the toolpath is flagged radial";
    // The work-plane depth axis matches the cut axis (-X); the origin is the entry.
    EXPECT_NEAR(tp.work_depth_axis.X(), -1.0, 1e-9);
    EXPECT_NEAR(tp.work_origin.X(), 40.0, 1e-9);
    EXPECT_NEAR(tp.work_origin.Z(), 15.0, 1e-9);
    // The plunge reaches the depth in LOCAL coords (+depth along local Z).
    bool reachesDepth = false;
    for (const auto& s : tp.segments)
        if (s.move == cam::PathSegment::Move::Linear &&
            std::abs(s.end_point.Z() - 8.0) < 1e-6) reachesDepth = true;
    EXPECT_TRUE(reachesDepth) << "the local-frame plunge reaches depth 8";
    // The G-code flags the radial setup.
    const std::string g = cam::toGCode(tp);
    EXPECT_NE(g.find("SETUP: RADIAL"), std::string::npos)
        << "radial G-code must carry a setup comment, not pretend it is a +Z plunge";
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
