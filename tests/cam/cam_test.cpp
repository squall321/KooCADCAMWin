// @lat: [[engine/cam-3axis-verify]]
//
// CAM slice-1 tests: toolpath generation + cheap collision check.
//
// Cases (5):
//   1. DrillHoleProducesPlausiblePath        — 4+ segments, Rapid/Linear mix
//   2. MillCircularPocketHasManySegments     — ≥ 10 segments
//   3. GCodeOutput                           — G0 / G1 / X Y Z present
//   4. CollisionFreeForOpenAir               — path well above stock, 0 events
//   5. CollisionDetectedForRapidIntoStock    — diving Rapid → ≥ 1 event

#include <gtest/gtest.h>

#include "cam/CollisionCheck.hpp"
#include "cam/Toolpath.hpp"

#include "skills/Skill.hpp"
#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"

#include <BRepAlgoAPI_Cut.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <gp.hxx>
#include <gp_Ax2.hxx>
#include <gp_Pnt.hxx>

#include <memory>
#include <nlohmann/json.hpp>

#include <algorithm>
#include <string>

using namespace koocadcam;

namespace {

// Build a synthetic drill_hole FeatureSignature without going through
// skill::drill_hole::apply (which would also do a Boolean cut — overkill
// for testing the CAM-side dispatch).  Mirrors the field set populated
// by drill_hole.cpp:apply().
skill::FeatureSignature makeDrillHoleSig(
    double x_mm, double y_mm, double dia_mm, double depth_mm)
{
    skill::FeatureSignature sig;
    sig.skill_id = "drill_hole";
    sig.params = {
        {"position_x_mm", x_mm},
        {"position_y_mm", y_mm},
        {"axis_dir",      {0.0, 0.0, -1.0}},
        {"diameter_mm",   dia_mm},
        {"depth_mm",      depth_mm},
        {"through_hole",  false},
    };
    sig.pattern = { {"kind", "drill_hole"} };
    sig.tooling.tool_type        = "drill";
    sig.tooling.tool_dia_mm      = dia_mm;
    sig.tooling.tool_length_mm   = depth_mm * 1.5 + 5.0;
    sig.tooling.tool_material    = "carbide";
    sig.tooling.flute_count      = 2;
    sig.tooling.cutting_speed_sfm = 300.0;
    sig.tooling.feed_per_tooth_mm = 0.05;
    return sig;
}

skill::FeatureSignature makePocketSig(
    double x_mm, double y_mm, double dia_mm, double depth_mm)
{
    skill::FeatureSignature sig;
    sig.skill_id = "mill_circular_pocket";
    sig.params = {
        {"position_x_mm",      x_mm},
        {"position_y_mm",      y_mm},
        {"axis_dir",           {0.0, 0.0, -1.0}},
        {"diameter_mm",        dia_mm},
        {"depth_mm",           depth_mm},
        {"bottom_corner_r_mm", 0.0},
    };
    sig.pattern = { {"kind", "mill_circular_pocket"} };
    sig.tooling.tool_type        = "end_mill";
    sig.tooling.tool_dia_mm      = dia_mm * 0.5;   // half-pocket-dia is a sane endmill
    sig.tooling.tool_length_mm   = depth_mm * 1.5 + 5.0;
    sig.tooling.tool_material    = "carbide";
    sig.tooling.flute_count      = 4;
    sig.tooling.cutting_speed_sfm = 600.0;
    sig.tooling.feed_per_tooth_mm = 0.025;
    return sig;
}

}  // namespace

// ─── 1. drill_hole toolpath is plausible ──────────────────────────────────
TEST(CamToolpath, DrillHoleProducesPlausiblePath)
{
    auto sig = makeDrillHoleSig(/*x*/25.0, /*y*/15.0,
                                /*dia*/3.0, /*depth*/6.0);
    auto tp = cam::drillHoleToolpath(sig);

    EXPECT_EQ(tp.feature_skill_id, std::string("drill_hole"));
    EXPECT_GE(tp.segments.size(), 4u) << "drill cycle must have at least 4 segments";

    EXPECT_EQ(tp.segments.front().move, cam::PathSegment::Move::Rapid)
        << "drill path must start with a rapid";
    EXPECT_EQ(tp.segments.back().move,  cam::PathSegment::Move::Rapid)
        << "drill path must end with a rapid (retract)";

    bool hasLinear = false;
    for (const auto& s : tp.segments) {
        if (s.move == cam::PathSegment::Move::Linear) { hasLinear = true; break; }
    }
    EXPECT_TRUE(hasLinear) << "drill path must contain at least one linear (plunge)";

    EXPECT_GT(tp.spindle_rpm, 0.0);
    EXPECT_GT(tp.est_cycle_time_s, 0.0);
}

// ─── 2. mill_circular_pocket toolpath has many segments ───────────────────
TEST(CamToolpath, MillCircularPocketHasManySegments)
{
    auto sig = makePocketSig(/*x*/0.0, /*y*/0.0,
                             /*dia*/20.0, /*depth*/3.0);
    auto tp = cam::millCircularPocketToolpath(sig);

    EXPECT_EQ(tp.feature_skill_id, std::string("mill_circular_pocket"));
    EXPECT_GE(tp.segments.size(), 10u)
        << "pocket helix + spiral should produce many small segments";

    // First move must be a rapid up to safe Z (or some clearance) — sanity.
    EXPECT_EQ(tp.segments.front().move, cam::PathSegment::Move::Rapid);

    // Should contain at least one ArcCCW (helix or spiral).
    bool hasArc = false;
    for (const auto& s : tp.segments) {
        if (s.move == cam::PathSegment::Move::ArcCCW ||
            s.move == cam::PathSegment::Move::ArcCW) {
            hasArc = true;
            break;
        }
    }
    EXPECT_TRUE(hasArc) << "pocket toolpath should include arc moves";
}

// ─── 3. G-code output looks like G-code ───────────────────────────────────
TEST(CamToolpath, GCodeOutput)
{
    auto sig = makeDrillHoleSig(10.0, 20.0, 3.0, 5.0);
    auto tp  = cam::drillHoleToolpath(sig);
    auto gcode = cam::toGCode(tp);

    EXPECT_NE(gcode.find("G0"), std::string::npos) << "expected at least one G0 rapid";
    EXPECT_NE(gcode.find("G1"), std::string::npos) << "expected at least one G1 linear";
    EXPECT_NE(gcode.find("X10"), std::string::npos)
        << "expected X coordinate '10' in g-code output";
    EXPECT_NE(gcode.find("Y20"), std::string::npos)
        << "expected Y coordinate '20' in g-code output";
    EXPECT_NE(gcode.find("Z"), std::string::npos);

    // Header sanity.
    EXPECT_NE(gcode.find("G21"), std::string::npos) << "expected G21 (metric)";
    EXPECT_NE(gcode.find("G90"), std::string::npos) << "expected G90 (absolute)";
}

// ─── 4. Path entirely above stock → no collisions ─────────────────────────
TEST(CamCollision, CollisionFreeForOpenAir)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);
    ASSERT_FALSE(stock->shape().IsNull());

    // Construct a path that hovers far above the stock top (z = 10).
    cam::Toolpath tp;
    tp.feature_skill_id = "drill_hole";
    tp.tool_id          = "drill_3mm_carbide";
    tp.tool_dia_mm      = 3.0;
    tp.tool_length_mm   = 20.0;
    tp.spindle_rpm      = 12000.0;
    tp.segments.push_back({cam::PathSegment::Move::Rapid,
                           gp_Pnt(25.0, 25.0, 100.0), 0.0, 0, 0, 0});
    tp.segments.push_back({cam::PathSegment::Move::Rapid,
                           gp_Pnt(30.0, 30.0, 100.0), 0.0, 0, 0, 0});
    tp.segments.push_back({cam::PathSegment::Move::Linear,
                           gp_Pnt(30.0, 30.0,  80.0), 200.0, 0, 0, 0});

    auto events = cam::checkPath(tp, *stock, /*step*/0.5);
    EXPECT_TRUE(events.empty()) << "open-air toolpath should not collide";
}

// ─── 5. Rapid diving into stock → at least 1 collision event ──────────────
TEST(CamCollision, CollisionDetectedForRapidIntoStock)
{
    // Stock occupies (0,0,0)-(50,50,10).
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);
    ASSERT_FALSE(stock->shape().IsNull());

    cam::Toolpath tp;
    tp.feature_skill_id = "drill_hole";
    tp.tool_id          = "drill_3mm_carbide";
    tp.tool_dia_mm      = 3.0;
    tp.tool_length_mm   = 20.0;
    tp.spindle_rpm      = 12000.0;
    // Start safely above the block.
    tp.segments.push_back({cam::PathSegment::Move::Rapid,
                           gp_Pnt(25.0, 25.0, 20.0), 0.0, 0, 0, 0});
    // RAPID straight down to z = -1 (well below the stock top at z=10).
    // This is the canonical "rapid into stock" mistake we want to catch.
    tp.segments.push_back({cam::PathSegment::Move::Rapid,
                           gp_Pnt(25.0, 25.0, -1.0), 0.0, 0, 0, 0});

    auto events = cam::checkPath(tp, *stock, /*step*/0.5);
    EXPECT_GE(events.size(), 1u) << "rapid into stock must be flagged";

    // The collision must be on segment index 1 (the diving rapid).
    bool foundOnDive = false;
    for (const auto& e : events) {
        if (e.segment_index == 1) { foundOnDive = true; break; }
    }
    EXPECT_TRUE(foundOnDive);
}

// ─── 6/7. Exact narrow phase on a POCKETED workpiece ──────────────────────
//
// The single-AABB broad phase treats a pocket interior as solid.  These two
// cases — impossible to get right with the bbox alone — prove the narrow phase
// (DistShapeShape + SolidClassifier) distinguishes air from material.
namespace {
// Cuboid (0,0,0)-(50,50,10) with a Ø20 open pocket (z 4..10) at the centre.
std::shared_ptr<skill::Workpiece> makePocketedStock()
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);
    const TopoDS_Shape pocket = BRepPrimAPI_MakeCylinder(
        gp_Ax2(gp_Pnt(25.0, 25.0, 4.0), gp::DZ()), 10.0, 10.0).Shape();  // z 4..14, capped at top
    BRepAlgoAPI_Cut cut(stock->shape(), pocket);
    cut.Build();
    return std::make_shared<skill::Workpiece>(cut.Shape());
}
}  // namespace

// A clearance move whose tip sits in the OPEN pocket air (below the stock top,
// above the pocket floor, clear of the walls) must NOT be flagged — yet the
// legacy bbox-only path DOES flag it (false positive), so the same path proves
// both that the narrow phase clears it and that it was a real bbox defect.
TEST(CamCollision, ClearanceOverOpenPocketNotFlagged)
{
    auto stock = makePocketedStock();
    ASSERT_FALSE(stock->shape().IsNull());

    cam::Toolpath tp;
    tp.tool_dia_mm    = 3.0;     // R 1.5, pocket R 10 → 8.5 mm clearance
    tp.tool_length_mm = 20.0;
    tp.segments.push_back({cam::PathSegment::Move::Rapid,
                           gp_Pnt(25.0, 25.0, 30.0), 0.0, 0, 0, 0});
    tp.segments.push_back({cam::PathSegment::Move::Linear,
                           gp_Pnt(25.0, 25.0,  7.0), 200.0, 0, 0, 0});  // tip inside pocket air

    auto exact  = cam::checkPath(tp, *stock, 0.5, 0.1, /*exact*/true);
    auto legacy = cam::checkPath(tp, *stock, 0.5, 0.1, /*exact*/false);
    EXPECT_TRUE(exact.empty())
        << "tool clearing an OPEN pocket must not be flagged (narrow phase)";
    EXPECT_GE(legacy.size(), 1u)
        << "the legacy bbox-only path false-positives this — the defect being fixed";
}

// A plunge whose tip is embedded in solid material IS a real gouge and must be
// flagged by the narrow phase.
TEST(CamCollision, PlungeIntoSolidWallFlagged)
{
    auto stock = makePocketedStock();
    ASSERT_FALSE(stock->shape().IsNull());

    cam::Toolpath tp;
    tp.tool_dia_mm    = 3.0;
    tp.tool_length_mm = 20.0;
    tp.segments.push_back({cam::PathSegment::Move::Rapid,
                           gp_Pnt(5.0, 5.0, 30.0), 0.0, 0, 0, 0});
    tp.segments.push_back({cam::PathSegment::Move::Linear,
                           gp_Pnt(5.0, 5.0,  5.0), 200.0, 0, 0, 0});   // tip inside solid corner

    auto events = cam::checkPath(tp, *stock, 0.5, 0.1, /*exact*/true);
    EXPECT_GE(events.size(), 1u) << "plunge into solid material must be flagged";
}

// ─── Bonus: generateAllToolpaths dispatch sanity ──────────────────────────
TEST(CamToolpath, DispatcherHandlesMixedSignatures)
{
    std::vector<skill::FeatureSignature> sigs;
    sigs.push_back(makeDrillHoleSig(10, 10, 3.0, 4.0));
    sigs.push_back(makePocketSig(   30, 30, 12.0, 2.0));

    // Unknown skill — should emit an empty toolpath but stay aligned.
    skill::FeatureSignature unknown;
    unknown.skill_id = "fancy_unimplemented_skill";
    unknown.tooling.tool_type   = "end_mill";
    unknown.tooling.tool_dia_mm = 6.0;
    sigs.push_back(unknown);

    auto tps = cam::generateAllToolpaths(sigs);
    ASSERT_EQ(tps.size(), 3u);
    EXPECT_FALSE(tps[0].segments.empty());
    EXPECT_FALSE(tps[1].segments.empty());
    EXPECT_TRUE (tps[2].segments.empty());
    EXPECT_EQ(tps[2].feature_skill_id, std::string("fancy_unimplemented_skill"));
}
