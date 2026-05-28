// @lat: [[process/test-strategy#skill round-trip]]
//
// robot_pick_place — 5-case sweep covering identity-geometry synthesis,
// payload + reach DFM gates, metadata signature replay.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/robot_pick_place.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

#include <cmath>

using namespace koocadcam;

namespace {

double volumeOf(const TopoDS_Shape& s)
{
    GProp_GProps p;
    BRepGProp::VolumeProperties(s, p);
    return p.Mass();
}

}  // namespace

// ── 1. Apply: geometry is COMPLETELY unchanged ───────────────────────────
TEST(SkillRobotPickPlace, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    skill::robot_pick_place::Input in;
    in.source_xyz   = { 0.0, 0.0, 0.0 };
    in.target_xyz   = { 600.0, 100.0, 50.0 };
    in.gripper_type = "parallel_jaw";
    in.payload_kg   = 2.5;

    auto out = skill::robot_pick_place::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("robot_pick_place"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ── 2. DFM: zero / negative payload rejected ─────────────────────────────
TEST(SkillRobotPickPlace, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::robot_pick_place::Input bad;
    bad.source_xyz = { 0.0, 0.0, 0.0 };
    bad.target_xyz = { 500.0, 0.0, 0.0 };
    bad.payload_kg = 0.0;             // invalid
    auto r = skill::robot_pick_place::validate(*stock, bad);
    EXPECT_FALSE(r.passed);
    EXPECT_THROW(skill::robot_pick_place::apply(*stock, bad), skill::SkillError);

    bad.payload_kg = -1.0;
    EXPECT_FALSE(skill::robot_pick_place::validate(*stock, bad).passed);

    bad.payload_kg  = 1.0;
    bad.approach_mm = -5.0;           // also invalid
    EXPECT_FALSE(skill::robot_pick_place::validate(*stock, bad).passed);
}

// ── 3. Signature records kind + source/target/gripper/payload ────────────
TEST(SkillRobotPickPlace, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::robot_pick_place::Input in;
    in.source_xyz   = { 10.0, 20.0, 30.0 };
    in.target_xyz   = { 510.0, 220.0, 230.0 };   // travel = 600 mm exact
    in.gripper_type = "vacuum";
    in.payload_kg   = 0.8;

    auto out = skill::robot_pick_place::apply(*stock, in);
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("robot_pick_place"));
    EXPECT_EQ(out.signature.pattern["gripper_type"].get<std::string>(),
              std::string("vacuum"));
    EXPECT_NEAR(out.signature.pattern["payload_kg"].get<double>(), 0.8, 1e-9);

    // source_xyz array preserved
    EXPECT_NEAR(out.signature.pattern["source_xyz"][0].get<double>(), 10.0, 1e-9);
    EXPECT_NEAR(out.signature.pattern["source_xyz"][1].get<double>(), 20.0, 1e-9);
    EXPECT_NEAR(out.signature.pattern["source_xyz"][2].get<double>(), 30.0, 1e-9);
    EXPECT_NEAR(out.signature.pattern["target_xyz"][0].get<double>(), 510.0, 1e-9);
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
    EXPECT_EQ(out.signature.tooling.tool_type, std::string("robot_arm"));
}

// ── 4. Recognize via metadata replay ─────────────────────────────────────
TEST(SkillRobotPickPlace, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::robot_pick_place::Input in;
    in.source_xyz   = { 0.0, 0.0, 0.0 };
    in.target_xyz   = { 250.0, 0.0, 0.0 };
    in.gripper_type = "magnetic";
    in.payload_kg   = 5.0;

    auto out = skill::robot_pick_place::apply(*stock, in);
    auto cands = skill::robot_pick_place::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["gripper_type"].get<std::string>(),
              std::string("magnetic"));
    EXPECT_NEAR(cands[0].recovered_params["payload_kg"].get<double>(), 5.0, 1e-9);

    // Same shape without metadata → empty.
    skill::Workpiece raw(out.workpiece->shape());
    EXPECT_TRUE(skill::robot_pick_place::recognize(raw).empty());
}

// ── 5. travel_mm computed correctly (specific assertion) ─────────────────
TEST(SkillRobotPickPlace, TravelDistanceComputedFromSourceTarget)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::robot_pick_place::Input in;
    in.source_xyz = { 0.0, 0.0, 0.0 };
    in.target_xyz = { 3.0, 4.0, 0.0 };       // 3-4-5 triangle → 5.0 mm
    in.payload_kg = 1.0;

    auto out = skill::robot_pick_place::apply(*stock, in);
    EXPECT_NEAR(out.signature.pattern["travel_mm"].get<double>(), 5.0, 1e-9);

    // Degenerate case: source == target → warning, still passes
    skill::robot_pick_place::Input deg;
    deg.source_xyz = { 100.0, 100.0, 100.0 };
    deg.target_xyz = { 100.0, 100.0, 100.0 };
    deg.payload_kg = 1.0;
    auto r = skill::robot_pick_place::validate(*stock, deg);
    EXPECT_TRUE(r.passed);
    bool warned = false;
    for (const auto& f : r.findings) {
        if (f.code == "DFM-ROBOT-DEGENERATE" && f.severity == "warning") {
            warned = true; break;
        }
    }
    EXPECT_TRUE(warned);
}
