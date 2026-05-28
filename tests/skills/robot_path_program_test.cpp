// @lat: [[process/test-strategy#skill round-trip]]
//
// robot_path_program — 5-case sweep.  Verifies waypoint list is stored,
// path length is computed correctly, and DFM rejects degenerate inputs.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/robot_path_program.hpp"

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
TEST(SkillRobotPathProgram, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();

    skill::robot_path_program::Input in;
    in.waypoints = {
        {{ 0.0,   0.0, 0.0 }},
        {{ 100.0, 0.0, 0.0 }},
        {{ 100.0, 50.0, 0.0 }},
    };
    in.motion_type = "LIN";
    in.speed_mm_s  = 250.0;
    in.payload_kg  = 1.5;

    auto out = skill::robot_path_program::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("robot_path_program"));
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
    EXPECT_EQ(out.workpiece->features().size(), 1u);
}

// ── 2. DFM: < 2 waypoints / nonpositive speed/payload rejected ───────────
TEST(SkillRobotPathProgram, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    // Single waypoint = no path.
    skill::robot_path_program::Input lone;
    lone.waypoints = { {{ 0.0, 0.0, 0.0 }} };
    lone.speed_mm_s = 250.0;
    lone.payload_kg = 1.0;
    EXPECT_FALSE(skill::robot_path_program::validate(*stock, lone).passed);
    EXPECT_THROW(skill::robot_path_program::apply(*stock, lone), skill::SkillError);

    // Empty waypoints.
    skill::robot_path_program::Input empty;
    empty.speed_mm_s = 250.0;
    empty.payload_kg = 1.0;
    EXPECT_FALSE(skill::robot_path_program::validate(*stock, empty).passed);

    // Zero speed.
    skill::robot_path_program::Input slow;
    slow.waypoints = { {{ 0.0, 0.0, 0.0 }}, {{ 10.0, 0.0, 0.0 }} };
    slow.speed_mm_s = 0.0;
    slow.payload_kg = 1.0;
    EXPECT_FALSE(skill::robot_path_program::validate(*stock, slow).passed);

    // Zero payload.
    skill::robot_path_program::Input light;
    light.waypoints = { {{ 0.0, 0.0, 0.0 }}, {{ 10.0, 0.0, 0.0 }} };
    light.speed_mm_s = 250.0;
    light.payload_kg = 0.0;
    EXPECT_FALSE(skill::robot_path_program::validate(*stock, light).passed);
}

// ── 3. Signature records kind + waypoints + motion_type + speed ──────────
TEST(SkillRobotPathProgram, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::robot_path_program::Input in;
    in.waypoints = {
        {{ 0.0,   0.0,  0.0 }},
        {{ 100.0, 0.0,  0.0 }},
        {{ 100.0, 100.0, 0.0 }},
    };
    in.motion_type = "PTP";
    in.speed_mm_s  = 500.0;
    in.payload_kg  = 2.0;

    auto out = skill::robot_path_program::apply(*stock, in);
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("robot_path_program"));
    EXPECT_EQ(out.signature.pattern["motion_type"].get<std::string>(),
              std::string("PTP"));
    EXPECT_EQ(out.signature.pattern["waypoint_count"].get<int>(), 3);
    EXPECT_EQ(out.signature.pattern["waypoints"].size(), 3u);
    EXPECT_NEAR(out.signature.pattern["waypoints"][0][0].get<double>(),
                0.0, 1e-9);
    EXPECT_NEAR(out.signature.pattern["waypoints"][1][0].get<double>(),
                100.0, 1e-9);
    EXPECT_NEAR(out.signature.pattern["speed_mm_s"].get<double>(), 500.0, 1e-9);
    EXPECT_NEAR(out.signature.pattern["payload_kg"].get<double>(), 2.0, 1e-9);
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
}

// ── 4. Recognize via metadata replay ─────────────────────────────────────
TEST(SkillRobotPathProgram, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::robot_path_program::Input in;
    in.waypoints = {
        {{ 0.0, 0.0, 0.0 }},
        {{ 25.0, 0.0, 0.0 }},
    };
    in.motion_type = "LIN";
    in.speed_mm_s  = 200.0;
    in.payload_kg  = 1.0;

    auto out = skill::robot_path_program::apply(*stock, in);
    auto cands = skill::robot_path_program::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["motion_type"].get<std::string>(),
              std::string("LIN"));
    EXPECT_EQ(cands[0].recovered_params["waypoints"].size(), 2u);

    // Raw workpiece (no metadata) → empty.
    skill::Workpiece raw(out.workpiece->shape());
    EXPECT_TRUE(skill::robot_path_program::recognize(raw).empty());
}

// ── 5. total_length_mm is the sum of segment distances ───────────────────
TEST(SkillRobotPathProgram, TotalLengthSumsSegments)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    // Square-ish path 100 → 100 → 100 = 300 mm total.
    skill::robot_path_program::Input in;
    in.waypoints = {
        {{ 0.0,   0.0,   0.0 }},
        {{ 100.0, 0.0,   0.0 }},   // +100
        {{ 100.0, 100.0, 0.0 }},   // +100
        {{ 0.0,   100.0, 0.0 }},   // +100
    };
    in.motion_type = "LIN";
    in.speed_mm_s  = 250.0;
    in.payload_kg  = 1.0;

    auto out = skill::robot_path_program::apply(*stock, in);
    EXPECT_NEAR(out.signature.pattern["total_length_mm"].get<double>(),
                300.0, 1e-9);
    EXPECT_EQ(out.signature.pattern["waypoint_count"].get<int>(), 4);

    // Sanity: estimated cycle time = 300/250 + 0.5*4 = 1.2 + 2.0 = 3.2 s
    EXPECT_NEAR(out.signature.tooling.est_cycle_time_s, 3.2, 1e-6);
}
