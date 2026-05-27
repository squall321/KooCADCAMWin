// @lat: [[process/test-strategy#skill round-trip]]
//
// engrave_path skill — spot-chain approximation tests.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/engrave_path.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

#include <cmath>

using namespace koocadcam;

namespace {
double volumeOf(const TopoDS_Shape& s)
{
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}
}  // namespace

// ── 1. Apply produces a valid solid with material removed ────────────────
TEST(SkillEngravePath, ApplyRemovesMaterial)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::engrave_path::Input in;
    in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.waypoints  = {
        { 10.0, 25.0 },
        { 20.0, 25.0 },
        { 30.0, 30.0 },
        { 40.0, 30.0 },
    };
    in.width_mm = 0.5;
    in.depth_mm = 0.2;

    auto out = skill::engrave_path::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    const double removed = volumeOf(stock->shape()) - volumeOf(out.workpiece->shape());
    EXPECT_GT(removed, 0.0);

    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_EQ(out.signature.skill_id, std::string("engrave_path"));
    EXPECT_EQ(out.signature.pattern["waypoint_count"].get<size_t>(), 4u);
}

// ── 2. DFM-019 rejects narrow channel ────────────────────────────────────
TEST(SkillEngravePath, ValidateRejectsThinChannel)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::engrave_path::Input in;
    in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.waypoints  = { { 10.0, 25.0 }, { 40.0, 25.0 } };
    in.width_mm = 0.10;   // < 0.15
    in.depth_mm = 0.2;

    auto r = skill::engrave_path::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool dfm019Found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-019") { dfm019Found = true; break; }
    EXPECT_TRUE(dfm019Found);
    EXPECT_THROW(skill::engrave_path::apply(*stock, in), skill::SkillError);
}

// ── 3. Single waypoint rejected ──────────────────────────────────────────
TEST(SkillEngravePath, ValidateRejectsSingleWaypoint)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::engrave_path::Input in;
    in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.waypoints  = { { 10.0, 25.0 } };
    in.width_mm = 0.3;
    in.depth_mm = 0.2;

    auto r = skill::engrave_path::validate(*stock, in);
    EXPECT_FALSE(r.passed);
}

// ── 4. Depth out of range rejected ───────────────────────────────────────
TEST(SkillEngravePath, ValidateRejectsOutOfRangeDepth)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::engrave_path::Input in;
    in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.waypoints  = { { 10.0, 25.0 }, { 40.0, 25.0 } };
    in.width_mm = 0.3;
    in.depth_mm = 0.02;  // < 0.05

    auto r = skill::engrave_path::validate(*stock, in);
    EXPECT_FALSE(r.passed);
}

// ── 5. Recognize replays metadata ────────────────────────────────────────
TEST(SkillEngravePath, RecognizeReplaysMetadata)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::engrave_path::Input in;
    in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.waypoints  = {
        { 15.0, 20.0 },
        { 35.0, 20.0 },
        { 35.0, 35.0 },
    };
    in.width_mm = 0.4;
    in.depth_mm = 0.18;

    auto out = skill::engrave_path::apply(*stock, in);
    auto cands = skill::engrave_path::recognize(*out.workpiece);

    ASSERT_EQ(cands.size(), 1u);
    EXPECT_EQ(cands[0].skill_id, std::string("engrave_path"));
    EXPECT_NEAR(cands[0].recovered_params["width_mm"].get<double>(), 0.4, 1e-6);
    EXPECT_NEAR(cands[0].recovered_params["depth_mm"].get<double>(), 0.18, 1e-6);
    ASSERT_TRUE(cands[0].recovered_params["waypoints"].is_array());
    EXPECT_EQ(cands[0].recovered_params["waypoints"].size(), 3u);
    EXPECT_GT(cands[0].confidence, 0.9);
}
