// @lat: [[process/test-strategy#skill round-trip]]
//
// laser_engrave_deep — Deep laser-engraving pocket round-trip tests.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/laser_engrave_deep.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

#include <cmath>

using namespace koocadcam;

namespace {
double volumeOf(const TopoDS_Shape& s) {
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}
}  // namespace

// ── 1. Apply removes a rectangular shallow pocket ────────────────────────
TEST(SkillLaserEngraveDeep, ApplyRemovesMaterialRectangular)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 10.0);

    skill::laser_engrave_deep::Input in;
    in.entry_face    = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 30.0;
    in.position_y_mm = 30.0;
    in.shape         = "rectangular";
    in.length_mm     = 20.0;
    in.width_mm      = 12.0;
    in.depth_mm      = 0.8;

    auto out = skill::laser_engrave_deep::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    const double expected = 20.0 * 12.0 * 0.8;
    const double removed = volumeOf(stock->shape()) - volumeOf(out.workpiece->shape());
    EXPECT_NEAR(removed, expected, expected * 0.05);
}

// ── 2. Validate rejects bad input (depth too shallow) ────────────────────
TEST(SkillLaserEngraveDeep, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 10.0);

    skill::laser_engrave_deep::Input in;
    in.entry_face    = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 30.0;
    in.position_y_mm = 30.0;
    in.shape         = "circular";
    in.diameter_mm   = 5.0;
    in.depth_mm      = 0.05;     // < 0.1 — use laser_mark

    auto r = skill::laser_engrave_deep::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-LE-DEPTH-LO" && f.severity == "error") {
            found = true; break;
        }
    EXPECT_TRUE(found);
    EXPECT_THROW(skill::laser_engrave_deep::apply(*stock, in), skill::SkillError);
}

// ── 3. Signature records kind + tool_type ────────────────────────────────
TEST(SkillLaserEngraveDeep, SignatureRecordsKindAndToolType)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 10.0);

    skill::laser_engrave_deep::Input in;
    in.entry_face    = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 30.0;
    in.position_y_mm = 30.0;
    in.shape         = "circular";
    in.diameter_mm   = 6.0;
    in.depth_mm      = 0.5;

    auto out = skill::laser_engrave_deep::apply(*stock, in);
    EXPECT_EQ(out.signature.skill_id, std::string("laser_engrave_deep"));
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("laser_engrave_deep"));
    EXPECT_EQ(out.signature.tooling.tool_type, std::string("laser_head"));
}

// ── 4. Recognize replays metadata ────────────────────────────────────────
TEST(SkillLaserEngraveDeep, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 10.0);

    skill::laser_engrave_deep::Input in;
    in.entry_face    = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 30.0;
    in.position_y_mm = 30.0;
    in.shape         = "rectangular";
    in.length_mm     = 18.0;
    in.width_mm      = 9.0;
    in.depth_mm      = 1.2;

    auto out = skill::laser_engrave_deep::apply(*stock, in);
    auto cands = skill::laser_engrave_deep::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_GT(cands[0].confidence, 0.9);
    EXPECT_EQ(cands[0].recovered_params["shape"].get<std::string>(),
              std::string("rectangular"));
    EXPECT_NEAR(cands[0].recovered_params["depth_mm"].get<double>(), 1.2, 1e-6);
}

// ── 5. Specific: depth > 2.0 mm rejected (out of envelope) ───────────────
TEST(SkillLaserEngraveDeep, DepthCeilingEnforced)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 20.0);

    skill::laser_engrave_deep::Input in;
    in.entry_face    = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 30.0;
    in.position_y_mm = 30.0;
    in.shape         = "circular";
    in.diameter_mm   = 5.0;
    in.depth_mm      = 3.0;       // > 2.0 — out of envelope

    auto r = skill::laser_engrave_deep::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-LE-DEPTH-HI" && f.severity == "error") {
            found = true; break;
        }
    EXPECT_TRUE(found);
    EXPECT_THROW(skill::laser_engrave_deep::apply(*stock, in), skill::SkillError);
}
