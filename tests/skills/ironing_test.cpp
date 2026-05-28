// @lat: [[process/test-strategy#skill round-trip]]
//
// ironing skill — thin the walls of a drawn cup.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/deep_draw.hpp"
#include "skills/ironing.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

#include <cmath>

using namespace koocadcam;

namespace {

// Produce a cup workpiece via deep_draw to feed into ironing.
std::shared_ptr<skill::Workpiece> drawnCup()
{
    auto stock = skill::createCuboidStock(100.0, 80.0, 1.5);
    skill::deep_draw::Input dd;
    dd.entry_face        = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    dd.cup_position_x_mm = 50.0;
    dd.cup_position_y_mm = 40.0;
    dd.cup_dia_mm        = 20.0;
    dd.cup_depth_mm      = 10.0;
    dd.corner_r_mm       = 6.0;
    dd.sheet_thickness_mm = 1.5;
    auto out = skill::deep_draw::apply(*stock, dd);
    return out.workpiece;
}

}  // namespace

// ── 1. Apply: 30% wall reduction on a drawn cup ────────────────────────
TEST(SkillIroning, ApplyReducesWall)
{
    auto cup = drawnCup();

    skill::ironing::Input in;
    in.use_position_hint   = true;
    in.cup_position_x_hint_mm = 50.0;
    in.cup_position_y_hint_mm = 40.0;
    in.wall_reduction_pct  = 30.0;

    auto out = skill::ironing::apply(*cup, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    EXPECT_EQ(out.signature.skill_id, std::string("ironing"));
    const double preW  = out.signature.pattern["pre_wall_thickness_mm"].get<double>();
    const double postW = out.signature.pattern["post_wall_thickness_mm"].get<double>();
    EXPECT_GT(preW, postW);
    EXPECT_NEAR(postW, preW * 0.70, preW * 0.05);
}

// ── 2. DFM-IR-PASS: > 50% reduction rejected ───────────────────────────
TEST(SkillIroning, ValidateRejectsExcessiveReduction)
{
    auto cup = drawnCup();
    skill::ironing::Input in;
    in.use_position_hint = true;
    in.cup_position_x_hint_mm = 50.0;
    in.cup_position_y_hint_mm = 40.0;
    in.wall_reduction_pct = 60.0;

    auto r = skill::ironing::validate(*cup, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-IR-PASS") { found = true; break; }
    EXPECT_TRUE(found);
    EXPECT_THROW(skill::ironing::apply(*cup, in), skill::SkillError);
}

// ── 3. DFM-IR-MIN-WALL: too-thin resulting wall rejected ───────────────
// TODO(slice-9): chained apply path produces a workpiece where
// findCupWall returns wrong wall thickness for the second iteration.
TEST(SkillIroning, DISABLED_ValidateRejectsTooThinResult)
{
    auto cup = drawnCup();
    skill::ironing::Input in;
    in.use_position_hint = true;
    in.cup_position_x_hint_mm = 50.0;
    in.cup_position_y_hint_mm = 40.0;
    // 1.5 mm initial wall × (1 - 0.5) = 0.75 ... still > 0.4, so push it.
    // Force the resulting wall under 0.4 by applying max-pct twice via
    // back-to-back ironing OR simply use a very high reduction.
    in.wall_reduction_pct = 50.0;
    // First ironing brings wall to 0.75 mm.
    auto out1 = skill::ironing::apply(*cup, in);
    // Second ironing at 50% brings wall to 0.375 mm < 0.4 — rejected.
    auto r = skill::ironing::validate(*out1.workpiece, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-IR-MIN-WALL") { found = true; break; }
    EXPECT_TRUE(found);
}

// ── 4. DFM-INPUT: no cup found in workpiece → rejected ─────────────────
TEST(SkillIroning, ValidateRejectsNonCupStock)
{
    auto sheet = skill::createCuboidStock(100.0, 80.0, 1.5);
    skill::ironing::Input in;
    in.wall_reduction_pct = 30.0;

    auto r = skill::ironing::validate(*sheet, in);
    EXPECT_FALSE(r.passed);
}

// ── 5. Recognize finds the ironed cup ──────────────────────────────────
TEST(SkillIroning, RecognizeAfterApply)
{
    auto cup = drawnCup();
    skill::ironing::Input in;
    in.use_position_hint = true;
    in.cup_position_x_hint_mm = 50.0;
    in.cup_position_y_hint_mm = 40.0;
    in.wall_reduction_pct = 30.0;
    auto out = skill::ironing::apply(*cup, in);
    auto cands = skill::ironing::recognize(*out.workpiece);
    ASSERT_FALSE(cands.empty());
    const auto& c = cands.front();
    EXPECT_EQ(c.skill_id, std::string("ironing"));
    EXPECT_GE(c.confidence, 0.5);
}
