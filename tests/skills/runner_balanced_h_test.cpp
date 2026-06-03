// @lat: [[process/test-strategy#skill round-trip]]
//
// runner_balanced_h — H-pattern balanced runner system (4 or 8 cavities).
//
// Cases (5):
//   1. ApplyRemovesVolume_4Cavity.
//   2. ApplyRemovesMoreVolume_8Cavity.
//   3. ValidateRejectsBadCavityCount.
//   4. ValidateRejectsBadDia.
//   5. RecognizeReplaysHistory.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/runner_balanced_h.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

using namespace koocadcam;

namespace {
double volumeOf(const TopoDS_Shape& s)
{
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}
}  // namespace

// ─── 1. 4-cavity runner removes volume ───────────────────────────────────
TEST(SkillRunnerBalancedH, ApplyRemovesVolume_4Cavity)
{
    auto stock = skill::createCuboidStock(120.0, 120.0, 20.0);
    const double volBefore = volumeOf(stock->shape());

    skill::runner_balanced_h::Input in;
    in.sprue_x_mm      = 60.0;
    in.sprue_y_mm      = 60.0;
    in.leg_length_mm   = 30.0;
    in.runner_dia_mm   = 4.0;
    in.runner_depth_mm = 8.0;
    in.cavity_count    = 4;

    auto out = skill::runner_balanced_h::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    const double volAfter = volumeOf(out.workpiece->shape());
    EXPECT_LT(volAfter, volBefore);
    EXPECT_GT(volBefore - volAfter, 10.0);
    EXPECT_EQ(out.signature.skill_id, std::string("runner_balanced_h"));
}

// ─── 2. 8-cavity runner removes more volume than 4-cavity ───────────────
TEST(SkillRunnerBalancedH, ApplyRemovesMoreVolume_8Cavity)
{
    auto stock4 = skill::createCuboidStock(120.0, 120.0, 20.0);
    auto stock8 = skill::createCuboidStock(120.0, 120.0, 20.0);

    skill::runner_balanced_h::Input in;
    in.sprue_x_mm      = 60.0;
    in.sprue_y_mm      = 60.0;
    in.leg_length_mm   = 25.0;
    in.runner_dia_mm   = 4.0;
    in.runner_depth_mm = 8.0;

    in.cavity_count = 4;
    auto out4 = skill::runner_balanced_h::apply(*stock4, in);
    const double removed4 =
        volumeOf(stock4->shape()) - volumeOf(out4.workpiece->shape());

    in.cavity_count = 8;
    auto out8 = skill::runner_balanced_h::apply(*stock8, in);
    const double removed8 =
        volumeOf(stock8->shape()) - volumeOf(out8.workpiece->shape());

    EXPECT_GT(removed8, removed4);
}

// ─── 3. DFM rejects bad cavity_count ─────────────────────────────────────
TEST(SkillRunnerBalancedH, ValidateRejectsBadCavityCount)
{
    auto stock = skill::createCuboidStock(120.0, 120.0, 20.0);

    skill::runner_balanced_h::Input in;
    in.sprue_x_mm      = 60.0;
    in.sprue_y_mm      = 60.0;
    in.leg_length_mm   = 30.0;
    in.runner_dia_mm   = 4.0;
    in.runner_depth_mm = 8.0;
    in.cavity_count    = 6;             // not 4 or 8

    auto r = skill::runner_balanced_h::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool sawCode = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-RUNNER-CAVITY") { sawCode = true; break; }
    EXPECT_TRUE(sawCode);
    EXPECT_THROW(skill::runner_balanced_h::apply(*stock, in),
                 skill::SkillError);
}

// ─── 4. DFM rejects bad diameter ─────────────────────────────────────────
TEST(SkillRunnerBalancedH, ValidateRejectsBadDia)
{
    auto stock = skill::createCuboidStock(120.0, 120.0, 20.0);

    skill::runner_balanced_h::Input in;
    in.sprue_x_mm      = 60.0;
    in.sprue_y_mm      = 60.0;
    in.leg_length_mm   = 30.0;
    in.runner_dia_mm   = 12.0;          // > 8 mm
    in.runner_depth_mm = 8.0;
    in.cavity_count    = 4;

    auto r = skill::runner_balanced_h::validate(*stock, in);
    EXPECT_FALSE(r.passed);
}

// ─── 5. Recognize replays history ────────────────────────────────────────
TEST(SkillRunnerBalancedH, RecognizeReplaysHistory)
{
    auto stock = skill::createCuboidStock(120.0, 120.0, 20.0);

    skill::runner_balanced_h::Input in;
    in.sprue_x_mm      = 60.0;
    in.sprue_y_mm      = 60.0;
    in.leg_length_mm   = 30.0;
    in.runner_dia_mm   = 4.0;
    in.runner_depth_mm = 8.0;
    in.cavity_count    = 4;
    auto out = skill::runner_balanced_h::apply(*stock, in);

    auto cands = skill::runner_balanced_h::recognize(*out.workpiece);
    ASSERT_GE(cands.size(), 1u);
    EXPECT_GE(cands.front().confidence, 0.9);
    EXPECT_EQ(cands.front().recovered_params["cavity_count"].get<int>(), 4);
    EXPECT_NEAR(cands.front().recovered_params["runner_dia_mm"].get<double>(),
                4.0, 0.01);
}
