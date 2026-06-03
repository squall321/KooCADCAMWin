// @lat: [[process/test-strategy#skill round-trip]]
//
// scale_body_uniform — uniformly scale the workpiece about a centre.
//
// Cases (5):
//   1. ApplyScaleByFactorChangesVolumeByCube — vol_after ≈ vol_before × f^3.
//   2. ApplyScaleByHalfShrinks — factor = 0.5 → vol × 0.125.
//   3. ValidateRejectsTooSmallFactor — < 0.5 → DFM error.
//   4. ValidateRejectsTooBigFactor — > 3.0 → DFM error.
//   5. RecognizeReplaysHistory.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/scale_body_uniform.hpp"

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

// ─── 1. Scale by 2 → volume × 8 ───────────────────────────────────────────
TEST(SkillScaleBodyUniform, ApplyScaleByTwoOctuplesVolume)
{
    auto stock = skill::createCuboidStock(20.0, 15.0, 10.0);
    const double volBefore = volumeOf(stock->shape());

    skill::scale_body_uniform::Input in;
    in.center_xyz   = { 0.0, 0.0, 0.0 };
    in.scale_factor = 2.0;

    auto out = skill::scale_body_uniform::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    const double volAfter = volumeOf(out.workpiece->shape());
    const double expected = volBefore * 8.0;
    EXPECT_NEAR(volAfter, expected, 0.05 * expected);
    EXPECT_EQ(out.workpiece->features().size(), 1u);
}

// ─── 2. Scale by 0.5 → volume × 0.125 ─────────────────────────────────────
TEST(SkillScaleBodyUniform, ApplyScaleByHalfShrinks)
{
    auto stock = skill::createCuboidStock(40.0, 30.0, 20.0);
    const double volBefore = volumeOf(stock->shape());

    skill::scale_body_uniform::Input in;
    in.center_xyz   = { 0.0, 0.0, 0.0 };
    in.scale_factor = 0.5;

    auto out = skill::scale_body_uniform::apply(*stock, in);
    const double volAfter = volumeOf(out.workpiece->shape());
    const double expected = volBefore * 0.125;
    EXPECT_NEAR(volAfter, expected, 0.05 * expected);
}

// ─── 3. Validate rejects factor < 0.5 ─────────────────────────────────────
TEST(SkillScaleBodyUniform, ValidateRejectsTooSmallFactor)
{
    auto stock = skill::createCuboidStock(10.0, 10.0, 10.0);
    skill::scale_body_uniform::Input in;
    in.scale_factor = 0.1;
    auto r = skill::scale_body_uniform::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_THROW(skill::scale_body_uniform::apply(*stock, in),
                 skill::SkillError);
}

// ─── 4. Validate rejects factor > 3.0 ─────────────────────────────────────
TEST(SkillScaleBodyUniform, ValidateRejectsTooBigFactor)
{
    auto stock = skill::createCuboidStock(10.0, 10.0, 10.0);
    skill::scale_body_uniform::Input in;
    in.scale_factor = 5.0;
    auto r = skill::scale_body_uniform::validate(*stock, in);
    EXPECT_FALSE(r.passed);
}

// ─── 5. Recognize replays history ─────────────────────────────────────────
TEST(SkillScaleBodyUniform, RecognizeReplaysHistory)
{
    auto stock = skill::createCuboidStock(10.0, 10.0, 10.0);
    skill::scale_body_uniform::Input in;
    in.center_xyz   = { 0.0, 0.0, 0.0 };
    in.scale_factor = 1.5;
    auto out = skill::scale_body_uniform::apply(*stock, in);
    auto cands = skill::scale_body_uniform::recognize(*out.workpiece);
    ASSERT_GE(cands.size(), 1u);
    EXPECT_EQ(cands.front().skill_id, std::string("scale_body_uniform"));
    EXPECT_GE(cands.front().confidence, 0.9);
}
