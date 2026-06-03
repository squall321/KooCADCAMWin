// @lat: [[process/test-strategy#skill round-trip]]
//
// body_combine_subtract — Boolean MINUS (workpiece - aux body).
//
// Cases (5):
//   1. subtracting an overlapping box REMOVES the intersection volume.
//   2. cylinder subtract works (drills a hole).
//   3. DFM rejects bad aux_kind.
//   4. DFM rejects non-positive dims.
//   5. recognize replays history.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/body_combine_subtract.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using namespace koocadcam;

namespace {
double volumeOf(const TopoDS_Shape& s)
{
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}
}  // namespace

// ─── 1. subtract a fully-inside box removes its volume ─────────────────────
TEST(SkillBodyCombineSubtract, SubtractRemovesBoxVolume)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 20.0);    // 32000 mm³
    const double volBefore = volumeOf(stock->shape());

    skill::body_combine_subtract::Input in;
    in.aux_kind   = "box";
    in.aux_origin = { 10.0, 10.0, 5.0 };
    in.aux_dims   = { 20.0, 20.0, 10.0 };       // 4000 mm³ fully inside

    auto out = skill::body_combine_subtract::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    const double volAfter = volumeOf(out.workpiece->shape());
    EXPECT_NEAR(volBefore - volAfter, 4000.0, 10.0);
    EXPECT_FALSE(out.signature.pattern["is_compound"].get<bool>());
    EXPECT_NEAR(out.signature.pattern["derived_removed_volume_mm3"].get<double>(),
                4000.0, 10.0);
}

// ─── 2. cylinder subtract drills a through-hole ────────────────────────────
TEST(SkillBodyCombineSubtract, CylinderSubtractDrillsHole)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 20.0);
    const double volBefore = volumeOf(stock->shape());

    skill::body_combine_subtract::Input in;
    in.aux_kind   = "cylinder";
    in.aux_origin = { 20.0, 20.0, -1.0 };      // start below stock
    in.aux_dims   = { 5.0, 22.0, 0.0 };        // r=5, h=22 → punch through Z

    auto out = skill::body_combine_subtract::apply(*stock, in);
    const double volAfter = volumeOf(out.workpiece->shape());
    const double expectedRemoved = M_PI * 25.0 * 20.0;   // through 20 mm
    EXPECT_NEAR(volBefore - volAfter, expectedRemoved, expectedRemoved * 0.02);
}

// ─── 3. DFM rejects bad aux_kind ───────────────────────────────────────────
TEST(SkillBodyCombineSubtract, ValidateRejectsBadKind)
{
    auto stock = skill::createCuboidStock(20.0, 20.0, 10.0);
    skill::body_combine_subtract::Input in;
    in.aux_kind = "torus";
    in.aux_dims = { 5.0, 5.0, 5.0 };
    auto r = skill::body_combine_subtract::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_THROW(skill::body_combine_subtract::apply(*stock, in), skill::SkillError);
}

// ─── 4. DFM rejects non-positive dims ──────────────────────────────────────
TEST(SkillBodyCombineSubtract, ValidateRejectsBadDims)
{
    auto stock = skill::createCuboidStock(20.0, 20.0, 10.0);
    skill::body_combine_subtract::Input in;
    in.aux_kind = "cylinder";
    in.aux_dims = { -1.0, 5.0, 0.0 };
    auto r = skill::body_combine_subtract::validate(*stock, in);
    EXPECT_FALSE(r.passed);
}

// ─── 5. recognize replays history ──────────────────────────────────────────
TEST(SkillBodyCombineSubtract, RecognizeReplaysHistory)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 20.0);
    skill::body_combine_subtract::Input in;
    in.aux_kind   = "box";
    in.aux_origin = { 10.0, 10.0, 5.0 };
    in.aux_dims   = { 8.0, 8.0, 8.0 };
    auto out   = skill::body_combine_subtract::apply(*stock, in);
    auto cands = skill::body_combine_subtract::recognize(*out.workpiece);
    ASSERT_GE(cands.size(), 1u);
    EXPECT_EQ(cands[0].skill_id, std::string("body_combine_subtract"));
    EXPECT_GT(cands[0].confidence, 0.9);
    EXPECT_EQ(cands[0].recovered_params["aux_kind"].get<std::string>(),
              std::string("box"));
}
