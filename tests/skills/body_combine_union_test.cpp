// @lat: [[process/test-strategy#skill round-trip]]
//
// body_combine_union — Boolean OR of workpiece + aux primitive.
//
// Cases (5):
//   1. fusing a non-overlapping box ADDS the full box volume.
//   2. fusing a cylinder also works.
//   3. DFM rejects bad aux_kind.
//   4. DFM rejects non-positive dims.
//   5. recognize replays history.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/body_combine_union.hpp"

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

// ─── 1. fusing a non-overlapping box adds the box volume ───────────────────
TEST(SkillBodyCombineUnion, FuseAddsBoxVolume)
{
    auto stock = skill::createCuboidStock(20.0, 20.0, 10.0);   // 4000 mm³
    const double volBefore = volumeOf(stock->shape());

    skill::body_combine_union::Input in;
    in.aux_kind   = "box";
    in.aux_origin = { 30.0, 0.0, 0.0 };       // detached to +X
    in.aux_dims   = { 10.0, 10.0, 10.0 };     // 1000 mm³

    auto out = skill::body_combine_union::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    const double volAfter = volumeOf(out.workpiece->shape());
    EXPECT_NEAR(volAfter - volBefore, 1000.0, 5.0);
    EXPECT_FALSE(out.signature.pattern["is_compound"].get<bool>());
    EXPECT_NEAR(out.signature.pattern["derived_added_volume_mm3"].get<double>(),
                1000.0, 5.0);
}

// ─── 2. cylinder aux works ─────────────────────────────────────────────────
TEST(SkillBodyCombineUnion, CylinderAuxWorks)
{
    auto stock = skill::createCuboidStock(20.0, 20.0, 10.0);
    const double volBefore = volumeOf(stock->shape());

    skill::body_combine_union::Input in;
    in.aux_kind   = "cylinder";
    in.aux_origin = { 40.0, 10.0, 0.0 };
    in.aux_dims   = { 5.0, 8.0, 0.0 };       // r=5, h=8 → π·25·8 ≈ 628 mm³

    auto out = skill::body_combine_union::apply(*stock, in);
    const double volAfter = volumeOf(out.workpiece->shape());
    const double expectedAdd = M_PI * 25.0 * 8.0;
    EXPECT_NEAR(volAfter - volBefore, expectedAdd, expectedAdd * 0.01);
}

// ─── 3. DFM rejects bad aux_kind ───────────────────────────────────────────
TEST(SkillBodyCombineUnion, ValidateRejectsBadKind)
{
    auto stock = skill::createCuboidStock(20.0, 20.0, 10.0);
    skill::body_combine_union::Input in;
    in.aux_kind = "sphere";       // unsupported
    in.aux_dims = { 5.0, 5.0, 5.0 };
    auto r = skill::body_combine_union::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_THROW(skill::body_combine_union::apply(*stock, in), skill::SkillError);
}

// ─── 4. DFM rejects non-positive dims ──────────────────────────────────────
TEST(SkillBodyCombineUnion, ValidateRejectsBadDims)
{
    auto stock = skill::createCuboidStock(20.0, 20.0, 10.0);
    skill::body_combine_union::Input in;
    in.aux_kind = "box";
    in.aux_dims = { 5.0, 0.0, 5.0 };
    auto r = skill::body_combine_union::validate(*stock, in);
    EXPECT_FALSE(r.passed);
}

// ─── 5. recognize replays history ──────────────────────────────────────────
TEST(SkillBodyCombineUnion, RecognizeReplaysHistory)
{
    auto stock = skill::createCuboidStock(20.0, 20.0, 10.0);
    skill::body_combine_union::Input in;
    in.aux_kind   = "box";
    in.aux_origin = { 30.0, 0.0, 0.0 };
    in.aux_dims   = { 6.0, 6.0, 6.0 };
    auto out   = skill::body_combine_union::apply(*stock, in);
    auto cands = skill::body_combine_union::recognize(*out.workpiece);
    ASSERT_GE(cands.size(), 1u);
    EXPECT_EQ(cands[0].skill_id, std::string("body_combine_union"));
    EXPECT_GT(cands[0].confidence, 0.9);
    EXPECT_EQ(cands[0].recovered_params["aux_kind"].get<std::string>(),
              std::string("box"));
}
