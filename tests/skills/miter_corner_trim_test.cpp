// @lat: [[process/test-strategy#skill round-trip]]
//
// miter_corner_trim — trim a workpiece at the bisecting plane of two
// member axes (45° miter for orthogonal members).
//
// Tests (5):
//   1. Apply: trim removes some volume of an orthogonal-axis cuboid.
//   2. DFM rejects parallel axes.
//   3. DFM rejects null workpiece.
//   4. Signature pattern carries is_compound + profile_kind = "miter".
//   5. Recognize replays the feature from history.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/miter_corner_trim.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

#include <array>

using namespace koocadcam;

namespace {

double volumeOf(const TopoDS_Shape& s)
{
    if (s.IsNull()) return 0.0;
    GProp_GProps p;
    BRepGProp::VolumeProperties(s, p);
    return p.Mass();
}

}  // namespace

// ─── 1. Apply: trim removes some volume ──────────────────────────────────
TEST(SkillMiterCornerTrim, ApplyRemovesVolume)
{
    // Use a 100³ cuboid as proxy for two intersecting members joined into
    // one body.  Corner at one corner of the box, axes along +X and +Y.
    auto stock = skill::createCuboidStock(100.0, 100.0, 100.0);
    const double v0 = volumeOf(stock->shape());

    skill::miter_corner_trim::Input in;
    in.corner_xyz        = { 50.0, 50.0, 50.0 };
    in.member_a_axis_dir = { 1.0, 0.0, 0.0 };
    in.member_b_axis_dir = { 0.0, 1.0, 0.0 };

    auto out = skill::miter_corner_trim::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    const double v1 = volumeOf(out.workpiece->shape());

    EXPECT_LT(v1, v0);    // some material removed
    EXPECT_GT(v1, 0.0);   // but not everything
}

// ─── 2. DFM rejects parallel axes ────────────────────────────────────────
TEST(SkillMiterCornerTrim, ValidateRejectsParallelAxes)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 50.0);

    skill::miter_corner_trim::Input in;
    in.corner_xyz        = { 0.0, 0.0, 0.0 };
    in.member_a_axis_dir = { 1.0, 0.0, 0.0 };
    in.member_b_axis_dir = { 2.0, 0.0, 0.0 };

    auto r = skill::miter_corner_trim::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_THROW(skill::miter_corner_trim::apply(*stock, in),
                 skill::SkillError);
}

// ─── 3. DFM rejects zero direction ───────────────────────────────────────
TEST(SkillMiterCornerTrim, ValidateRejectsZeroAxis)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 50.0);

    skill::miter_corner_trim::Input in;
    in.corner_xyz        = { 0.0, 0.0, 0.0 };
    in.member_a_axis_dir = { 0.0, 0.0, 0.0 };
    in.member_b_axis_dir = { 0.0, 1.0, 0.0 };

    auto r = skill::miter_corner_trim::validate(*stock, in);
    EXPECT_FALSE(r.passed);
}

// ─── 4. Signature pattern ────────────────────────────────────────────────
TEST(SkillMiterCornerTrim, SignatureCompound)
{
    auto stock = skill::createCuboidStock(100.0, 100.0, 100.0);

    skill::miter_corner_trim::Input in;
    in.corner_xyz        = { 50.0, 50.0, 50.0 };
    in.member_a_axis_dir = { 1.0, 0.0, 0.0 };
    in.member_b_axis_dir = { 0.0, 1.0, 0.0 };

    auto out = skill::miter_corner_trim::apply(*stock, in);
    EXPECT_EQ(out.signature.skill_id, std::string("miter_corner_trim"));
    EXPECT_TRUE(out.signature.pattern.value("is_compound", false));
    EXPECT_EQ(out.signature.pattern.value("profile_kind", std::string{}),
              std::string("miter"));
}

// ─── 5. Recognize replays history ────────────────────────────────────────
TEST(SkillMiterCornerTrim, RecognizeReplaysHistory)
{
    auto stock = skill::createCuboidStock(100.0, 100.0, 100.0);

    skill::miter_corner_trim::Input in;
    in.corner_xyz        = { 50.0, 50.0, 50.0 };
    in.member_a_axis_dir = { 1.0, 0.0, 0.0 };
    in.member_b_axis_dir = { 0.0, 1.0, 0.0 };

    auto out   = skill::miter_corner_trim::apply(*stock, in);
    auto cands = skill::miter_corner_trim::recognize(*out.workpiece);
    ASSERT_GE(cands.size(), 1u);
    EXPECT_EQ(cands.front().skill_id, std::string("miter_corner_trim"));
    EXPECT_GT(cands.front().confidence, 0.2);
}
