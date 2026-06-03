// @lat: [[process/test-strategy#skill round-trip]]
//
// intermittent_weld_bead — replicated short fillet weld beads at intervals.
//
// Tests (5):
//   1. Apply: real volume ≈ bead_count × bead_length × height² (±25 %).
//   2. DFM rejects bead_count = 0.
//   3. DFM rejects negative gap_length.
//   4. Signature pattern carries is_compound + bead_count.
//   5. Recognize replays the feature from history.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/intermittent_weld_bead.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

#include <array>

using namespace koocadcam;

namespace {

double volumeOf(const TopoDS_Shape& s)
{
    GProp_GProps p;
    BRepGProp::VolumeProperties(s, p);
    return p.Mass();
}

}  // namespace

// ─── 1. Apply: real volume ≈ N × bL × h² ─────────────────────────────────
TEST(SkillIntermittentWeldBead, ApplyAddsRealVolume)
{
    auto stock = skill::createCuboidStock(20.0, 20.0, 20.0);
    const double v0 = volumeOf(stock->shape());

    skill::intermittent_weld_bead::Input in;
    in.bead_start_xyz     = { 200.0, 0.0, 0.0 };
    in.bead_direction_xyz = { 0.0,   0.0, 1.0 };
    in.bead_length_mm     = 25.0;
    in.gap_length_mm      = 50.0;
    in.bead_height_mm     = 5.0;
    in.bead_count         = 4;

    auto out = skill::intermittent_weld_bead::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    const double v1 = volumeOf(out.workpiece->shape());

    // Each bead = 25 × 5 × 5 = 625 mm³ ; 4 beads = 2500 mm³.
    const double expected = 4.0 * 25.0 * 5.0 * 5.0;
    EXPECT_NEAR(v1 - v0, expected, expected * 0.25);
}

// ─── 2. DFM rejects bead_count = 0 ───────────────────────────────────────
TEST(SkillIntermittentWeldBead, ValidateRejectsZeroCount)
{
    auto stock = skill::createCuboidStock(20.0, 20.0, 20.0);

    skill::intermittent_weld_bead::Input in;
    in.bead_count     = 0;
    in.bead_length_mm = 25.0;
    in.gap_length_mm  = 10.0;
    in.bead_height_mm = 5.0;

    auto r = skill::intermittent_weld_bead::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_THROW(skill::intermittent_weld_bead::apply(*stock, in),
                 skill::SkillError);
}

// ─── 3. DFM rejects negative gap ─────────────────────────────────────────
TEST(SkillIntermittentWeldBead, ValidateRejectsNegativeGap)
{
    auto stock = skill::createCuboidStock(20.0, 20.0, 20.0);

    skill::intermittent_weld_bead::Input in;
    in.bead_count     = 3;
    in.bead_length_mm = 25.0;
    in.gap_length_mm  = -5.0;
    in.bead_height_mm = 5.0;

    auto r = skill::intermittent_weld_bead::validate(*stock, in);
    EXPECT_FALSE(r.passed);
}

// ─── 4. Signature pattern ────────────────────────────────────────────────
TEST(SkillIntermittentWeldBead, SignatureCompound)
{
    auto stock = skill::createCuboidStock(20.0, 20.0, 20.0);

    skill::intermittent_weld_bead::Input in;
    in.bead_start_xyz     = { 200.0, 0.0, 0.0 };
    in.bead_direction_xyz = { 0.0,   0.0, 1.0 };
    in.bead_length_mm     = 20.0;
    in.gap_length_mm      = 40.0;
    in.bead_height_mm     = 4.0;
    in.bead_count         = 5;

    auto out = skill::intermittent_weld_bead::apply(*stock, in);
    EXPECT_EQ(out.signature.skill_id,
              std::string("intermittent_weld_bead"));
    EXPECT_TRUE(out.signature.pattern.value("is_compound", false));
    EXPECT_EQ(out.signature.pattern.value("profile_kind", std::string{}),
              std::string("bead"));
    EXPECT_EQ(out.signature.pattern.value("bead_count", 0), 5);
}

// ─── 5. Recognize replays history ────────────────────────────────────────
TEST(SkillIntermittentWeldBead, RecognizeReplaysHistory)
{
    auto stock = skill::createCuboidStock(20.0, 20.0, 20.0);

    skill::intermittent_weld_bead::Input in;
    in.bead_start_xyz     = { 200.0, 0.0, 0.0 };
    in.bead_direction_xyz = { 0.0,   0.0, 1.0 };
    in.bead_length_mm     = 25.0;
    in.gap_length_mm      = 30.0;
    in.bead_height_mm     = 5.0;
    in.bead_count         = 3;

    auto out   = skill::intermittent_weld_bead::apply(*stock, in);
    auto cands = skill::intermittent_weld_bead::recognize(*out.workpiece);
    ASSERT_GE(cands.size(), 1u);
    EXPECT_EQ(cands.front().skill_id,
              std::string("intermittent_weld_bead"));
    EXPECT_GT(cands.front().confidence, 0.2);
}
