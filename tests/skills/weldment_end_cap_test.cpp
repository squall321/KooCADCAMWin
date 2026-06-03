// @lat: [[process/test-strategy#skill round-trip]]
//
// weldment_end_cap — closed plate at the end of a tube/profile.
//
// Tests (5):
//   1. Apply: square end cap adds real volume ≈ D² × t.
//   2. DFM rejects bad profile_kind.
//   3. DFM rejects zero cap thickness.
//   4. Signature pattern carries is_compound + profile_kind.
//   5. Recognize replays the feature from history.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/weldment_end_cap.hpp"

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

// ─── 1. Apply: square cap adds real volume ────────────────────────────────
TEST(SkillWeldmentEndCap, ApplySquareCapAddsVolume)
{
    auto stock = skill::createCuboidStock(20.0, 20.0, 20.0);
    const double v0 = volumeOf(stock->shape());

    skill::weldment_end_cap::Input in;
    in.tube_axis_origin = { 200.0, 0.0, 0.0 };
    in.tube_axis_dir    = { 1.0,   0.0, 0.0 };
    in.cap_thickness_mm = 5.0;
    in.profile_kind     = "square";
    in.profile_dim_mm   = 50.0;

    auto out = skill::weldment_end_cap::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    const double v1 = volumeOf(out.workpiece->shape());

    const double expected = 50.0 * 50.0 * 5.0;  // 12 500 mm³
    EXPECT_NEAR(v1 - v0, expected, expected * 0.20);
}

// ─── 2. DFM rejects bad profile_kind ──────────────────────────────────────
TEST(SkillWeldmentEndCap, ValidateRejectsBadKind)
{
    auto stock = skill::createCuboidStock(20.0, 20.0, 20.0);

    skill::weldment_end_cap::Input in;
    in.profile_kind     = "octagon";
    in.profile_dim_mm   = 30.0;
    in.cap_thickness_mm = 3.0;

    auto r = skill::weldment_end_cap::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_THROW(skill::weldment_end_cap::apply(*stock, in),
                 skill::SkillError);
}

// ─── 3. DFM rejects zero thickness ────────────────────────────────────────
TEST(SkillWeldmentEndCap, ValidateRejectsZeroThickness)
{
    auto stock = skill::createCuboidStock(20.0, 20.0, 20.0);

    skill::weldment_end_cap::Input in;
    in.profile_kind     = "round";
    in.profile_dim_mm   = 30.0;
    in.cap_thickness_mm = 0.0;

    auto r = skill::weldment_end_cap::validate(*stock, in);
    EXPECT_FALSE(r.passed);
}

// ─── 4. Signature pattern ─────────────────────────────────────────────────
TEST(SkillWeldmentEndCap, SignatureCompound)
{
    auto stock = skill::createCuboidStock(20.0, 20.0, 20.0);

    skill::weldment_end_cap::Input in;
    in.tube_axis_origin = { 200.0, 0.0, 0.0 };
    in.tube_axis_dir    = { 1.0,   0.0, 0.0 };
    in.cap_thickness_mm = 5.0;
    in.profile_kind     = "round";
    in.profile_dim_mm   = 40.0;

    auto out = skill::weldment_end_cap::apply(*stock, in);
    EXPECT_EQ(out.signature.skill_id, std::string("weldment_end_cap"));
    EXPECT_TRUE(out.signature.pattern.value("is_compound", false));
    EXPECT_EQ(out.signature.pattern.value("profile_kind", std::string{}),
              std::string("round"));
    EXPECT_NEAR(out.signature.pattern.value("cap_thickness_mm", 0.0), 5.0, 1e-9);
}

// ─── 5. Recognize replays history ─────────────────────────────────────────
TEST(SkillWeldmentEndCap, RecognizeReplaysHistory)
{
    auto stock = skill::createCuboidStock(20.0, 20.0, 20.0);

    skill::weldment_end_cap::Input in;
    in.tube_axis_origin = { 200.0, 0.0, 0.0 };
    in.tube_axis_dir    = { 1.0,   0.0, 0.0 };
    in.cap_thickness_mm = 4.0;
    in.profile_kind     = "square";
    in.profile_dim_mm   = 30.0;

    auto out   = skill::weldment_end_cap::apply(*stock, in);
    auto cands = skill::weldment_end_cap::recognize(*out.workpiece);
    ASSERT_GE(cands.size(), 1u);
    EXPECT_EQ(cands.front().skill_id, std::string("weldment_end_cap"));
    EXPECT_GT(cands.front().confidence, 0.3);
}
