// @lat: [[process/test-strategy#skill round-trip]]
//
// linear_pattern — count_x × count_y grid of cylindrical cuts.
//
// Cases (5):
//   1. 3x1 pattern removes ~3 × π r² × depth volume on a flat plate.
//   2. 2x2 grid removes ~4 × per-hole volume.
//   3. DFM rejects pitch ≤ hole_dia.
//   4. DFM rejects count_x = 0.
//   5. recognize finds the pattern (history replay).

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/linear_pattern.hpp"

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

// ─── 1. 3-hole linear row volume ───────────────────────────────────────────
TEST(SkillLinearPattern, ThreeInRowVolume)
{
    auto stock = skill::createCuboidStock(100.0, 50.0, 10.0);
    const double volBefore = volumeOf(stock->shape());

    skill::linear_pattern::Input in;
    in.face          = skill::FaceByNormal{ gp_Dir(0, 0, 1), 5.0, "largest" };
    in.hole_dia_mm   = 4.0;
    in.hole_depth_mm = 5.0;
    in.count_x       = 3;
    in.pitch_x_mm    = 15.0;
    in.count_y       = 1;
    in.pitch_y_mm    = 0.0;
    in.start_x_mm    = -15.0;        // -15, 0, +15 from face center
    in.start_y_mm    = 0.0;

    auto out = skill::linear_pattern::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    const double volAfter = volumeOf(out.workpiece->shape());
    const double removed  = volBefore - volAfter;
    const double per      = M_PI * 2.0 * 2.0 * 5.0;
    const double expected = 3.0 * per;
    EXPECT_NEAR(removed, expected, expected * 0.15)
        << "3 × π r² × depth should match within 15%";

    EXPECT_EQ(out.signature.pattern["instance_count"].get<int>(), 3);
    EXPECT_TRUE(out.signature.pattern["is_pattern"].get<bool>());
}

// ─── 2. 2x2 grid volume ────────────────────────────────────────────────────
TEST(SkillLinearPattern, TwoByTwoGridVolume)
{
    auto stock = skill::createCuboidStock(100.0, 100.0, 10.0);
    const double volBefore = volumeOf(stock->shape());

    skill::linear_pattern::Input in;
    in.face          = skill::FaceByNormal{ gp_Dir(0, 0, 1), 5.0, "largest" };
    in.hole_dia_mm   = 3.0;
    in.hole_depth_mm = 4.0;
    in.count_x       = 2;
    in.pitch_x_mm    = 20.0;
    in.count_y       = 2;
    in.pitch_y_mm    = 20.0;
    in.start_x_mm    = -10.0;
    in.start_y_mm    = -10.0;

    auto out = skill::linear_pattern::apply(*stock, in);
    const double removed  = volBefore - volumeOf(out.workpiece->shape());
    const double per      = M_PI * 1.5 * 1.5 * 4.0;
    const double expected = 4.0 * per;
    EXPECT_NEAR(removed, expected, expected * 0.15);

    EXPECT_EQ(out.signature.pattern["instance_count"].get<int>(), 4);
}

// ─── 3. DFM rejects overlapping pitch ──────────────────────────────────────
TEST(SkillLinearPattern, ValidateRejectsPitchTooSmall)
{
    auto stock = skill::createCuboidStock(100.0, 50.0, 10.0);
    skill::linear_pattern::Input in;
    in.face          = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.hole_dia_mm   = 5.0;
    in.hole_depth_mm = 4.0;
    in.count_x       = 3;
    in.pitch_x_mm    = 4.0;          // <= dia — overlap
    auto r = skill::linear_pattern::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings) if (f.code == "DFM-PATT-1") found = true;
    EXPECT_TRUE(found);
    EXPECT_THROW(skill::linear_pattern::apply(*stock, in), skill::SkillError);
}

// ─── 4. DFM rejects count_x = 0 ────────────────────────────────────────────
TEST(SkillLinearPattern, ValidateRejectsZeroCount)
{
    auto stock = skill::createCuboidStock(100.0, 50.0, 10.0);
    skill::linear_pattern::Input in;
    in.face          = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.hole_dia_mm   = 3.0;
    in.hole_depth_mm = 4.0;
    in.count_x       = 0;
    in.pitch_x_mm    = 15.0;
    auto r = skill::linear_pattern::validate(*stock, in);
    EXPECT_FALSE(r.passed);
}

// ─── 5. Recognize finds the pattern (history replay) ───────────────────────
TEST(SkillLinearPattern, RecognizeFindsPattern)
{
    auto stock = skill::createCuboidStock(100.0, 50.0, 10.0);
    skill::linear_pattern::Input in;
    in.face          = skill::FaceByNormal{ gp_Dir(0, 0, 1), 5.0, "largest" };
    in.hole_dia_mm   = 4.0;
    in.hole_depth_mm = 5.0;
    in.count_x       = 3;
    in.pitch_x_mm    = 15.0;
    in.start_x_mm    = -15.0;

    auto out = skill::linear_pattern::apply(*stock, in);
    auto cands = skill::linear_pattern::recognize(*out.workpiece);
    ASSERT_GE(cands.size(), 1u);
    EXPECT_EQ(cands[0].skill_id, std::string("linear_pattern"));
    EXPECT_GT(cands[0].confidence, 0.5);
}
