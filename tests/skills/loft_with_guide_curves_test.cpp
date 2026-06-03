// @lat: [[process/test-strategy#skill round-trip]]
//
// loft_with_guide_curves — skin a solid through exactly 2 closed
// cross-sections along a guide spine, using BRepOffsetAPI_MakePipeShell.
//
// Tests (5):
//   1. Apply adds real volume ≈ 0.5*(A0+A1)*L (±25 %).
//   2. DFM rejects single section.
//   3. DFM rejects empty guide_curve.
//   4. Signature carries section_count=2 + path_length + derived_volume.
//   5. Recognize replays history.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/loft_with_guide_curves.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

#include <cmath>
#include <utility>
#include <vector>

using namespace koocadcam;

namespace {

double volumeOf(const TopoDS_Shape& s)
{
    GProp_GProps p;
    BRepGProp::VolumeProperties(s, p);
    return p.Mass();
}

std::vector<std::pair<double,double>> sq(double half)
{
    return {
        { -half, -half }, { half, -half }, { half, half }, { -half, half }
    };
}

}  // namespace

// ─── 1. Apply adds real lofted volume ─────────────────────────────────────
TEST(SkillLoftWithGuideCurves, ApplyAddsRealVolume)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 5.0);
    const double v0 = volumeOf(stock->shape());

    skill::loft_with_guide_curves::Input in;
    in.sections    = { sq(5.0), sq(5.0) };          // 100 mm² each
    // Straight guide from far above the stock, length = 20 mm.
    in.guide_curve = {
        gp_Pnt(100.0, 100.0, 40.0),
        gp_Pnt(100.0, 100.0, 60.0),
    };

    auto out = skill::loft_with_guide_curves::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    const double v1 = volumeOf(out.workpiece->shape());

    // Expected ≈ 100 × 20 = 2000 mm³.
    const double expected = 100.0 * 20.0;
    EXPECT_NEAR(v1 - v0, expected, expected * 0.25);
}

// ─── 2. DFM rejects single section ────────────────────────────────────────
TEST(SkillLoftWithGuideCurves, ValidateRejectsSingleSection)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 5.0);

    skill::loft_with_guide_curves::Input in;
    in.sections    = { sq(5.0) };                // wrong size
    in.guide_curve = {
        gp_Pnt(0.0, 0.0, 0.0), gp_Pnt(0.0, 0.0, 20.0),
    };

    auto r = skill::loft_with_guide_curves::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_THROW(skill::loft_with_guide_curves::apply(*stock, in),
                 skill::SkillError);
}

// ─── 3. DFM rejects empty guide ───────────────────────────────────────────
TEST(SkillLoftWithGuideCurves, ValidateRejectsEmptyGuide)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 5.0);

    skill::loft_with_guide_curves::Input in;
    in.sections    = { sq(5.0), sq(5.0) };
    in.guide_curve = {};                       // empty

    auto r = skill::loft_with_guide_curves::validate(*stock, in);
    EXPECT_FALSE(r.passed);
}

// ─── 4. Signature pattern ─────────────────────────────────────────────────
TEST(SkillLoftWithGuideCurves, SignatureCompound)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 5.0);

    skill::loft_with_guide_curves::Input in;
    in.sections    = { sq(5.0), sq(5.0) };
    in.guide_curve = {
        gp_Pnt(100.0, 100.0, 40.0),
        gp_Pnt(100.0, 100.0, 60.0),
    };

    auto out = skill::loft_with_guide_curves::apply(*stock, in);
    EXPECT_EQ(out.signature.skill_id, std::string("loft_with_guide_curves"));
    EXPECT_TRUE(out.signature.pattern.value("is_compound", false));
    EXPECT_EQ(out.signature.pattern.value("section_count", 0), 2);
    EXPECT_NEAR(out.signature.pattern.value("path_length_mm", 0.0), 20.0, 1e-6);
    EXPECT_GT(out.signature.pattern.value("derived_volume_mm3", 0.0), 1500.0);
}

// ─── 5. Recognize replays history ─────────────────────────────────────────
TEST(SkillLoftWithGuideCurves, RecognizeReplaysHistory)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 5.0);

    skill::loft_with_guide_curves::Input in;
    in.sections    = { sq(5.0), sq(5.0) };
    in.guide_curve = {
        gp_Pnt(100.0, 100.0, 40.0),
        gp_Pnt(100.0, 100.0, 60.0),
    };

    auto out   = skill::loft_with_guide_curves::apply(*stock, in);
    auto cands = skill::loft_with_guide_curves::recognize(*out.workpiece);
    ASSERT_GE(cands.size(), 1u);
    EXPECT_EQ(cands.front().skill_id, std::string("loft_with_guide_curves"));
    EXPECT_GT(cands.front().confidence, 0.4);
}
