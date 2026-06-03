// @lat: [[process/test-strategy#skill round-trip]]
//
// loft_between_profiles — skin a solid through ≥ 2 closed cross-sections
// at different Z, using ThruSections.
//
// Tests (5):
//   1. Apply adds real volume ≈ trapezoidal sum 0.5*(A_k + A_k+1)*Δz (±25 %).
//   2. DFM rejects single-section input.
//   3. DFM rejects mismatched z_positions size.
//   4. Signature pattern carries section_count + path_length + derived_volume.
//   5. Recognize replays history.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/loft_between_profiles.hpp"

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

std::vector<std::pair<double,double>> centeredSquare(double half, double cx, double cy)
{
    return {
        { cx - half, cy - half },
        { cx + half, cy - half },
        { cx + half, cy + half },
        { cx - half, cy + half },
    };
}

}  // namespace

// ─── 1. Apply adds real lofted volume ─────────────────────────────────────
TEST(SkillLoftBetweenProfiles, ApplyAddsRealVolume)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 5.0);
    const double v0 = volumeOf(stock->shape());

    skill::loft_between_profiles::Input in;
    // Two square sections placed well above the stock, far from origin so
    // the lofted solid does not intersect the stock at all (clean add).
    in.sections = {
        centeredSquare(5.0, 100.0, 100.0),   // 10x10, area=100, z=40
        centeredSquare(5.0, 100.0, 100.0),   // same, z=60 → prism 100x20
    };
    in.section_z_positions = { 40.0, 60.0 };
    in.is_solid            = true;

    auto out = skill::loft_between_profiles::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    const double v1 = volumeOf(out.workpiece->shape());

    // Expected ≈ 100 × 20 = 2000 mm³.
    const double expected = 100.0 * 20.0;
    EXPECT_NEAR(v1 - v0, expected, expected * 0.25);
}

// ─── 2. DFM rejects single section ────────────────────────────────────────
TEST(SkillLoftBetweenProfiles, ValidateRejectsSingleSection)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 5.0);

    skill::loft_between_profiles::Input in;
    in.sections            = { centeredSquare(5.0, 0.0, 0.0) };
    in.section_z_positions = { 10.0 };

    auto r = skill::loft_between_profiles::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_THROW(skill::loft_between_profiles::apply(*stock, in),
                 skill::SkillError);
}

// ─── 3. DFM rejects z-size mismatch ───────────────────────────────────────
TEST(SkillLoftBetweenProfiles, ValidateRejectsZMismatch)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 5.0);

    skill::loft_between_profiles::Input in;
    in.sections = {
        centeredSquare(5.0, 0.0, 0.0),
        centeredSquare(5.0, 0.0, 0.0),
    };
    in.section_z_positions = { 10.0 };   // wrong size

    auto r = skill::loft_between_profiles::validate(*stock, in);
    EXPECT_FALSE(r.passed);
}

// ─── 4. Signature pattern ─────────────────────────────────────────────────
TEST(SkillLoftBetweenProfiles, SignatureCompound)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 5.0);

    skill::loft_between_profiles::Input in;
    in.sections = {
        centeredSquare(5.0, 100.0, 100.0),
        centeredSquare(5.0, 100.0, 100.0),
    };
    in.section_z_positions = { 40.0, 60.0 };

    auto out = skill::loft_between_profiles::apply(*stock, in);
    EXPECT_EQ(out.signature.skill_id, std::string("loft_between_profiles"));
    EXPECT_TRUE(out.signature.pattern.value("is_compound", false));
    EXPECT_EQ(out.signature.pattern.value("section_count", 0), 2);
    EXPECT_NEAR(out.signature.pattern.value("path_length_mm", 0.0), 20.0, 1e-6);
    EXPECT_GT(out.signature.pattern.value("derived_volume_mm3", 0.0), 1500.0);
}

// ─── 5. Recognize replays history ─────────────────────────────────────────
TEST(SkillLoftBetweenProfiles, RecognizeReplaysHistory)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 5.0);

    skill::loft_between_profiles::Input in;
    in.sections = {
        centeredSquare(5.0, 100.0, 100.0),
        centeredSquare(5.0, 100.0, 100.0),
    };
    in.section_z_positions = { 40.0, 60.0 };

    auto out   = skill::loft_between_profiles::apply(*stock, in);
    auto cands = skill::loft_between_profiles::recognize(*out.workpiece);
    ASSERT_GE(cands.size(), 1u);
    EXPECT_EQ(cands.front().skill_id, std::string("loft_between_profiles"));
    EXPECT_GT(cands.front().confidence, 0.4);
}
