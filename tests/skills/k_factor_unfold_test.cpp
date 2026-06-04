// @lat: [[process/test-strategy#skill round-trip]]
//
// k_factor_unfold — unfold bent sheet using K-factor neutral axis.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/k_factor_unfold.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

#include <cmath>

using namespace koocadcam;

namespace {
double volumeOf(const TopoDS_Shape& s) {
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}
}  // namespace

// ── 1. Apply: unfold a 90° bend on 100×80×1.5 sheet, K=0.42 ────────────
TEST(SkillKFactorUnfold, ApplyProducesValidSolid)
{
    auto stock = skill::createCuboidStock(100.0, 80.0, 1.5);

    skill::k_factor_unfold::Input in;
    in.bend_axis_origin   = { 50.0, 0.0, 0.0 };
    in.bend_axis_dir      = { 0.0, 1.0, 0.0 };
    in.bend_angle_deg     = 90.0;
    in.sheet_thickness_mm = 1.5;
    in.k_factor           = 0.42;
    in.inner_radius_mm    = 1.0;

    auto out = skill::k_factor_unfold::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    EXPECT_GT(volumeOf(out.workpiece->shape()), 0.0);

    EXPECT_EQ(out.signature.skill_id, std::string("k_factor_unfold"));
    EXPECT_TRUE(out.signature.pattern["is_compound"].get<bool>());

    // Developed length: BA = (π/2)·(1.0 + 0.42·1.5) = (π/2)·1.63 ≈ 2.5604
    const double expectedBA = (M_PI / 2.0) * (1.0 + 0.42 * 1.5);
    EXPECT_NEAR(out.signature.pattern["developed_length_mm"].get<double>(),
                expectedBA, 1e-3);
}

// ── 2. DFM-K-FACTOR: K outside [0.3, 0.5] ──────────────────────────────
TEST(SkillKFactorUnfold, ValidateRejectsBadKFactor)
{
    auto stock = skill::createCuboidStock(100.0, 80.0, 1.5);

    skill::k_factor_unfold::Input in;
    in.bend_axis_dir      = { 0.0, 1.0, 0.0 };
    in.bend_angle_deg     = 45.0;
    in.sheet_thickness_mm = 1.5;
    in.k_factor           = 0.6;       // > 0.5
    in.inner_radius_mm    = 1.0;

    auto r = skill::k_factor_unfold::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-K-FACTOR") { found = true; break; }
    EXPECT_TRUE(found);
    EXPECT_THROW(skill::k_factor_unfold::apply(*stock, in), skill::SkillError);
}

// ── 3. DFM-K-SHEET: too-thick stock ────────────────────────────────────
TEST(SkillKFactorUnfold, ValidateRejectsTooThickSheet)
{
    auto stock = skill::createCuboidStock(80.0, 60.0, 8.0);

    skill::k_factor_unfold::Input in;
    in.bend_axis_dir      = { 0.0, 1.0, 0.0 };
    in.bend_angle_deg     = 90.0;
    in.sheet_thickness_mm = 8.0;
    in.k_factor           = 0.42;
    in.inner_radius_mm    = 2.0;

    auto r = skill::k_factor_unfold::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-K-SHEET") { found = true; break; }
    EXPECT_TRUE(found);
}

// ── 4. DFM-K-ANGLE: 0 or > 180 rejected ────────────────────────────────
TEST(SkillKFactorUnfold, ValidateRejectsBadAngle)
{
    auto stock = skill::createCuboidStock(100.0, 80.0, 1.5);

    skill::k_factor_unfold::Input in;
    in.bend_axis_dir      = { 0.0, 1.0, 0.0 };
    in.bend_angle_deg     = 250.0;     // > 180
    in.sheet_thickness_mm = 1.5;
    in.k_factor           = 0.42;
    in.inner_radius_mm    = 1.0;

    auto r = skill::k_factor_unfold::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-K-ANGLE") { found = true; break; }
    EXPECT_TRUE(found);
}

// ── 5. Recognize: flat blank has no horizontal-axis cylinders ──────────
TEST(SkillKFactorUnfold, RecognizeFlatBlank)
{
    auto stock = skill::createCuboidStock(100.0, 80.0, 1.5);
    auto cands = skill::k_factor_unfold::recognize(*stock);

    ASSERT_FALSE(cands.empty());
    const auto& c = cands.front();
    EXPECT_EQ(c.skill_id, std::string("k_factor_unfold"));
    EXPECT_TRUE(c.matched_geometry["is_flat_blank"].get<bool>());
}
