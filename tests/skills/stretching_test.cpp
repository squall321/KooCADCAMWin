// @lat: [[process/test-strategy#skill round-trip]]
//
// stretching skill — localized depression that thins material.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/stretching.hpp"

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

// ── 1. Apply: 20% reduction over a circle ─────────────────────────────
TEST(SkillStretching, ApplyCreatesDepression)
{
    auto stock = skill::createCuboidStock(100.0, 80.0, 1.5);

    skill::stretching::Input in;
    in.entry_face                       = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm                    = 50.0;
    in.position_y_mm                    = 40.0;
    in.stretch_radius_mm                = 10.0;       // > 5 × 1.5 = 7.5
    in.target_thickness_reduction_pct   = 20.0;       // 1.5 × 0.2 = 0.3 mm depth

    auto out = skill::stretching::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    // Expected depressed volume ≈ π × r² × depression_depth
    const double depressionDepth = 1.5 * 0.20;
    const double expected = M_PI * 10.0 * 10.0 * depressionDepth;
    const double removed  = volumeOf(stock->shape()) - volumeOf(out.workpiece->shape());
    EXPECT_NEAR(removed, expected, expected * 0.05);

    EXPECT_EQ(out.signature.skill_id, std::string("stretching"));
    EXPECT_NEAR(out.signature.pattern["depression_depth_mm"].get<double>(),
                depressionDepth, 1e-6);
}

// ── 2. DFM-ST-REDUCTION: > 30% reduction rejected ─────────────────────
TEST(SkillStretching, ValidateRejectsExcessiveReduction)
{
    auto stock = skill::createCuboidStock(100.0, 80.0, 1.5);

    skill::stretching::Input in;
    in.entry_face                     = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm                  = 50.0;
    in.position_y_mm                  = 40.0;
    in.stretch_radius_mm              = 10.0;
    in.target_thickness_reduction_pct = 40.0;        // > 30

    auto r = skill::stretching::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-ST-REDUCTION") { found = true; break; }
    EXPECT_TRUE(found);
    EXPECT_THROW(skill::stretching::apply(*stock, in), skill::SkillError);
}

// ── 3. DFM-ST-RADIUS: stretch_radius ≤ 5 × thickness rejected ─────────
TEST(SkillStretching, ValidateRejectsTooSmallRadius)
{
    auto stock = skill::createCuboidStock(100.0, 80.0, 1.5);

    skill::stretching::Input in;
    in.entry_face                     = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm                  = 50.0;
    in.position_y_mm                  = 40.0;
    in.stretch_radius_mm              = 5.0;         // ≤ 5 × 1.5 = 7.5
    in.target_thickness_reduction_pct = 20.0;

    auto r = skill::stretching::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-ST-RADIUS") { found = true; break; }
    EXPECT_TRUE(found);
}

// ── 4. DFM-INPUT: off-sheet region rejected ──────────────────────────
TEST(SkillStretching, ValidateRejectsOffSheet)
{
    auto stock = skill::createCuboidStock(100.0, 80.0, 1.5);

    skill::stretching::Input in;
    in.entry_face                     = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm                  = 95.0;        // off the right
    in.position_y_mm                  = 40.0;
    in.stretch_radius_mm              = 10.0;
    in.target_thickness_reduction_pct = 20.0;

    auto r = skill::stretching::validate(*stock, in);
    EXPECT_FALSE(r.passed);
}

// ── 5. Recognize finds the circular depression ────────────────────────
TEST(SkillStretching, RecognizeFindsDepression)
{
    auto stock = skill::createCuboidStock(100.0, 80.0, 1.5);

    skill::stretching::Input in;
    in.entry_face                     = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm                  = 60.0;
    in.position_y_mm                  = 30.0;
    in.stretch_radius_mm              = 12.0;
    in.target_thickness_reduction_pct = 20.0;

    auto out = skill::stretching::apply(*stock, in);
    auto cands = skill::stretching::recognize(*out.workpiece);
    ASSERT_FALSE(cands.empty());
    const auto& c = cands.front();
    EXPECT_EQ(c.skill_id, std::string("stretching"));
    EXPECT_NEAR(c.recovered_params["position_x_mm"].get<double>(), 60.0, 1e-3);
    EXPECT_NEAR(c.recovered_params["position_y_mm"].get<double>(), 30.0, 1e-3);
    EXPECT_NEAR(c.recovered_params["stretch_radius_mm"].get<double>(), 12.0, 0.1);
}
