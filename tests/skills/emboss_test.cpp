// @lat: [[process/test-strategy#skill round-trip]]
//
// emboss skill — raised localized feature on a sheet's top face.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/emboss.hpp"

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

// ── 1. Circular emboss adds a cylinder of expected volume ──────────────
TEST(SkillEmboss, ApplyCircleAddsBump)
{
    auto stock = skill::createCuboidStock(100.0, 80.0, 1.5);

    skill::emboss::Input in;
    in.position_x_mm = 50.0;
    in.position_y_mm = 40.0;
    in.shape         = skill::emboss::Shape::Circle;
    in.diameter_mm   = 10.0;
    in.height_mm     = 2.0;          // 2 ≤ 3 × 1.5 = 4.5

    auto out = skill::emboss::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    const double expected = M_PI * 5.0 * 5.0 * 2.0;
    const double added = volumeOf(out.workpiece->shape()) - volumeOf(stock->shape());
    EXPECT_NEAR(added, expected, expected * 0.05);

    EXPECT_EQ(out.signature.skill_id, std::string("emboss"));
    EXPECT_EQ(out.signature.pattern["shape"].get<std::string>(), "circle");
}

// ── 2. Rectangular emboss with corner_r builds valid solid ─────────────
TEST(SkillEmboss, ApplyRectangleWithCornerR)
{
    auto stock = skill::createCuboidStock(100.0, 80.0, 1.5);

    skill::emboss::Input in;
    in.position_x_mm = 50.0;
    in.position_y_mm = 40.0;
    in.shape         = skill::emboss::Shape::Rectangle;
    in.length_mm     = 20.0;
    in.width_mm      = 10.0;
    in.height_mm     = 1.0;
    in.corner_r_mm   = 1.5;            // ≥ sheet_thickness = 1.5

    auto out = skill::emboss::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    EXPECT_EQ(out.signature.pattern["shape"].get<std::string>(), "rectangle");
}

// ── 3. DFM-E-DEPTH: too tall emboss rejected ──────────────────────────
TEST(SkillEmboss, ValidateRejectsTooTall)
{
    auto stock = skill::createCuboidStock(100.0, 80.0, 1.5);

    skill::emboss::Input in;
    in.position_x_mm = 50.0;
    in.position_y_mm = 40.0;
    in.shape         = skill::emboss::Shape::Circle;
    in.diameter_mm   = 10.0;
    in.height_mm     = 6.0;          // > 3 × 1.5 = 4.5

    auto r = skill::emboss::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-E-DEPTH") { found = true; break; }
    EXPECT_TRUE(found);
    EXPECT_THROW(skill::emboss::apply(*stock, in), skill::SkillError);
}

// ── 4. DFM-E-CORNER: corner_r < thickness rejected ────────────────────
TEST(SkillEmboss, ValidateRejectsTooSharpCorner)
{
    auto stock = skill::createCuboidStock(100.0, 80.0, 1.5);

    skill::emboss::Input in;
    in.position_x_mm = 50.0;
    in.position_y_mm = 40.0;
    in.shape         = skill::emboss::Shape::Rectangle;
    in.length_mm     = 15.0;
    in.width_mm      = 10.0;
    in.height_mm     = 1.0;
    in.corner_r_mm   = 0.5;          // < 1.5

    auto r = skill::emboss::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-E-CORNER") { found = true; break; }
    EXPECT_TRUE(found);
}

// ── 5. Recognize finds the protruding feature ─────────────────────────
TEST(SkillEmboss, RecognizeFindsEmboss)
{
    auto stock = skill::createCuboidStock(100.0, 80.0, 1.5);

    skill::emboss::Input in;
    in.position_x_mm = 60.0;
    in.position_y_mm = 30.0;
    in.shape         = skill::emboss::Shape::Circle;
    in.diameter_mm   = 8.0;
    in.height_mm     = 2.0;

    auto out = skill::emboss::apply(*stock, in);
    auto cands = skill::emboss::recognize(*out.workpiece);

    ASSERT_FALSE(cands.empty());
    const auto& c = cands.front();
    EXPECT_EQ(c.skill_id, std::string("emboss"));
    EXPECT_NEAR(c.recovered_params["position_x_mm"].get<double>(), 60.0, 1e-3);
    EXPECT_NEAR(c.recovered_params["position_y_mm"].get<double>(), 30.0, 1e-3);
    EXPECT_NEAR(c.recovered_params["height_mm"].get<double>(), 2.0, 0.05);
    EXPECT_NEAR(c.recovered_params["dims"]["diameter_mm"].get<double>(), 8.0, 0.05);
}
