// @lat: [[process/test-strategy#skill round-trip]]
//
// blanking skill — cut a shaped blank from a sheet via INTERSECTION.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/blanking.hpp"

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

// ── 1. Apply: result volume equals blank polygon × thickness ──────────
TEST(SkillBlanking, ApplyChangesGeometryOrPreserves)
{
    auto stock = skill::createCuboidStock(100.0, 80.0, 1.5);

    skill::blanking::Input in;
    in.shape        = skill::blanking::Shape::Rectangle;
    in.center_x_mm  = 50.0;
    in.center_y_mm  = 40.0;
    in.length_mm    = 30.0;
    in.width_mm     = 20.0;

    auto out = skill::blanking::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    // The result's volume should equal blank area × thickness.
    const double expected = 30.0 * 20.0 * 1.5;
    EXPECT_NEAR(volumeOf(out.workpiece->shape()), expected, expected * 0.02);

    // The result MUST be smaller than the original sheet (intersection).
    EXPECT_LT(volumeOf(out.workpiece->shape()), volumeOf(stock->shape()));
}

// ── 2. Validate rejects bad input: oversize blank ──────────────────────
TEST(SkillBlanking, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(100.0, 80.0, 1.5);

    skill::blanking::Input in;
    in.shape        = skill::blanking::Shape::Rectangle;
    in.center_x_mm  = 50.0;
    in.center_y_mm  = 40.0;
    in.length_mm    = 200.0;     // > sheet length
    in.width_mm     = 20.0;

    auto r = skill::blanking::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_THROW(skill::blanking::apply(*stock, in), skill::SkillError);

    // Min-feature rule also: a blank narrower than 1.5 × thickness.
    skill::blanking::Input in2;
    in2.shape        = skill::blanking::Shape::Rectangle;
    in2.center_x_mm  = 50.0;
    in2.center_y_mm  = 40.0;
    in2.length_mm    = 30.0;
    in2.width_mm     = 1.0;       // < 1.5 × 1.5 = 2.25
    auto r2 = skill::blanking::validate(*stock, in2);
    EXPECT_FALSE(r2.passed);
    bool found = false;
    for (const auto& f : r2.findings)
        if (f.code == "DFM-BLK-MIN-W") { found = true; break; }
    EXPECT_TRUE(found);
}

// ── 3. Signature records kind + blank area & shape ─────────────────────
TEST(SkillBlanking, SignatureRecordsKind)
{
    auto stock = skill::createCuboidStock(100.0, 80.0, 1.5);

    skill::blanking::Input in;
    in.shape        = skill::blanking::Shape::Circle;
    in.center_x_mm  = 50.0;
    in.center_y_mm  = 40.0;
    in.diameter_mm  = 20.0;

    auto out = skill::blanking::apply(*stock, in);
    EXPECT_EQ(out.signature.skill_id, std::string("blanking"));
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(), std::string("blanking"));
    EXPECT_EQ(out.signature.pattern["shape"].get<std::string>(), std::string("circle"));
    EXPECT_TRUE(out.signature.pattern["is_blanked"].get<bool>());
    EXPECT_EQ(out.signature.pattern["operation"].get<std::string>(),
              std::string("intersection"));
    EXPECT_EQ(out.signature.tooling.tool_type, std::string("blanking_die"));

    const double expected = M_PI * 10.0 * 10.0;
    EXPECT_NEAR(out.signature.pattern["blank_area_mm2"].get<double>(),
                expected, expected * 0.01);
}

// ── 4. Recognize replays the blank from metadata ──────────────────────
TEST(SkillBlanking, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(100.0, 80.0, 1.5);

    skill::blanking::Input in;
    in.shape        = skill::blanking::Shape::Rectangle;
    in.center_x_mm  = 60.0;
    in.center_y_mm  = 30.0;
    in.length_mm    = 25.0;
    in.width_mm     = 15.0;

    auto out   = skill::blanking::apply(*stock, in);
    auto cands = skill::blanking::recognize(*out.workpiece);

    ASSERT_EQ(cands.size(), 1u);
    const auto& c = cands.front();
    EXPECT_EQ(c.skill_id, std::string("blanking"));
    EXPECT_NEAR(c.confidence, 1.0, 1e-6);
    EXPECT_EQ(c.recovered_params["shape"].get<std::string>(), std::string("rectangle"));
    EXPECT_NEAR(c.recovered_params["dims"]["length_mm"].get<double>(), 25.0, 1e-3);

    // Stripped of metadata → recognize empty.
    skill::Workpiece raw(out.workpiece->shape());
    auto cands2 = skill::blanking::recognize(raw);
    EXPECT_TRUE(cands2.empty());
}

// ── 5. Specific: intersection (not subtraction) - result equals blank ──
TEST(SkillBlanking, IsIntersectionNotSubtraction)
{
    // Verify that blanking RETAINS the blank and DISCARDS everything else
    // (the inverse of a regular cut).  We check that the result bbox is
    // smaller than the original sheet bbox.
    auto stock = skill::createCuboidStock(100.0, 80.0, 1.5);

    skill::blanking::Input in;
    in.shape        = skill::blanking::Shape::Circle;
    in.center_x_mm  = 50.0;
    in.center_y_mm  = 40.0;
    in.diameter_mm  = 20.0;

    auto out = skill::blanking::apply(*stock, in);
    double xMin, yMin, zMin, xMax, yMax, zMax;
    out.workpiece->boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);

    // The blank's XY bbox must equal the circle's bbox (within tol), NOT
    // the original 100 × 80 sheet bbox.
    EXPECT_NEAR(xMax - xMin, 20.0, 0.1);
    EXPECT_NEAR(yMax - yMin, 20.0, 0.1);
    EXPECT_NEAR(zMax - zMin, 1.5, 1e-3);
}
