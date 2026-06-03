// @lat: [[process/test-strategy#rack_involute]]
//
// rack_involute — REAL multi-cut linear rack with trapezoidal flank gaps.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/rack_involute.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

#include <cmath>

using namespace koocadcam;

namespace {

double volumeOf(const TopoDS_Shape& s)
{
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}

skill::rack_involute::Input goodInput()
{
    // m=2, tooth_count=10, pitch = π·m ≈ 6.28
    // rack length required ≈ 10 · 6.28 = 62.8 mm
    skill::rack_involute::Input in;
    in.module_mm          = 2.0;
    in.tooth_count        = 10;
    in.face_width_mm      = 10.0;
    in.pressure_angle_deg = 20.0;
    in.rack_length_mm     = 80.0;
    return in;
}

}  // namespace

TEST(SkillRackInvolute, ApplyRemovesRealGeometry)
{
    // Bar: 80 mm × 10 mm wide × 5 mm tall.
    auto stock = skill::createCuboidStock(80.0, 10.0, 5.0);
    const double vStock = volumeOf(stock->shape());
    const int    fStock = stock->faceCount();

    auto in  = goodInput();
    auto out = skill::rack_involute::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    const double vRack = volumeOf(out.workpiece->shape());
    EXPECT_LT(vRack, vStock);
    EXPECT_GT(vStock - vRack, 1.0);

    EXPECT_GT(out.workpiece->faceCount(), fStock + in.tooth_count);
}

TEST(SkillRackInvolute, ValidateRejectsShortRack)
{
    auto stock = skill::createCuboidStock(80.0, 10.0, 5.0);

    auto in = goodInput();
    in.rack_length_mm = 10.0;        // too short for 10 teeth
    auto rep = skill::rack_involute::validate(*stock, in);
    EXPECT_FALSE(rep.passed);
}

TEST(SkillRackInvolute, SignatureMarksRack)
{
    auto stock = skill::createCuboidStock(80.0, 10.0, 5.0);
    auto in    = goodInput();
    auto out   = skill::rack_involute::apply(*stock, in);

    EXPECT_EQ(out.signature.skill_id, std::string("rack_involute"));
    EXPECT_TRUE(out.signature.pattern.at("is_gear").get<bool>());
    EXPECT_TRUE(out.signature.pattern.at("is_rack").get<bool>());
    EXPECT_EQ(out.signature.pattern.at("tooth_count").get<int>(), 10);
    EXPECT_NEAR(out.signature.pattern.at("pitch_mm").get<double>(),
                M_PI * 2.0, 1e-9);
}

TEST(SkillRackInvolute, RecognizeReplays)
{
    auto stock = skill::createCuboidStock(80.0, 10.0, 5.0);
    auto in    = goodInput();
    auto out   = skill::rack_involute::apply(*stock, in);

    auto cands = skill::rack_involute::recognize(*out.workpiece);
    ASSERT_FALSE(cands.empty());

    bool found = false;
    for (const auto& c : cands) {
        if (std::abs(c.confidence - 1.0) < 1e-9) { found = true; break; }
    }
    EXPECT_TRUE(found);
}
