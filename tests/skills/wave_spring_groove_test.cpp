// @lat: [[process/test-strategy#compound feature]]
//
// wave_spring_groove — 2 sub-feature chain: main slot + bottom relief.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/wave_spring_groove.hpp"

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

// ─── 1. apply produces a valid shape with multiple sub-features ───────────
TEST(SkillWaveSpringGroove, ApplyProducesValidShapeWithMultipleSubFeatures)
{
    auto stock = skill::createCylindricalStock(40.0, 30.0);

    skill::wave_spring_groove::Input in;
    in.bore_axis          = gp_Ax1(gp_Pnt(0.0, 0.0, 0.0), gp_Dir(0.0, 0.0, 1.0));
    in.mean_dia           = 30.0;
    in.wave_spring_id     = 28.0;
    in.wave_spring_height = 2.0;
    in.axial_position_mm  = 15.0;

    const double v0 = volumeOf(stock->shape());
    const int    n0 = stock->faceCount();
    auto out = skill::wave_spring_groove::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    const double v1 = volumeOf(out.workpiece->shape());

    // Volume removed is positive (material was cut).
    EXPECT_GT(v0 - v1, 0.0);

    // Face count must change (slot creates new walls).
    EXPECT_NE(out.workpiece->faceCount(), n0);
}

// ─── 2. validate rejects bad input (depth too large) ──────────────────────
TEST(SkillWaveSpringGroove, ValidateRejectsBadInput)
{
    auto stock = skill::createCylindricalStock(40.0, 30.0);

    skill::wave_spring_groove::Input in;
    in.bore_axis          = gp_Ax1(gp_Pnt(0.0, 0.0, 0.0), gp_Dir(0.0, 0.0, 1.0));
    in.mean_dia           = 30.0;
    in.wave_spring_id     = 20.0;     // depth = 5 mm > 0.05 × 30 mm = 1.5 mm
    in.wave_spring_height = 2.0;
    in.axial_position_mm  = 15.0;

    auto r = skill::wave_spring_groove::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool foundDepth = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-WAVE-DEPTH") { foundDepth = true; break; }
    EXPECT_TRUE(foundDepth);

    EXPECT_THROW(skill::wave_spring_groove::apply(*stock, in),
                 skill::SkillError);
}

// ─── 3. signature records is_compound + subfeature_count >= 2 ────────────
TEST(SkillWaveSpringGroove, SignatureRecordsCompoundKind)
{
    auto stock = skill::createCylindricalStock(40.0, 30.0);

    skill::wave_spring_groove::Input in;
    in.bore_axis          = gp_Ax1(gp_Pnt(0.0, 0.0, 0.0), gp_Dir(0.0, 0.0, 1.0));
    in.mean_dia           = 30.0;
    in.wave_spring_id     = 28.5;
    in.wave_spring_height = 1.8;
    in.axial_position_mm  = 12.0;

    auto out = skill::wave_spring_groove::apply(*stock, in);

    EXPECT_EQ(out.signature.skill_id, std::string("wave_spring_groove"));
    EXPECT_EQ(out.signature.pattern.at("kind"), std::string("wave_spring_groove"));
    EXPECT_TRUE(out.signature.pattern.at("is_compound").get<bool>());
    EXPECT_GE(out.signature.pattern.at("subfeature_count").get<int>(), 2);
    EXPECT_NEAR(out.signature.params.at("mean_dia").get<double>(), 30.0, 1e-9);
    EXPECT_NEAR(out.signature.params.at("wave_spring_id").get<double>(),
                28.5, 1e-9);
}

// ─── 4. recognize() replays metadata at confidence 1.0 ───────────────────
TEST(SkillWaveSpringGroove, RecognizeMetadataReplay)
{
    auto stock = skill::createCylindricalStock(40.0, 30.0);

    skill::wave_spring_groove::Input in;
    in.bore_axis          = gp_Ax1(gp_Pnt(0.0, 0.0, 0.0), gp_Dir(0.0, 0.0, 1.0));
    in.mean_dia           = 32.0;
    in.wave_spring_id     = 30.5;
    in.wave_spring_height = 1.5;
    in.axial_position_mm  = 10.0;

    auto out = skill::wave_spring_groove::apply(*stock, in);
    auto cands = skill::wave_spring_groove::recognize(*out.workpiece);

    ASSERT_EQ(cands.size(), 1u);
    EXPECT_EQ(cands[0].skill_id, std::string("wave_spring_groove"));
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-9);
    EXPECT_NEAR(cands[0].recovered_params.at("mean_dia").get<double>(),
                32.0, 1e-9);
    EXPECT_NEAR(cands[0].recovered_params.at("wave_spring_id").get<double>(),
                30.5, 1e-9);
}

// ─── 5. specific: groove depth = (mean_dia − wave_spring_id) / 2 ─────────
TEST(SkillWaveSpringGroove, DepthEqualsDifferenceOverTwo)
{
    auto stock = skill::createCylindricalStock(50.0, 40.0);

    skill::wave_spring_groove::Input in;
    in.bore_axis          = gp_Ax1(gp_Pnt(0.0, 0.0, 0.0), gp_Dir(0.0, 0.0, 1.0));
    in.mean_dia           = 40.0;
    in.wave_spring_id     = 38.0;
    in.wave_spring_height = 2.0;
    in.axial_position_mm  = 20.0;

    auto out = skill::wave_spring_groove::apply(*stock, in);
    const double depth = out.signature.pattern.at("groove_depth_mm").get<double>();
    EXPECT_NEAR(depth, (40.0 - 38.0) / 2.0, 1e-9);

    // Width = height × 1.05
    const double width = out.signature.pattern.at("groove_width_mm").get<double>();
    EXPECT_NEAR(width, 2.0 * 1.05, 1e-9);
}
