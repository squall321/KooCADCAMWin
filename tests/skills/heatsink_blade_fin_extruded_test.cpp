// @lat: [[process/test-strategy#heatsink_blade_fin_extruded]]

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/heatsink_blade_fin_extruded.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

using namespace koocadcam;

namespace {
double volumeOf(const TopoDS_Shape& s) {
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}
}  // namespace

// ─── 1. apply ADDS blade-fin + base plate volume ────────────────────────
TEST(SkillHeatsinkBladeFin, ApplyAddsBladeFinVolume)
{
    auto stock = skill::createCuboidStock(120.0, 80.0, 5.0);
    const double v0 = volumeOf(stock->shape());

    skill::heatsink_blade_fin_extruded::Input in;
    in.face_id            = 0;
    in.base_thickness_mm  = 0.0;     // no base plate this test
    in.fin_count          = 6;
    in.fin_pitch_mm       = 6.0;
    in.fin_thickness_mm   = 2.0;
    in.fin_height_mm      = 15.0;
    in.fin_length_mm      = 80.0;
    in.origin_x_mm        = 10.0;
    in.origin_y_mm        = 10.0;

    auto out = skill::heatsink_blade_fin_extruded::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    const double v1 = volumeOf(out.workpiece->shape());
    // expected = 6 × 80 × 2 × 15 = 14400 mm³
    const double expected = 6.0 * 80.0 * 2.0 * 15.0;
    const double added    = v1 - v0;
    EXPECT_NEAR(added, expected, expected * 0.05)
        << "expected blade-fin added volume ≈ " << expected << ", got " << added;

    EXPECT_EQ(out.signature.skill_id,
              std::string("heatsink_blade_fin_extruded"));
}

// ─── 2. validate rejects fin_thickness < 0.5 + AR > 12 ──────────────────
TEST(SkillHeatsinkBladeFin, ValidateRejectsBadInputs)
{
    auto stock = skill::createCuboidStock(120.0, 80.0, 5.0);

    {  // thickness too thin
        skill::heatsink_blade_fin_extruded::Input in;
        in.fin_count = 6;       in.fin_pitch_mm = 5.0;
        in.fin_thickness_mm = 0.3;
        in.fin_height_mm = 5.0; in.fin_length_mm = 40.0;
        auto r = skill::heatsink_blade_fin_extruded::validate(*stock, in);
        EXPECT_FALSE(r.passed);
        bool found = false;
        for (const auto& f : r.findings)
            if (f.code == "DFM-FIN-THK") found = true;
        EXPECT_TRUE(found);
    }
    {  // AR too high
        skill::heatsink_blade_fin_extruded::Input in;
        in.fin_count = 4;       in.fin_pitch_mm = 5.0;
        in.fin_thickness_mm = 1.0;
        in.fin_height_mm = 30.0;  // AR = 30
        in.fin_length_mm = 40.0;
        auto r = skill::heatsink_blade_fin_extruded::validate(*stock, in);
        EXPECT_FALSE(r.passed);
        bool found = false;
        for (const auto& f : r.findings)
            if (f.code == "DFM-FIN-AR") found = true;
        EXPECT_TRUE(found);
        EXPECT_THROW(skill::heatsink_blade_fin_extruded::apply(*stock, in),
                     skill::SkillError);
    }
}

// ─── 3. signature carries electronics_feature_type + fin count ───────────
TEST(SkillHeatsinkBladeFin, SignatureCarriesCompound)
{
    auto stock = skill::createCuboidStock(120.0, 80.0, 5.0);
    skill::heatsink_blade_fin_extruded::Input in;
    in.fin_count = 5; in.fin_pitch_mm = 6.0;
    in.fin_thickness_mm = 2.0;
    in.fin_height_mm = 15.0; in.fin_length_mm = 70.0;
    in.origin_x_mm = 5.0; in.origin_y_mm = 10.0;
    auto out = skill::heatsink_blade_fin_extruded::apply(*stock, in);
    EXPECT_TRUE(out.signature.pattern.at("is_compound").get<bool>());
    EXPECT_EQ(out.signature.pattern.at("electronics_feature_type").get<std::string>(),
              std::string("blade_fin_heat_sink"));
    EXPECT_EQ(out.signature.pattern.at("fin_count").get<int>(), 5);
    EXPECT_NEAR(out.signature.pattern.at("fin_pitch_mm").get<double>(), 6.0, 1e-6);
}

// ─── 4. recognize from metadata ──────────────────────────────────────────
TEST(SkillHeatsinkBladeFin, RecognizeFromMetadata)
{
    auto stock = skill::createCuboidStock(120.0, 80.0, 5.0);
    skill::heatsink_blade_fin_extruded::Input in;
    in.fin_count = 4; in.fin_pitch_mm = 6.0;
    in.fin_thickness_mm = 2.0;
    in.fin_height_mm = 12.0; in.fin_length_mm = 60.0;
    in.origin_x_mm = 10.0; in.origin_y_mm = 10.0;
    auto out = skill::heatsink_blade_fin_extruded::apply(*stock, in);

    auto cands = skill::heatsink_blade_fin_extruded::recognize(*out.workpiece);
    ASSERT_GE(cands.size(), 1u);
    EXPECT_GE(cands.front().confidence, 0.9);
    EXPECT_EQ(cands.front().recovered_params.at("fin_count").get<int>(), 4);
}

// ─── 5. base plate option also adds volume ──────────────────────────────
TEST(SkillHeatsinkBladeFin, BasePlateAddsVolume)
{
    auto stock = skill::createCuboidStock(120.0, 80.0, 5.0);
    const double v0 = volumeOf(stock->shape());

    skill::heatsink_blade_fin_extruded::Input in;
    in.base_thickness_mm  = 2.0;
    in.fin_count          = 4;
    in.fin_pitch_mm       = 6.0;
    in.fin_thickness_mm   = 2.0;
    in.fin_height_mm      = 10.0;
    in.fin_length_mm      = 60.0;
    in.origin_x_mm        = 10.0;
    in.origin_y_mm        = 10.0;
    auto out = skill::heatsink_blade_fin_extruded::apply(*stock, in);
    const double v1 = volumeOf(out.workpiece->shape());
    const double spanY = (4 - 1) * 6.0 + 2.0;       // 20 mm
    const double baseV = 60.0 * spanY * 2.0;        // 2400
    const double finsV = 4 * 60.0 * 2.0 * 10.0;     // 4800
    const double expected = baseV + finsV;          // 7200 mm³
    EXPECT_NEAR(v1 - v0, expected, expected * 0.08)
        << "expected " << expected << ", got " << v1 - v0;
}
