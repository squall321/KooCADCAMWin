// @lat: [[process/test-strategy#compound macro]]
//
// pulley_with_keyway_compound — compound macro: bore + V-groove + keyway
// + 2 balance drills.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/pulley_with_keyway_compound.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

#include <cmath>

using namespace koocadcam;

namespace {
double volumeOf(const TopoDS_Shape& s) {
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}

skill::pulley_with_keyway_compound::Input goodInput()
{
    skill::pulley_with_keyway_compound::Input in;
    in.outer_dia_mm        = 80.0;
    in.groove_angle_deg    = 38.0;
    in.groove_depth_mm     = 4.0;
    in.thickness_mm        = 20.0;
    in.key_w_mm            = 6.0;
    in.key_h_mm            = 2.8;     // standard for Ø25 bore + key 6
    in.bore_dia_mm         = 25.0;
    in.balance_drill_dia_mm = 5.0;
    return in;
}
}  // namespace

TEST(SkillPulleyWithKeywayCompound, ApplyProducesValidShapeWithMultipleSubFeatures)
{
    auto stock = skill::createCylindricalStock(80.0, 20.0);
    const int faces0 = stock->faceCount();

    auto in  = goodInput();
    auto out = skill::pulley_with_keyway_compound::apply(*stock, in);

    ASSERT_FALSE(out.workpiece->shape().IsNull());

    // Volume must decrease (5 sub-features cut material).
    const double v0 = volumeOf(stock->shape());
    const double v1 = volumeOf(out.workpiece->shape());
    EXPECT_LT(v1, v0);

    // Face count must increase consistent with multiple sub-features (bore
    // alone adds at least 1 cyl face + edges; full chain adds several more).
    EXPECT_GT(out.workpiece->faceCount(), faces0);
}

TEST(SkillPulleyWithKeywayCompound, ValidateRejectsBadInput)
{
    auto stock = skill::createCylindricalStock(80.0, 20.0);

    // Bore too small.
    auto in  = goodInput();
    in.bore_dia_mm = 1.0;
    auto rep = skill::pulley_with_keyway_compound::validate(*stock, in);
    EXPECT_FALSE(rep.passed);
    EXPECT_THROW(skill::pulley_with_keyway_compound::apply(*stock, in),
                 skill::SkillError);
}

TEST(SkillPulleyWithKeywayCompound, SignatureRecordsCompoundKind)
{
    auto stock = skill::createCylindricalStock(80.0, 20.0);
    auto in    = goodInput();
    auto out   = skill::pulley_with_keyway_compound::apply(*stock, in);

    EXPECT_EQ(out.signature.skill_id, std::string("pulley_with_keyway_compound"));
    EXPECT_EQ(out.signature.pattern.at("kind").get<std::string>(),
              std::string("pulley_with_keyway_compound"));
    EXPECT_TRUE(out.signature.pattern.at("is_compound").get<bool>());
    EXPECT_GE(out.signature.pattern.at("subfeature_count").get<int>(), 2);
    EXPECT_EQ(out.signature.pattern.at("subfeature_count").get<int>(), 5);
}

TEST(SkillPulleyWithKeywayCompound, RecognizeMetadataReplay)
{
    auto stock = skill::createCylindricalStock(80.0, 20.0);
    auto in    = goodInput();
    auto out   = skill::pulley_with_keyway_compound::apply(*stock, in);

    auto cands = skill::pulley_with_keyway_compound::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-9);
    EXPECT_NEAR(cands[0].recovered_params.at("outer_dia_mm").get<double>(),
                80.0, 1e-6);
    EXPECT_NEAR(cands[0].recovered_params.at("bore_dia_mm").get<double>(),
                25.0, 1e-6);
}

// Specific assertion — groove width = 2 · depth · tan(angle/2)
// (compound-skill identity: derived param must match the law).
TEST(SkillPulleyWithKeywayCompound, GrooveWidthMatchesAngleLaw)
{
    auto stock = skill::createCylindricalStock(80.0, 20.0);
    auto in    = goodInput();
    auto out   = skill::pulley_with_keyway_compound::apply(*stock, in);

    const double angRad = in.groove_angle_deg * M_PI / 180.0;
    const double expected = 2.0 * in.groove_depth_mm * std::tan(angRad / 2.0);
    const double recorded = out.signature.pattern.at("groove_width_mm").get<double>();
    EXPECT_NEAR(recorded, expected, 1e-6);
}
