// @lat: [[process/test-strategy#compound macro]]
//
// gear_blank_with_hub_step — compound macro: hub-step turn + bore + keyway.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/gear_blank_with_hub_step.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

#include <cmath>

using namespace koocadcam;

namespace {
double volumeOf(const TopoDS_Shape& s) {
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}

skill::gear_blank_with_hub_step::Input goodInput()
{
    skill::gear_blank_with_hub_step::Input in;
    in.blank_dia_mm   = 80.0;
    in.blank_thick_mm = 25.0;
    in.hub_dia_mm     = 40.0;
    in.hub_height_mm  = 10.0;
    in.bore_dia_mm    = 20.0;
    in.keyway_w_mm    = 5.0;
    in.keyway_h_mm    = 2.3;
    return in;
}
}  // namespace

TEST(SkillGearBlankWithHubStep, ApplyProducesValidShapeWithMultipleSubFeatures)
{
    auto stock = skill::createCylindricalStock(80.0, 25.0);
    const int faces0 = stock->faceCount();

    auto in  = goodInput();
    auto out = skill::gear_blank_with_hub_step::apply(*stock, in);

    ASSERT_FALSE(out.workpiece->shape().IsNull());
    EXPECT_LT(volumeOf(out.workpiece->shape()), volumeOf(stock->shape()));
    EXPECT_GT(out.workpiece->faceCount(), faces0);
}

TEST(SkillGearBlankWithHubStep, ValidateRejectsBadInput)
{
    auto stock = skill::createCylindricalStock(80.0, 25.0);

    // Hub diameter exceeds blank → no step.
    auto in = goodInput();
    in.hub_dia_mm = 90.0;
    auto rep = skill::gear_blank_with_hub_step::validate(*stock, in);
    EXPECT_FALSE(rep.passed);
    bool found = false;
    for (const auto& f : rep.findings) if (f.code == "DFM-GEAR-HUB-DIA") found = true;
    EXPECT_TRUE(found);
    EXPECT_THROW(skill::gear_blank_with_hub_step::apply(*stock, in),
                 skill::SkillError);
}

TEST(SkillGearBlankWithHubStep, SignatureRecordsCompoundKind)
{
    auto stock = skill::createCylindricalStock(80.0, 25.0);
    auto in    = goodInput();
    auto out   = skill::gear_blank_with_hub_step::apply(*stock, in);

    EXPECT_EQ(out.signature.skill_id, std::string("gear_blank_with_hub_step"));
    EXPECT_TRUE(out.signature.pattern.at("is_compound").get<bool>());
    EXPECT_GE(out.signature.pattern.at("subfeature_count").get<int>(), 2);
    // step + bore + keyway = 3
    EXPECT_EQ(out.signature.pattern.at("subfeature_count").get<int>(), 3);
    EXPECT_TRUE(out.signature.pattern.at("has_keyway").get<bool>());
}

TEST(SkillGearBlankWithHubStep, RecognizeMetadataReplay)
{
    auto stock = skill::createCylindricalStock(80.0, 25.0);
    auto in    = goodInput();
    auto out   = skill::gear_blank_with_hub_step::apply(*stock, in);

    auto cands = skill::gear_blank_with_hub_step::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-9);
    EXPECT_NEAR(cands[0].recovered_params.at("hub_dia_mm").get<double>(),
                40.0, 1e-6);
}

// Specific assertion — hub_wall = (hub_dia - bore_dia) / 2; must be >= 3 mm.
TEST(SkillGearBlankWithHubStep, HubWallMatchesDimensionLaw)
{
    auto stock = skill::createCylindricalStock(80.0, 25.0);
    auto in    = goodInput();
    auto out   = skill::gear_blank_with_hub_step::apply(*stock, in);

    const double expected = (in.hub_dia_mm - in.bore_dia_mm) / 2.0;
    const double recorded = out.signature.pattern.at("hub_wall_mm").get<double>();
    EXPECT_NEAR(recorded, expected, 1e-6);
    EXPECT_GE(recorded, 3.0);
}
