// @lat: [[process/test-strategy#compound macro]]
//
// sprocket_blank_with_bore — compound macro: bore + N lightening drills
// + top/bottom face chamfers.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/sprocket_blank_with_bore.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

#include <cmath>

using namespace koocadcam;

namespace {
double volumeOf(const TopoDS_Shape& s) {
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}

skill::sprocket_blank_with_bore::Input goodInput()
{
    skill::sprocket_blank_with_bore::Input in;
    in.outer_dia_mm     = 100.0;
    in.bore_dia_mm      = 25.0;
    in.thickness_mm     = 12.0;
    in.lightening_count = 4;
    in.chamfer_size_mm  = 0.5;
    return in;
}
}  // namespace

TEST(SkillSprocketBlankWithBore, ApplyProducesValidShapeWithMultipleSubFeatures)
{
    auto stock = skill::createCylindricalStock(100.0, 12.0);
    const int faces0 = stock->faceCount();

    auto in  = goodInput();
    auto out = skill::sprocket_blank_with_bore::apply(*stock, in);

    ASSERT_FALSE(out.workpiece->shape().IsNull());
    EXPECT_LT(volumeOf(out.workpiece->shape()), volumeOf(stock->shape()));
    EXPECT_GT(out.workpiece->faceCount(), faces0);
}

TEST(SkillSprocketBlankWithBore, ValidateRejectsBadInput)
{
    auto stock = skill::createCylindricalStock(100.0, 12.0);

    // Hub wall too thin.
    auto in = goodInput();
    in.bore_dia_mm = 98.0;   // wall = 1 mm < 3 mm
    auto rep = skill::sprocket_blank_with_bore::validate(*stock, in);
    EXPECT_FALSE(rep.passed);
    EXPECT_THROW(skill::sprocket_blank_with_bore::apply(*stock, in),
                 skill::SkillError);
}

TEST(SkillSprocketBlankWithBore, SignatureRecordsCompoundKind)
{
    auto stock = skill::createCylindricalStock(100.0, 12.0);
    auto in    = goodInput();
    auto out   = skill::sprocket_blank_with_bore::apply(*stock, in);

    EXPECT_EQ(out.signature.skill_id, std::string("sprocket_blank_with_bore"));
    EXPECT_TRUE(out.signature.pattern.at("is_compound").get<bool>());
    EXPECT_GE(out.signature.pattern.at("subfeature_count").get<int>(), 2);
    // 1 bore + 4 lightening + 2 chamfer = 7
    EXPECT_EQ(out.signature.pattern.at("subfeature_count").get<int>(), 7);
}

TEST(SkillSprocketBlankWithBore, RecognizeMetadataReplay)
{
    auto stock = skill::createCylindricalStock(100.0, 12.0);
    auto in    = goodInput();
    auto out   = skill::sprocket_blank_with_bore::apply(*stock, in);

    auto cands = skill::sprocket_blank_with_bore::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-9);
    EXPECT_EQ(cands[0].recovered_params.at("lightening_count").get<int>(), 4);
}

// Specific assertion: hub wall = (outer - bore) / 2.
TEST(SkillSprocketBlankWithBore, HubWallMatchesDimensionLaw)
{
    auto stock = skill::createCylindricalStock(100.0, 12.0);
    auto in    = goodInput();
    auto out   = skill::sprocket_blank_with_bore::apply(*stock, in);

    const double expected = (in.outer_dia_mm - in.bore_dia_mm) / 2.0;
    const double recorded = out.signature.pattern.at("wall_thickness_mm").get<double>();
    EXPECT_NEAR(recorded, expected, 1e-6);
    EXPECT_GE(recorded, 3.0);   // matches DFM gate
}
