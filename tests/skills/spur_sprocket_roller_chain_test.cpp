// @lat: [[process/test-strategy#spur_sprocket_roller_chain]]

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/spur_sprocket_roller_chain.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

using namespace koocadcam;

namespace {
double volumeOf(const TopoDS_Shape& s) {
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}

skill::spur_sprocket_roller_chain::Input goodInput()
{
    skill::spur_sprocket_roller_chain::Input in;
    in.face_id            = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.center_xy          = gp_Pnt(0.0, 0.0, 0.0);
    in.chain_pitch_mm     = 12.70;   // ANSI #40
    in.roller_dia_mm      = 7.92;
    in.tooth_count        = 17;
    in.blank_outer_dia_mm = 80.0;
    in.face_width_mm      = 8.0;
    return in;
}
}  // namespace

TEST(SkillSpurSprocketRollerChain, ApplyRemovesMaterial)
{
    auto stock = skill::createCylindricalStock(80.0, 8.0);
    const double v0 = volumeOf(stock->shape());
    auto in  = goodInput();
    auto out = skill::spur_sprocket_roller_chain::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    const double v1 = volumeOf(out.workpiece->shape());

    EXPECT_GT(v0 - v1, 0.0);   // material removed (N roller seats)
    EXPECT_GT(out.workpiece->faceCount(), stock->faceCount());
}

TEST(SkillSpurSprocketRollerChain, ValidateRejectsTooFewTeeth)
{
    auto stock = skill::createCylindricalStock(80.0, 8.0);
    auto in = goodInput();
    in.tooth_count = 7;

    auto r = skill::spur_sprocket_roller_chain::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-PT-TEETH") { found = true; break; }
    EXPECT_TRUE(found);

    EXPECT_THROW(skill::spur_sprocket_roller_chain::apply(*stock, in),
                 skill::SkillError);
}

TEST(SkillSpurSprocketRollerChain, ValidateRejectsRollerTooLarge)
{
    auto stock = skill::createCylindricalStock(80.0, 8.0);
    auto in = goodInput();
    in.roller_dia_mm = 14.0;   // >= chain_pitch_mm

    auto r = skill::spur_sprocket_roller_chain::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-PT-ROLLER") { found = true; break; }
    EXPECT_TRUE(found);
}

TEST(SkillSpurSprocketRollerChain, SignatureCompoundSprocket)
{
    auto stock = skill::createCylindricalStock(80.0, 8.0);
    auto in    = goodInput();
    auto out   = skill::spur_sprocket_roller_chain::apply(*stock, in);

    EXPECT_EQ(out.signature.skill_id,
              std::string("spur_sprocket_roller_chain"));
    EXPECT_TRUE(out.signature.pattern.value("is_compound", false));
    EXPECT_EQ(out.signature.pattern.value("powertrans_feature_type", std::string()),
              std::string("roller_chain_sprocket_teeth"));
    EXPECT_EQ(out.signature.pattern.value("subfeature_count", 0), 17);
}

TEST(SkillSpurSprocketRollerChain, RecognizeMetadataReplay)
{
    auto stock = skill::createCylindricalStock(80.0, 8.0);
    auto in    = goodInput();
    auto out   = skill::spur_sprocket_roller_chain::apply(*stock, in);
    auto cands = skill::spur_sprocket_roller_chain::recognize(*out.workpiece);
    ASSERT_FALSE(cands.empty());
    EXPECT_DOUBLE_EQ(cands[0].confidence, 1.0);
}
