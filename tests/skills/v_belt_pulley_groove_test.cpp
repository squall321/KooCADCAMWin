// @lat: [[process/test-strategy#v_belt_pulley_groove]]

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/v_belt_pulley_groove.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

using namespace koocadcam;

namespace {
double volumeOf(const TopoDS_Shape& s) {
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}

skill::v_belt_pulley_groove::Input goodInput()
{
    skill::v_belt_pulley_groove::Input in;
    in.face_id          = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.center_xy        = gp_Pnt(0.0, 0.0, 0.0);
    in.pulley_od_mm     = 120.0;
    in.belt_section     = "A";
    in.groove_count     = 3;
    in.groove_angle_deg = 36.0;
    in.groove_depth_mm  = 11.0;
    return in;
}
}  // namespace

TEST(SkillVBeltPulleyGroove, ApplyRemovesMaterial)
{
    auto stock = skill::createCylindricalStock(120.0, 60.0);
    const double v0 = volumeOf(stock->shape());
    auto in  = goodInput();
    auto out = skill::v_belt_pulley_groove::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    const double v1 = volumeOf(out.workpiece->shape());

    EXPECT_GT(v0 - v1, 0.0);   // material removed (N V-grooves)
    EXPECT_GT(out.workpiece->faceCount(), stock->faceCount());
}

TEST(SkillVBeltPulleyGroove, ValidateRejectsBadSection)
{
    auto stock = skill::createCylindricalStock(120.0, 60.0);
    auto in = goodInput();
    in.belt_section = "ZZZ";

    auto r = skill::v_belt_pulley_groove::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-PT-SECTION") { found = true; break; }
    EXPECT_TRUE(found);

    EXPECT_THROW(skill::v_belt_pulley_groove::apply(*stock, in),
                 skill::SkillError);
}

TEST(SkillVBeltPulleyGroove, ValidateRejectsBadAngle)
{
    auto stock = skill::createCylindricalStock(120.0, 60.0);
    auto in = goodInput();
    in.groove_angle_deg = 60.0;   // outside [34, 38]

    auto r = skill::v_belt_pulley_groove::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-PT-ANGLE") { found = true; break; }
    EXPECT_TRUE(found);
}

TEST(SkillVBeltPulleyGroove, SignatureCompoundSheave)
{
    auto stock = skill::createCylindricalStock(120.0, 60.0);
    auto in    = goodInput();
    auto out   = skill::v_belt_pulley_groove::apply(*stock, in);

    EXPECT_EQ(out.signature.skill_id, std::string("v_belt_pulley_groove"));
    EXPECT_TRUE(out.signature.pattern.value("is_compound", false));
    EXPECT_EQ(out.signature.pattern.value("powertrans_feature_type", std::string()),
              std::string("v_belt_sheave_grooves"));
    EXPECT_EQ(out.signature.pattern.value("subfeature_count", 0), 3);
}

TEST(SkillVBeltPulleyGroove, RecognizeMetadataReplay)
{
    auto stock = skill::createCylindricalStock(120.0, 60.0);
    auto in    = goodInput();
    auto out   = skill::v_belt_pulley_groove::apply(*stock, in);
    auto cands = skill::v_belt_pulley_groove::recognize(*out.workpiece);
    ASSERT_FALSE(cands.empty());
    EXPECT_DOUBLE_EQ(cands[0].confidence, 1.0);
}
