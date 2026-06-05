// @lat: [[process/test-strategy#flange_coupling_bolt_circle]]

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/flange_coupling_bolt_circle.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

using namespace koocadcam;

namespace {
double volumeOf(const TopoDS_Shape& s) {
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}

skill::flange_coupling_bolt_circle::Input goodInput()
{
    skill::flange_coupling_bolt_circle::Input in;
    in.face_id            = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.center_xy          = gp_Pnt(0.0, 0.0, 0.0);
    in.bore_dia_mm        = 25.0;   // in DIN 6885 range (22–30 band)
    in.key_length_mm      = 20.0;
    in.bolt_circle_dia_mm = 80.0;
    in.bolt_count         = 4;
    in.bolt_dia_mm        = 8.0;
    return in;
}
}  // namespace

TEST(SkillFlangeCouplingBoltCircle, ApplyRemovesMaterial)
{
    auto stock = skill::createCylindricalStock(120.0, 25.0);
    const double v0 = volumeOf(stock->shape());
    auto in  = goodInput();
    auto out = skill::flange_coupling_bolt_circle::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    const double v1 = volumeOf(out.workpiece->shape());

    EXPECT_GT(v0 - v1, 0.0);   // bore + keyway + bolts removed
    EXPECT_GT(out.workpiece->faceCount(), stock->faceCount());
}

TEST(SkillFlangeCouplingBoltCircle, ValidateRejectsBoreOutOfKeyTable)
{
    auto stock = skill::createCylindricalStock(120.0, 25.0);
    auto in = goodInput();
    in.bore_dia_mm = 200.0;   // beyond DIN 6885 table

    auto r = skill::flange_coupling_bolt_circle::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-PT-KEY") { found = true; break; }
    EXPECT_TRUE(found);

    EXPECT_THROW(skill::flange_coupling_bolt_circle::apply(*stock, in),
                 skill::SkillError);
}

TEST(SkillFlangeCouplingBoltCircle, ValidateRejectsPcdTooSmall)
{
    auto stock = skill::createCylindricalStock(120.0, 25.0);
    auto in = goodInput();
    in.bolt_circle_dia_mm = 20.0;   // <= bore_dia_mm

    auto r = skill::flange_coupling_bolt_circle::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-PT-PCD") { found = true; break; }
    EXPECT_TRUE(found);
}

TEST(SkillFlangeCouplingBoltCircle, SignatureCompoundFlangeCoupling)
{
    auto stock = skill::createCylindricalStock(120.0, 25.0);
    auto in    = goodInput();
    auto out   = skill::flange_coupling_bolt_circle::apply(*stock, in);

    EXPECT_EQ(out.signature.skill_id,
              std::string("flange_coupling_bolt_circle"));
    EXPECT_TRUE(out.signature.pattern.value("is_compound", false));
    EXPECT_EQ(out.signature.pattern.value("powertrans_feature_type", std::string()),
              std::string("rigid_flange_coupling"));
    EXPECT_EQ(out.signature.pattern.value("subfeature_count", 0), 1 + 1 + 4);
}

TEST(SkillFlangeCouplingBoltCircle, RecognizeMetadataReplay)
{
    auto stock = skill::createCylindricalStock(120.0, 25.0);
    auto in    = goodInput();
    auto out   = skill::flange_coupling_bolt_circle::apply(*stock, in);
    auto cands = skill::flange_coupling_bolt_circle::recognize(*out.workpiece);
    ASSERT_FALSE(cands.empty());
    EXPECT_DOUBLE_EQ(cands[0].confidence, 1.0);
}
