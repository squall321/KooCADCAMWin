// @lat: [[process/test-strategy#radiator_finger_array]]

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/radiator_finger_array.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

using namespace koocadcam;

namespace {
double volumeOf(const TopoDS_Shape& s) {
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}

skill::radiator_finger_array::Input goodInput()
{
    skill::radiator_finger_array::Input in;
    in.face_id          = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.fin_count        = 8;
    in.fin_pitch_mm     = 3.0;
    in.fin_thickness_mm = 0.5;
    in.fin_length_mm    = 30.0;
    in.fin_height_mm    = 25.0;
    in.tube_dia_mm      = 6.0;
    return in;
}
}  // namespace

TEST(SkillRadiatorFingerArray, ApplyRemovesMaterial)
{
    auto stock = skill::createCuboidStock(40.0, 30.0, 30.0);
    const double v0 = volumeOf(stock->shape());
    auto in  = goodInput();
    auto out = skill::radiator_finger_array::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    const double v1 = volumeOf(out.workpiece->shape());

    EXPECT_GT(v0 - v1, 0.0);   // material removed (tubes + air gaps)
    EXPECT_GT(out.workpiece->faceCount(), stock->faceCount());
}

TEST(SkillRadiatorFingerArray, ValidateRejectsThinFin)
{
    auto stock = skill::createCuboidStock(40.0, 30.0, 30.0);
    auto in = goodInput();
    in.fin_thickness_mm = 0.10;   // < 0.15 mm floor

    auto r = skill::radiator_finger_array::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-FIN-THK") { found = true; break; }
    EXPECT_TRUE(found);

    EXPECT_THROW(skill::radiator_finger_array::apply(*stock, in), skill::SkillError);
}

TEST(SkillRadiatorFingerArray, SignatureCompoundHvacType)
{
    auto stock = skill::createCuboidStock(40.0, 30.0, 30.0);
    auto in    = goodInput();
    auto out   = skill::radiator_finger_array::apply(*stock, in);

    EXPECT_EQ(out.signature.skill_id, std::string("radiator_finger_array"));
    EXPECT_TRUE(out.signature.pattern.value("is_compound", false));
    EXPECT_EQ(out.signature.pattern.value("hvac_feature_type", std::string()),
              std::string("radiator_finger_array"));
    EXPECT_EQ(out.signature.pattern.value("fin_count", 0), in.fin_count);
    EXPECT_EQ(out.signature.pattern.value("tube_count", 0), 2);
}

TEST(SkillRadiatorFingerArray, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(40.0, 30.0, 30.0);
    auto in    = goodInput();
    auto out   = skill::radiator_finger_array::apply(*stock, in);
    auto cands = skill::radiator_finger_array::recognize(*out.workpiece);
    ASSERT_FALSE(cands.empty());
    EXPECT_DOUBLE_EQ(cands[0].confidence, 1.0);
}
