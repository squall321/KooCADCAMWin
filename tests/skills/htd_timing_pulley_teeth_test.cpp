// @lat: [[process/test-strategy#htd_timing_pulley_teeth]]

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/htd_timing_pulley_teeth.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

using namespace koocadcam;

namespace {
double volumeOf(const TopoDS_Shape& s) {
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}

skill::htd_timing_pulley_teeth::Input goodInput()
{
    skill::htd_timing_pulley_teeth::Input in;
    in.face_id            = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.center_xy          = gp_Pnt(0.0, 0.0, 0.0);
    in.htd_pitch_mm       = 5.0;
    in.tooth_count        = 24;
    in.belt_width_mm      = 15.0;
    in.tooth_depth_mm     = 2.0;
    in.blank_outer_dia_mm = 44.0;  // PCD = 5*24/pi ≈ 38.2 < 44
    return in;
}
}  // namespace

TEST(SkillHtdTimingPulleyTeeth, ApplyRemovesMaterial)
{
    auto stock = skill::createCylindricalStock(44.0, 15.0);
    const double v0 = volumeOf(stock->shape());
    auto in  = goodInput();
    auto out = skill::htd_timing_pulley_teeth::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    const double v1 = volumeOf(out.workpiece->shape());

    EXPECT_GT(v0 - v1, 0.0);   // material removed (N tooth gaps)
    EXPECT_GT(out.workpiece->faceCount(), stock->faceCount());
}

TEST(SkillHtdTimingPulleyTeeth, ValidateRejectsBadPitch)
{
    auto stock = skill::createCylindricalStock(44.0, 15.0);
    auto in = goodInput();
    in.htd_pitch_mm = 4.0;   // not in {3,5,8}

    auto r = skill::htd_timing_pulley_teeth::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-PT-PITCH") { found = true; break; }
    EXPECT_TRUE(found);

    EXPECT_THROW(skill::htd_timing_pulley_teeth::apply(*stock, in),
                 skill::SkillError);
}

TEST(SkillHtdTimingPulleyTeeth, ValidateRejectsTooFewTeeth)
{
    auto stock = skill::createCylindricalStock(44.0, 15.0);
    auto in = goodInput();
    in.tooth_count = 8;

    auto r = skill::htd_timing_pulley_teeth::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-PT-TEETH") { found = true; break; }
    EXPECT_TRUE(found);
}

TEST(SkillHtdTimingPulleyTeeth, SignatureCompoundTimingPulley)
{
    auto stock = skill::createCylindricalStock(44.0, 15.0);
    auto in    = goodInput();
    auto out   = skill::htd_timing_pulley_teeth::apply(*stock, in);

    EXPECT_EQ(out.signature.skill_id, std::string("htd_timing_pulley_teeth"));
    EXPECT_TRUE(out.signature.pattern.value("is_compound", false));
    EXPECT_EQ(out.signature.pattern.value("powertrans_feature_type", std::string()),
              std::string("htd_timing_pulley_teeth"));
    EXPECT_EQ(out.signature.pattern.value("subfeature_count", 0), 24);
}

TEST(SkillHtdTimingPulleyTeeth, RecognizeMetadataReplay)
{
    auto stock = skill::createCylindricalStock(44.0, 15.0);
    auto in    = goodInput();
    auto out   = skill::htd_timing_pulley_teeth::apply(*stock, in);
    auto cands = skill::htd_timing_pulley_teeth::recognize(*out.workpiece);
    ASSERT_FALSE(cands.empty());
    EXPECT_DOUBLE_EQ(cands[0].confidence, 1.0);
}
