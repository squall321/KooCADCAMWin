// @lat: [[process/test-strategy#pitch_bearing_bolt_circle]]

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/pitch_bearing_bolt_circle.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

using namespace koocadcam;

namespace {
double volumeOf(const TopoDS_Shape& s) {
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}

skill::pitch_bearing_bolt_circle::Input goodInput()
{
    skill::pitch_bearing_bolt_circle::Input in;
    in.face_id            = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.center_xy          = gp_Pnt(0.0, 0.0, 0.0);
    in.bolt_circle_dia_mm = 700.0;
    in.bolt_count         = 24;
    in.bolt_dia_mm        = 27.0;
    in.center_bore_dia_mm = 400.0;
    in.bore_depth_mm      = 40.0;
    return in;
}
}  // namespace

TEST(SkillPitchBearingBoltCircle, ApplyRemovesMaterial)
{
    auto stock = skill::createCylindricalStock(820.0, 80.0);
    const double v0 = volumeOf(stock->shape());
    auto in  = goodInput();
    auto out = skill::pitch_bearing_bolt_circle::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    const double v1 = volumeOf(out.workpiece->shape());

    EXPECT_GT(v0 - v1, 0.0);   // center bore + bolt circle removed
    EXPECT_GT(out.workpiece->faceCount(), stock->faceCount());
}

TEST(SkillPitchBearingBoltCircle, ValidateRejectsBoltCount)
{
    auto stock = skill::createCylindricalStock(820.0, 80.0);
    auto in = goodInput();
    in.bolt_count = 8;   // below [12, 120] band

    auto r = skill::pitch_bearing_bolt_circle::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-BOLT-COUNT") { found = true; break; }
    EXPECT_TRUE(found);

    EXPECT_THROW(skill::pitch_bearing_bolt_circle::apply(*stock, in),
                 skill::SkillError);
}

TEST(SkillPitchBearingBoltCircle, ValidateRejectsTightPcd)
{
    auto stock = skill::createCylindricalStock(820.0, 80.0);
    auto in = goodInput();
    in.bolt_circle_dia_mm = 410.0;   // does not clear 400 bore + bolts

    auto r = skill::pitch_bearing_bolt_circle::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-PCD") { found = true; break; }
    EXPECT_TRUE(found);
}

TEST(SkillPitchBearingBoltCircle, SignatureCompound)
{
    auto stock = skill::createCylindricalStock(820.0, 80.0);
    auto in    = goodInput();
    auto out   = skill::pitch_bearing_bolt_circle::apply(*stock, in);

    EXPECT_EQ(out.signature.skill_id,
              std::string("pitch_bearing_bolt_circle"));
    EXPECT_TRUE(out.signature.pattern.value("is_compound", false));
    EXPECT_EQ(out.signature.pattern.value("wind_feature_type", std::string()),
              std::string("pitch_bearing_bolt_circle"));
    EXPECT_EQ(out.signature.pattern.value("subfeature_count", 0),
              in.bolt_count + 1);
}

TEST(SkillPitchBearingBoltCircle, RecognizeMetadataReplay)
{
    auto stock = skill::createCylindricalStock(820.0, 80.0);
    auto in    = goodInput();
    auto out   = skill::pitch_bearing_bolt_circle::apply(*stock, in);
    auto cands = skill::pitch_bearing_bolt_circle::recognize(*out.workpiece);
    ASSERT_FALSE(cands.empty());
    EXPECT_DOUBLE_EQ(cands[0].confidence, 1.0);
}
