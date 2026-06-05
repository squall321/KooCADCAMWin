// @lat: [[process/test-strategy#brake_disc_hub_bolt_circle]]

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/brake_disc_hub_bolt_circle.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

using namespace koocadcam;

namespace {
double volumeOf(const TopoDS_Shape& s) {
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}

skill::brake_disc_hub_bolt_circle::Input goodInput()
{
    skill::brake_disc_hub_bolt_circle::Input in;
    in.face_id            = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.center_xy          = gp_Pnt(0.0, 0.0, 0.0);
    in.hub_bore_dia_mm    = 80.0;
    in.bolt_circle_dia_mm = 160.0;
    in.bolt_count         = 6;
    in.bolt_dia_mm        = 14.0;
    in.vent_slot_count    = 8;
    in.vent_slot_width_mm = 8.0;
    return in;
}
}  // namespace

TEST(SkillBrakeDiscHubBoltCircle, ApplyRemovesMaterial)
{
    auto stock = skill::createCylindricalStock(300.0, 25.0);
    const double v0 = volumeOf(stock->shape());
    auto in  = goodInput();
    auto out = skill::brake_disc_hub_bolt_circle::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    const double v1 = volumeOf(out.workpiece->shape());

    EXPECT_GT(v0 - v1, 0.0);   // material removed (bore + bolts + vent slots)
    EXPECT_GT(out.workpiece->faceCount(), stock->faceCount());
}

TEST(SkillBrakeDiscHubBoltCircle, ValidateRejectsBoltCount)
{
    auto stock = skill::createCylindricalStock(300.0, 25.0);
    auto in = goodInput();
    in.bolt_count = 2;   // outside [4, 12]

    auto r = skill::brake_disc_hub_bolt_circle::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-COUNT") { found = true; break; }
    EXPECT_TRUE(found);

    EXPECT_THROW(skill::brake_disc_hub_bolt_circle::apply(*stock, in),
                 skill::SkillError);
}

TEST(SkillBrakeDiscHubBoltCircle, ValidateRejectsPcdInsideBore)
{
    auto stock = skill::createCylindricalStock(300.0, 25.0);
    auto in = goodInput();
    in.bolt_circle_dia_mm = 50.0;   // <= hub_bore_dia_mm

    auto r = skill::brake_disc_hub_bolt_circle::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-PCD") { found = true; break; }
    EXPECT_TRUE(found);
}

TEST(SkillBrakeDiscHubBoltCircle, SignatureCompoundBrakeDisc)
{
    auto stock = skill::createCylindricalStock(300.0, 25.0);
    auto in    = goodInput();
    auto out   = skill::brake_disc_hub_bolt_circle::apply(*stock, in);

    EXPECT_EQ(out.signature.skill_id, std::string("brake_disc_hub_bolt_circle"));
    EXPECT_TRUE(out.signature.pattern.value("is_compound", false));
    EXPECT_EQ(out.signature.pattern.value("railway_feature_type", std::string()),
              std::string("brake_disc_hub_bolt_circle"));
    EXPECT_EQ(out.signature.pattern.value("subfeature_count", 0),
              1 + in.bolt_count + in.vent_slot_count);
}

TEST(SkillBrakeDiscHubBoltCircle, RecognizeMetadataReplay)
{
    auto stock = skill::createCylindricalStock(300.0, 25.0);
    auto in    = goodInput();
    auto out   = skill::brake_disc_hub_bolt_circle::apply(*stock, in);
    auto cands = skill::brake_disc_hub_bolt_circle::recognize(*out.workpiece);
    ASSERT_FALSE(cands.empty());
    EXPECT_DOUBLE_EQ(cands[0].confidence, 1.0);
}
