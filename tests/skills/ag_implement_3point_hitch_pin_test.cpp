// @lat: [[process/test-strategy#ag_implement_3point_hitch_pin]]

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/ag_implement_3point_hitch_pin.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

using namespace koocadcam;

namespace {
double volumeOf(const TopoDS_Shape& s) {
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}

skill::ag_implement_3point_hitch_pin::Input goodInput()
{
    skill::ag_implement_3point_hitch_pin::Input in;
    in.face_id              = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.center_xy            = gp_Pnt(40.0, 30.0, 0.0);
    in.hitch_category       = 1;
    in.pin_position         = "top_link";
    in.linch_pin_hole_dia_mm = 6.0;
    return in;
}
}  // namespace

TEST(SkillAgImplement3PointHitchPin, ApplyRemovesMaterial)
{
    auto stock = skill::createCuboidStock(80.0, 60.0, 25.0);
    const double v0 = volumeOf(stock->shape());
    auto in  = goodInput();
    auto out = skill::ag_implement_3point_hitch_pin::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    const double v1 = volumeOf(out.workpiece->shape());

    EXPECT_GT(v0 - v1, 0.0);   // material removed (pin bore + cross hole)
    EXPECT_GT(out.workpiece->faceCount(), stock->faceCount());
}

TEST(SkillAgImplement3PointHitchPin, ValidateRejectsBadCategory)
{
    auto stock = skill::createCuboidStock(80.0, 60.0, 25.0);
    auto in = goodInput();
    in.hitch_category = 5;  // out of {1,2,3}

    auto r = skill::ag_implement_3point_hitch_pin::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-CAT") { found = true; break; }
    EXPECT_TRUE(found);

    EXPECT_THROW(skill::ag_implement_3point_hitch_pin::apply(*stock, in),
                 skill::SkillError);
}

TEST(SkillAgImplement3PointHitchPin, ValidateRejectsBadPosition)
{
    auto stock = skill::createCuboidStock(80.0, 60.0, 25.0);
    auto in = goodInput();
    in.pin_position = "side_link";

    auto r = skill::ag_implement_3point_hitch_pin::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-POSITION") { found = true; break; }
    EXPECT_TRUE(found);
}

TEST(SkillAgImplement3PointHitchPin, SignatureCompoundCat1Pin)
{
    auto stock = skill::createCuboidStock(80.0, 60.0, 25.0);
    auto in    = goodInput();
    auto out   = skill::ag_implement_3point_hitch_pin::apply(*stock, in);

    EXPECT_EQ(out.signature.skill_id,
              std::string("ag_implement_3point_hitch_pin"));
    EXPECT_TRUE(out.signature.pattern.value("is_compound", false));
    EXPECT_EQ(out.signature.pattern.value("ag_feature_type", std::string()),
              std::string("three_point_hitch_pin_mount"));
    EXPECT_EQ(out.signature.pattern.value("subfeature_count", 0), 2);
    // Cat 1 → pin dia 19 mm
    EXPECT_NEAR(out.signature.pattern.value("derived_pin_dia_mm", 0.0),
                19.0, 0.01);
}

TEST(SkillAgImplement3PointHitchPin, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(80.0, 60.0, 25.0);
    auto in    = goodInput();
    auto out   = skill::ag_implement_3point_hitch_pin::apply(*stock, in);
    auto cands = skill::ag_implement_3point_hitch_pin::recognize(*out.workpiece);
    ASSERT_FALSE(cands.empty());
    EXPECT_DOUBLE_EQ(cands[0].confidence, 1.0);
}
