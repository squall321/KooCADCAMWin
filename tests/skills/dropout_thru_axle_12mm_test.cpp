// @lat: [[process/test-strategy#dropout_thru_axle_12mm]]

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/dropout_thru_axle_12mm.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

using namespace koocadcam;

namespace {
double volumeOf(const TopoDS_Shape& s) {
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}

skill::dropout_thru_axle_12mm::Input goodInput()
{
    skill::dropout_thru_axle_12mm::Input in;
    in.face_id          = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.center_xy        = gp_Pnt(30.0, 40.0, 0.0);
    in.axle_dia_mm      = 12.0;
    in.thread_key       = "M12";
    in.hanger_bolt_dia_mm = 4.0;
    in.hanger_offset_mm = 16.0;
    return in;
}
}  // namespace

TEST(SkillDropoutThruAxle12mm, ApplyRemovesMaterial)
{
    // Dropout plate: 60 x 60 x 18 mm; axle bore at (30, 40) with hanger below.
    auto stock = skill::createCuboidStock(60.0, 60.0, 18.0);
    const double v0 = volumeOf(stock->shape());
    auto in  = goodInput();
    auto out = skill::dropout_thru_axle_12mm::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    const double v1 = volumeOf(out.workpiece->shape());

    EXPECT_GT(v0 - v1, 0.0);   // axle bore + thread bore + hanger hole
    EXPECT_GT(out.workpiece->faceCount(), stock->faceCount());
}

TEST(SkillDropoutThruAxle12mm, ValidateRejectsBadAxle)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 18.0);
    auto in = goodInput();
    in.axle_dia_mm = 15.0;   // not the 12 mm standard

    auto r = skill::dropout_thru_axle_12mm::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-AXLE") { found = true; break; }
    EXPECT_TRUE(found);

    EXPECT_THROW(skill::dropout_thru_axle_12mm::apply(*stock, in),
                 skill::SkillError);
}

TEST(SkillDropoutThruAxle12mm, ValidateRejectsUnknownThread)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 18.0);
    auto in = goodInput();
    in.thread_key = "M99";   // not in central table

    auto r = skill::dropout_thru_axle_12mm::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-THREAD") { found = true; break; }
    EXPECT_TRUE(found);
}

TEST(SkillDropoutThruAxle12mm, SignatureCompoundDropout)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 18.0);
    auto in    = goodInput();
    auto out   = skill::dropout_thru_axle_12mm::apply(*stock, in);

    EXPECT_EQ(out.signature.skill_id, std::string("dropout_thru_axle_12mm"));
    EXPECT_TRUE(out.signature.pattern.value("is_compound", false));
    EXPECT_EQ(out.signature.pattern.value("bicycle_feature_type", std::string()),
              std::string("rear_dropout_thru_axle_12mm"));
    EXPECT_EQ(out.signature.pattern.value("subfeature_count", 0), 3);
}

TEST(SkillDropoutThruAxle12mm, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 18.0);
    auto in    = goodInput();
    auto out   = skill::dropout_thru_axle_12mm::apply(*stock, in);
    auto cands = skill::dropout_thru_axle_12mm::recognize(*out.workpiece);
    ASSERT_FALSE(cands.empty());
    EXPECT_DOUBLE_EQ(cands[0].confidence, 1.0);
}
