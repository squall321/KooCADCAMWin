// @lat: [[process/test-strategy#drawer_slide_pilot_array]]

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/drawer_slide_pilot_array.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

using namespace koocadcam;

namespace {
double volumeOf(const TopoDS_Shape& s) {
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}

skill::drawer_slide_pilot_array::Input goodInput()
{
    skill::drawer_slide_pilot_array::Input in;
    in.face_id       = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.row_origin    = gp_Pnt(20.0, 30.0, 0.0);
    in.hole_dia_mm   = 3.0;
    in.hole_count    = 4;
    in.pitch_mm      = 32.0;
    in.hole_depth_mm = 10.0;
    return in;
}
}  // namespace

TEST(SkillDrawerSlidePilotArray, ApplyRemovesMaterial)
{
    auto stock = skill::createCuboidStock(150.0, 60.0, 16.0);
    const double v0 = volumeOf(stock->shape());
    auto in  = goodInput();
    auto out = skill::drawer_slide_pilot_array::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    const double v1 = volumeOf(out.workpiece->shape());

    EXPECT_GT(v0 - v1, 0.0);   // N pilot holes
    EXPECT_GT(out.workpiece->faceCount(), stock->faceCount());
}

TEST(SkillDrawerSlidePilotArray, ValidateRejectsOverlapPitch)
{
    auto stock = skill::createCuboidStock(150.0, 60.0, 16.0);
    auto in = goodInput();
    in.pitch_mm = 2.0;   // <= hole_dia_mm

    auto r = skill::drawer_slide_pilot_array::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-PITCH") { found = true; break; }
    EXPECT_TRUE(found);

    EXPECT_THROW(skill::drawer_slide_pilot_array::apply(*stock, in),
                 skill::SkillError);
}

TEST(SkillDrawerSlidePilotArray, ValidateRejectsThroughDepth)
{
    auto stock = skill::createCuboidStock(150.0, 60.0, 16.0);
    auto in = goodInput();
    in.hole_depth_mm = 20.0;   // >= panel thickness (16)

    auto r = skill::drawer_slide_pilot_array::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-DEPTH") { found = true; break; }
    EXPECT_TRUE(found);
}

TEST(SkillDrawerSlidePilotArray, SignatureCompound)
{
    auto stock = skill::createCuboidStock(150.0, 60.0, 16.0);
    auto in    = goodInput();
    auto out   = skill::drawer_slide_pilot_array::apply(*stock, in);

    EXPECT_TRUE(out.signature.pattern.value("is_compound", false));
    EXPECT_EQ(out.signature.pattern.value("furniture_feature_type", std::string()),
              std::string("drawer_slide_pilot_array"));
    EXPECT_EQ(out.signature.pattern.value("subfeature_count", 0), 4);
}

TEST(SkillDrawerSlidePilotArray, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(150.0, 60.0, 16.0);
    auto in    = goodInput();
    auto out   = skill::drawer_slide_pilot_array::apply(*stock, in);
    auto cands = skill::drawer_slide_pilot_array::recognize(*out.workpiece);
    ASSERT_FALSE(cands.empty());
    EXPECT_DOUBLE_EQ(cands[0].confidence, 1.0);
}
