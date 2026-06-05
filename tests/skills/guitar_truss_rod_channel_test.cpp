// @lat: [[process/test-strategy#guitar_truss_rod_channel]]

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/guitar_truss_rod_channel.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

using namespace koocadcam;

namespace {
double volumeOf(const TopoDS_Shape& s) {
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}

skill::guitar_truss_rod_channel::Input goodInput()
{
    skill::guitar_truss_rod_channel::Input in;
    in.face_id              = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.rod_axis_origin      = gp_Pnt(20.0, 30.0, 0.0);   // start near one end
    in.rod_axis_dir         = gp_Dir(1, 0, 0);
    in.rod_length_mm        = 450.0;
    in.rod_channel_width_mm = 6.0;
    in.rod_channel_depth_mm = 7.0;
    in.nut_recess_dia_mm    = 7.0;
    in.nut_recess_depth_mm  = 10.0;
    in.end_anchor_recess_w_mm = 8.0;
    in.end_anchor_recess_l_mm = 12.0;
    in.end_anchor_recess_d_mm = 8.0;
    return in;
}
}  // namespace

// Test 1: apply removes material from a guitar-neck-blank-sized stock.
TEST(SkillGuitarTrussRodChannel, ApplyRemovesMaterial)
{
    auto stock = skill::createCuboidStock(500.0, 60.0, 20.0);
    const double v0 = volumeOf(stock->shape());
    auto in  = goodInput();
    auto out = skill::guitar_truss_rod_channel::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    const double v1 = volumeOf(out.workpiece->shape());
    EXPECT_GT(v0 - v1, 0.0);
    EXPECT_GT(out.workpiece->faceCount(), stock->faceCount());
}

TEST(SkillGuitarTrussRodChannel, ValidateRejectsBadChannelWidth)
{
    auto stock = skill::createCuboidStock(500.0, 60.0, 20.0);
    auto in = goodInput();
    in.rod_channel_width_mm = 2.0;   // < 4 mm minimum

    auto r = skill::guitar_truss_rod_channel::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-CHANNEL") { found = true; break; }
    EXPECT_TRUE(found);
    EXPECT_THROW(skill::guitar_truss_rod_channel::apply(*stock, in),
                 skill::SkillError);
}

TEST(SkillGuitarTrussRodChannel, SignatureRecordsAudioFeatureType)
{
    auto stock = skill::createCuboidStock(500.0, 60.0, 20.0);
    auto in    = goodInput();
    auto out   = skill::guitar_truss_rod_channel::apply(*stock, in);

    EXPECT_EQ(out.signature.skill_id,
              std::string("guitar_truss_rod_channel"));
    EXPECT_TRUE(out.signature.pattern.value("is_compound", false));
    EXPECT_EQ(out.signature.pattern.value("audio_feature_type", std::string()),
              std::string("guitar_truss_rod_channel"));
    EXPECT_EQ(out.signature.pattern.value("subfeature_count", 0), 3);
    EXPECT_NEAR(out.signature.pattern.value("rod_length_mm", 0.0), 450.0, 1e-6);
}

TEST(SkillGuitarTrussRodChannel, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(500.0, 60.0, 20.0);
    auto in    = goodInput();
    auto out   = skill::guitar_truss_rod_channel::apply(*stock, in);
    auto cands = skill::guitar_truss_rod_channel::recognize(*out.workpiece);
    ASSERT_FALSE(cands.empty());
    EXPECT_DOUBLE_EQ(cands[0].confidence, 1.0);
    EXPECT_EQ(cands[0].recovered_params.value("rod_length_mm", 0.0),
              in.rod_length_mm);
}
