// @lat: [[process/test-strategy#speaker_cone_mount_4hole]]

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/speaker_cone_mount_4hole.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

using namespace koocadcam;

namespace {
double volumeOf(const TopoDS_Shape& s) {
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}

skill::speaker_cone_mount_4hole::Input goodInput()
{
    skill::speaker_cone_mount_4hole::Input in;
    in.face_id          = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.center_x_mm      = 150.0;
    in.center_y_mm      = 150.0;
    in.axis_dir         = gp_Dir(0, 0, -1);
    in.cone_dia_mm      = 165.0;   // 6.5"
    in.mount_bolt_count = 4;
    in.mount_pcd_mm     = 190.0;
    in.bolt_thread_size_key = "M5";
    in.recess_dia_mm    = 0.0;     // no recess
    in.recess_depth_mm  = 0.0;
    in.panel_thickness_mm = 18.0;
    return in;
}
}  // namespace

// Test 1: apply removes material — cone hole + 4 bolt clearance holes.
TEST(SkillSpeakerConeMount4Hole, ApplyRemovesMaterial)
{
    auto stock = skill::createCuboidStock(300.0, 300.0, 18.0);
    const double v0 = volumeOf(stock->shape());
    auto in  = goodInput();
    auto out = skill::speaker_cone_mount_4hole::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    const double v1 = volumeOf(out.workpiece->shape());
    EXPECT_GT(v0 - v1, 0.0);
    EXPECT_GT(out.workpiece->faceCount(), stock->faceCount());
}

TEST(SkillSpeakerConeMount4Hole, ValidateRejectsBoltCountOutOfSet)
{
    auto stock = skill::createCuboidStock(300.0, 300.0, 18.0);
    auto in = goodInput();
    in.mount_bolt_count = 6;   // not in {4, 8}

    auto r = skill::speaker_cone_mount_4hole::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-BOLT-COUNT") { found = true; break; }
    EXPECT_TRUE(found);
    EXPECT_THROW(skill::speaker_cone_mount_4hole::apply(*stock, in),
                 skill::SkillError);
}

TEST(SkillSpeakerConeMount4Hole, ValidateRejectsNonStandardConeDia)
{
    auto stock = skill::createCuboidStock(300.0, 300.0, 18.0);
    auto in = goodInput();
    in.cone_dia_mm = 175.0;   // not in {100, 130, 165, 200, 250}
    // bump PCD to keep clearance valid
    in.mount_pcd_mm = 200.0;

    auto r = skill::speaker_cone_mount_4hole::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-CONE-DIA") { found = true; break; }
    EXPECT_TRUE(found);
}

TEST(SkillSpeakerConeMount4Hole, SignatureRecordsCompoundAndBoltCount)
{
    auto stock = skill::createCuboidStock(300.0, 300.0, 18.0);
    auto in    = goodInput();
    auto out   = skill::speaker_cone_mount_4hole::apply(*stock, in);

    EXPECT_EQ(out.signature.skill_id,
              std::string("speaker_cone_mount_4hole"));
    EXPECT_TRUE(out.signature.pattern.value("is_compound", false));
    EXPECT_EQ(out.signature.pattern.value("audio_feature_type", std::string()),
              std::string("speaker_cone_mount"));
    EXPECT_EQ(out.signature.pattern.value("mount_bolt_count", 0), 4);
    // subfeature_count = 1 (cone) + 4 (bolts) = 5 (no recess)
    EXPECT_EQ(out.signature.pattern.value("subfeature_count", 0), 5);
    EXPECT_FALSE(out.signature.pattern.value("has_rear_bumper_recess", true));
}

TEST(SkillSpeakerConeMount4Hole, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(300.0, 300.0, 18.0);
    auto in    = goodInput();
    auto out   = skill::speaker_cone_mount_4hole::apply(*stock, in);
    auto cands = skill::speaker_cone_mount_4hole::recognize(*out.workpiece);
    ASSERT_FALSE(cands.empty());
    EXPECT_DOUBLE_EQ(cands[0].confidence, 1.0);
    EXPECT_EQ(cands[0].recovered_params.value("mount_bolt_count", 0),
              in.mount_bolt_count);
}
