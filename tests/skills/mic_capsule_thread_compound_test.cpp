// @lat: [[process/test-strategy#mic_capsule_thread_compound]]

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/mic_capsule_thread_compound.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

using namespace koocadcam;

namespace {
double volumeOf(const TopoDS_Shape& s) {
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}

skill::mic_capsule_thread_compound::Input goodInput()
{
    skill::mic_capsule_thread_compound::Input in;
    in.face_id        = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.center_x_mm    = 30.0;
    in.center_y_mm    = 30.0;
    in.axis_dir       = gp_Dir(0, 0, -1);
    in.mount_style    = "5/8-27 UNS";
    in.mount_depth_mm = 12.0;
    return in;
}
}  // namespace

// Test 1: apply removes material.
TEST(SkillMicCapsuleThreadCompound, ApplyRemovesMaterial)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 25.0);
    const double v0 = volumeOf(stock->shape());
    auto in  = goodInput();
    auto out = skill::mic_capsule_thread_compound::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    const double v1 = volumeOf(out.workpiece->shape());
    EXPECT_GT(v0 - v1, 0.0);
    EXPECT_GT(out.workpiece->faceCount(), stock->faceCount());
}

TEST(SkillMicCapsuleThreadCompound, ValidateRejectsUnknownMountStyle)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 25.0);
    auto in = goodInput();
    in.mount_style = "BOGUS-MOUNT";   // unknown

    auto r = skill::mic_capsule_thread_compound::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-MOUNT-STYLE") { found = true; break; }
    EXPECT_TRUE(found);
    EXPECT_THROW(skill::mic_capsule_thread_compound::apply(*stock, in),
                 skill::SkillError);
}

TEST(SkillMicCapsuleThreadCompound, SignatureRecords5_8_27UNS)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 25.0);
    auto in    = goodInput();
    auto out   = skill::mic_capsule_thread_compound::apply(*stock, in);

    EXPECT_EQ(out.signature.skill_id,
              std::string("mic_capsule_thread_compound"));
    EXPECT_TRUE(out.signature.pattern.value("is_compound", false));
    EXPECT_EQ(out.signature.pattern.value("audio_feature_type", std::string()),
              std::string("mic_capsule_thread"));
    EXPECT_EQ(out.signature.pattern.value("subfeature_count", 0), 2);
    EXPECT_EQ(out.signature.pattern.value("mount_style", std::string()),
              std::string("5/8-27 UNS"));
    // 5/8 inch = 15.875 mm
    EXPECT_NEAR(out.signature.pattern.value("thread_nominal_dia_mm", 0.0),
                15.875, 1e-6);
    // 27 TPI → pitch 25.4/27 ≈ 0.9407 mm
    EXPECT_NEAR(out.signature.pattern.value("thread_pitch_mm", 0.0),
                25.4 / 27.0, 1e-6);
}

TEST(SkillMicCapsuleThreadCompound, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 25.0);
    auto in    = goodInput();
    in.mount_style = "3/8-16 UNC";
    auto out   = skill::mic_capsule_thread_compound::apply(*stock, in);
    auto cands = skill::mic_capsule_thread_compound::recognize(*out.workpiece);
    ASSERT_FALSE(cands.empty());
    EXPECT_DOUBLE_EQ(cands[0].confidence, 1.0);
    EXPECT_EQ(cands[0].recovered_params.value("mount_style", std::string()),
              std::string("3/8-16 UNC"));
}
