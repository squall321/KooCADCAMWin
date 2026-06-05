// @lat: [[process/test-strategy#gripper_finger_dovetail_mount]]

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/gripper_finger_dovetail_mount.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

using namespace koocadcam;

namespace {
double volumeOf(const TopoDS_Shape& s) {
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}

skill::gripper_finger_dovetail_mount::Input goodInput()
{
    skill::gripper_finger_dovetail_mount::Input in;
    in.face_id            = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.center_xy          = gp_Pnt(20.0, 20.0, 0.0);   // centre of 40×40 stock
    in.dovetail_width_mm  = 16.0;
    in.dovetail_angle_deg = 15.0;
    in.dovetail_depth_mm  = 8.0;
    in.clamp_thread_key   = "M4";
    return in;
}
}  // namespace

TEST(SkillGripperFingerDovetailMount, ApplyRemovesMaterial)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 25.0);
    const double v0 = volumeOf(stock->shape());
    auto in  = goodInput();
    auto out = skill::gripper_finger_dovetail_mount::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    const double v1 = volumeOf(out.workpiece->shape());

    EXPECT_GT(v0 - v1, 0.0);   // dovetail slot + clamp bore removed
    EXPECT_GT(out.workpiece->faceCount(), stock->faceCount());
}

TEST(SkillGripperFingerDovetailMount, ValidateRejectsBadAngle)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 25.0);
    auto in = goodInput();
    in.dovetail_angle_deg = 45.0;   // > 30

    auto r = skill::gripper_finger_dovetail_mount::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-ROBOTICS-ANGLE") { found = true; break; }
    EXPECT_TRUE(found);

    EXPECT_THROW(skill::gripper_finger_dovetail_mount::apply(*stock, in),
                 skill::SkillError);
}

TEST(SkillGripperFingerDovetailMount, ValidateRejectsUnknownThread)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 25.0);
    auto in = goodInput();
    in.clamp_thread_key = "M99";

    auto r = skill::gripper_finger_dovetail_mount::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-THREAD") { found = true; break; }
    EXPECT_TRUE(found);
}

TEST(SkillGripperFingerDovetailMount, SignatureCompoundDovetail)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 25.0);
    auto in    = goodInput();
    auto out   = skill::gripper_finger_dovetail_mount::apply(*stock, in);

    EXPECT_EQ(out.signature.skill_id,
              std::string("gripper_finger_dovetail_mount"));
    EXPECT_TRUE(out.signature.pattern.value("is_compound", false));
    EXPECT_EQ(out.signature.pattern.value("robotics_feature_type", std::string()),
              std::string("gripper_finger_dovetail"));
    EXPECT_EQ(out.signature.pattern.value("subfeature_count", 0), 2);
}

TEST(SkillGripperFingerDovetailMount, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 25.0);
    auto in    = goodInput();
    auto out   = skill::gripper_finger_dovetail_mount::apply(*stock, in);
    auto cands = skill::gripper_finger_dovetail_mount::recognize(*out.workpiece);
    ASSERT_FALSE(cands.empty());
    EXPECT_DOUBLE_EQ(cands[0].confidence, 1.0);
}
