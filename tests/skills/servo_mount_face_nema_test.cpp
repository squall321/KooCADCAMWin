// @lat: [[process/test-strategy#servo_mount_face_nema]]

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/servo_mount_face_nema.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

using namespace koocadcam;

namespace {
double volumeOf(const TopoDS_Shape& s) {
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}

skill::servo_mount_face_nema::Input goodInput()
{
    skill::servo_mount_face_nema::Input in;
    in.face_id         = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.center_xy       = gp_Pnt(40.0, 40.0, 0.0);   // centre of 80×80 stock
    in.nema_size       = "23";
    in.pilot_dia_mm    = 38.1;
    in.bolt_thread_key = "M5";
    return in;
}
}  // namespace

TEST(SkillServoMountFaceNema, ApplyRemovesMaterial)
{
    auto stock = skill::createCuboidStock(80.0, 80.0, 12.0);
    const double v0 = volumeOf(stock->shape());
    auto in  = goodInput();
    auto out = skill::servo_mount_face_nema::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    const double v1 = volumeOf(out.workpiece->shape());

    EXPECT_GT(v0 - v1, 0.0);   // pilot bore + 4 bolt holes removed
    EXPECT_GT(out.workpiece->faceCount(), stock->faceCount());
}

TEST(SkillServoMountFaceNema, ValidateRejectsInvalidNema)
{
    auto stock = skill::createCuboidStock(80.0, 80.0, 12.0);
    auto in = goodInput();
    in.nema_size = "11";

    auto r = skill::servo_mount_face_nema::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-NEMA") { found = true; break; }
    EXPECT_TRUE(found);

    EXPECT_THROW(skill::servo_mount_face_nema::apply(*stock, in),
                 skill::SkillError);
}

TEST(SkillServoMountFaceNema, ValidateRejectsUnknownThread)
{
    auto stock = skill::createCuboidStock(80.0, 80.0, 12.0);
    auto in = goodInput();
    in.bolt_thread_key = "M99";

    auto r = skill::servo_mount_face_nema::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-THREAD") { found = true; break; }
    EXPECT_TRUE(found);
}

TEST(SkillServoMountFaceNema, SignatureCompoundNema)
{
    auto stock = skill::createCuboidStock(80.0, 80.0, 12.0);
    auto in    = goodInput();
    auto out   = skill::servo_mount_face_nema::apply(*stock, in);

    EXPECT_EQ(out.signature.skill_id,
              std::string("servo_mount_face_nema"));
    EXPECT_TRUE(out.signature.pattern.value("is_compound", false));
    EXPECT_EQ(out.signature.pattern.value("robotics_feature_type", std::string()),
              std::string("nema_servo_mount_face"));
    EXPECT_EQ(out.signature.pattern.value("subfeature_count", 0),
              1 + 4);
}

TEST(SkillServoMountFaceNema, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(80.0, 80.0, 12.0);
    auto in    = goodInput();
    auto out   = skill::servo_mount_face_nema::apply(*stock, in);
    auto cands = skill::servo_mount_face_nema::recognize(*out.workpiece);
    ASSERT_FALSE(cands.empty());
    EXPECT_DOUBLE_EQ(cands[0].confidence, 1.0);
}
