// @lat: [[process/test-strategy#cam_lock_minifix_bore]]

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/cam_lock_minifix_bore.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

using namespace koocadcam;

namespace {
double volumeOf(const TopoDS_Shape& s) {
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}

skill::cam_lock_minifix_bore::Input goodInput()
{
    skill::cam_lock_minifix_bore::Input in;
    in.face_id         = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.center_xy       = gp_Pnt(40.0, 40.0, 0.0);
    in.cam_dia_mm      = 15.0;
    in.cam_depth_mm    = 12.5;
    in.dowel_dia_mm    = 8.0;
    in.dowel_offset_mm = 32.0;
    in.bolt_dia_mm     = 7.0;
    return in;
}
}  // namespace

TEST(SkillCamLockMinifixBore, ApplyRemovesMaterial)
{
    auto stock = skill::createCuboidStock(100.0, 80.0, 18.0);
    const double v0 = volumeOf(stock->shape());
    auto in  = goodInput();
    auto out = skill::cam_lock_minifix_bore::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    const double v1 = volumeOf(out.workpiece->shape());

    EXPECT_GT(v0 - v1, 0.0);   // cam bore + dowel bore + bolt cross hole
    EXPECT_GT(out.workpiece->faceCount(), stock->faceCount());
}

TEST(SkillCamLockMinifixBore, ValidateRejectsCamDiaOutOfRange)
{
    auto stock = skill::createCuboidStock(100.0, 80.0, 18.0);
    auto in = goodInput();
    in.cam_dia_mm = 30.0;   // > 25

    auto r = skill::cam_lock_minifix_bore::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-CAMDIA") { found = true; break; }
    EXPECT_TRUE(found);

    EXPECT_THROW(skill::cam_lock_minifix_bore::apply(*stock, in),
                 skill::SkillError);
}

TEST(SkillCamLockMinifixBore, ValidateRejectsDowelOverlap)
{
    auto stock = skill::createCuboidStock(100.0, 80.0, 18.0);
    auto in = goodInput();
    in.dowel_offset_mm = 8.0;   // dowel bore overlaps the cam bore

    auto r = skill::cam_lock_minifix_bore::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-DOWELFIT") { found = true; break; }
    EXPECT_TRUE(found);
}

TEST(SkillCamLockMinifixBore, SignatureCompound)
{
    auto stock = skill::createCuboidStock(100.0, 80.0, 18.0);
    auto in    = goodInput();
    auto out   = skill::cam_lock_minifix_bore::apply(*stock, in);

    EXPECT_TRUE(out.signature.pattern.value("is_compound", false));
    EXPECT_EQ(out.signature.pattern.value("furniture_feature_type", std::string()),
              std::string("cam_lock_minifix_joint"));
    EXPECT_EQ(out.signature.pattern.value("subfeature_count", 0), 3);
}

TEST(SkillCamLockMinifixBore, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(100.0, 80.0, 18.0);
    auto in    = goodInput();
    auto out   = skill::cam_lock_minifix_bore::apply(*stock, in);
    auto cands = skill::cam_lock_minifix_bore::recognize(*out.workpiece);
    ASSERT_FALSE(cands.empty());
    EXPECT_DOUBLE_EQ(cands[0].confidence, 1.0);
}
