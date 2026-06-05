// @lat: [[process/test-strategy#yaw_brake_caliper_mount]]

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/yaw_brake_caliper_mount.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

using namespace koocadcam;

namespace {
double volumeOf(const TopoDS_Shape& s) {
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}

skill::yaw_brake_caliper_mount::Input goodInput()
{
    skill::yaw_brake_caliper_mount::Input in;
    in.face_id            = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.center_xy          = gp_Pnt(75.0, 75.0, 0.0);
    in.pad_spacing_mm     = 120.0;
    in.bolt_thread_key    = "M16";
    in.bolt_dia_mm        = 17.0;
    in.disc_slot_width_mm = 40.0;
    in.disc_slot_depth_mm = 30.0;
    return in;
}
}  // namespace

TEST(SkillYawBrakeCaliperMount, ApplyRemovesMaterial)
{
    auto stock = skill::createCuboidStock(150.0, 150.0, 60.0);
    const double v0 = volumeOf(stock->shape());
    auto in  = goodInput();
    auto out = skill::yaw_brake_caliper_mount::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    const double v1 = volumeOf(out.workpiece->shape());

    EXPECT_GT(v0 - v1, 0.0);   // 4 bolt holes + disc slot removed
    EXPECT_GT(out.workpiece->faceCount(), stock->faceCount());
}

TEST(SkillYawBrakeCaliperMount, ValidateRejectsUnknownThread)
{
    auto stock = skill::createCuboidStock(150.0, 150.0, 60.0);
    auto in = goodInput();
    in.bolt_thread_key = "M99";

    auto r = skill::yaw_brake_caliper_mount::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-THREAD") { found = true; break; }
    EXPECT_TRUE(found);

    EXPECT_THROW(skill::yaw_brake_caliper_mount::apply(*stock, in),
                 skill::SkillError);
}

TEST(SkillYawBrakeCaliperMount, ValidateRejectsWideSlot)
{
    auto stock = skill::createCuboidStock(150.0, 150.0, 60.0);
    auto in = goodInput();
    in.disc_slot_width_mm = 130.0;   // >= pad_spacing 120

    auto r = skill::yaw_brake_caliper_mount::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-SLOT") { found = true; break; }
    EXPECT_TRUE(found);
}

TEST(SkillYawBrakeCaliperMount, SignatureCompound)
{
    auto stock = skill::createCuboidStock(150.0, 150.0, 60.0);
    auto in    = goodInput();
    auto out   = skill::yaw_brake_caliper_mount::apply(*stock, in);

    EXPECT_EQ(out.signature.skill_id,
              std::string("yaw_brake_caliper_mount"));
    EXPECT_TRUE(out.signature.pattern.value("is_compound", false));
    EXPECT_EQ(out.signature.pattern.value("wind_feature_type", std::string()),
              std::string("yaw_brake_caliper_mount"));
    EXPECT_EQ(out.signature.pattern.value("subfeature_count", 0), 5);
}

TEST(SkillYawBrakeCaliperMount, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(150.0, 150.0, 60.0);
    auto in    = goodInput();
    auto out   = skill::yaw_brake_caliper_mount::apply(*stock, in);
    auto cands = skill::yaw_brake_caliper_mount::recognize(*out.workpiece);
    ASSERT_FALSE(cands.empty());
    EXPECT_DOUBLE_EQ(cands[0].confidence, 1.0);
}
