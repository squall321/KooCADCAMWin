// @lat: [[process/test-strategy#nacelle_bedplate_mount_pad]]

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/nacelle_bedplate_mount_pad.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

using namespace koocadcam;

namespace {
double volumeOf(const TopoDS_Shape& s) {
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}

skill::nacelle_bedplate_mount_pad::Input goodInput()
{
    skill::nacelle_bedplate_mount_pad::Input in;
    in.face_id           = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.center_xy         = gp_Pnt(150.0, 150.0, 0.0);
    in.pad_length_mm     = 200.0;
    in.pad_width_mm      = 150.0;
    in.pad_height_mm     = 30.0;
    in.bolt_spacing_x_mm = 140.0;
    in.bolt_spacing_y_mm = 100.0;
    in.bolt_thread_key   = "M20";
    return in;
}
}  // namespace

TEST(SkillNacelleBedplateMountPad, ApplyAddsMaterial)
{
    auto stock = skill::createCuboidStock(300.0, 300.0, 60.0);
    const double v0 = volumeOf(stock->shape());
    auto in  = goodInput();
    auto out = skill::nacelle_bedplate_mount_pad::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    const double v1 = volumeOf(out.workpiece->shape());

    EXPECT_GT(v1 - v0, 0.0);   // pad fused (net positive) minus 4 holes
    EXPECT_GT(out.workpiece->faceCount(), stock->faceCount());
}

TEST(SkillNacelleBedplateMountPad, ValidateRejectsUnknownThread)
{
    auto stock = skill::createCuboidStock(300.0, 300.0, 60.0);
    auto in = goodInput();
    in.bolt_thread_key = "M99";

    auto r = skill::nacelle_bedplate_mount_pad::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-THREAD") { found = true; break; }
    EXPECT_TRUE(found);

    EXPECT_THROW(skill::nacelle_bedplate_mount_pad::apply(*stock, in),
                 skill::SkillError);
}

TEST(SkillNacelleBedplateMountPad, ValidateRejectsTightSpacing)
{
    auto stock = skill::createCuboidStock(300.0, 300.0, 60.0);
    auto in = goodInput();
    in.bolt_spacing_x_mm = 199.0;   // edge clearance < bolt dia

    auto r = skill::nacelle_bedplate_mount_pad::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-SPACING") { found = true; break; }
    EXPECT_TRUE(found);
}

TEST(SkillNacelleBedplateMountPad, SignatureCompound)
{
    auto stock = skill::createCuboidStock(300.0, 300.0, 60.0);
    auto in    = goodInput();
    auto out   = skill::nacelle_bedplate_mount_pad::apply(*stock, in);

    EXPECT_EQ(out.signature.skill_id,
              std::string("nacelle_bedplate_mount_pad"));
    EXPECT_TRUE(out.signature.pattern.value("is_compound", false));
    EXPECT_EQ(out.signature.pattern.value("wind_feature_type", std::string()),
              std::string("nacelle_bedplate_mount_pad"));
    EXPECT_EQ(out.signature.pattern.value("subfeature_count", 0), 5);
}

TEST(SkillNacelleBedplateMountPad, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(300.0, 300.0, 60.0);
    auto in    = goodInput();
    auto out   = skill::nacelle_bedplate_mount_pad::apply(*stock, in);
    auto cands = skill::nacelle_bedplate_mount_pad::recognize(*out.workpiece);
    ASSERT_FALSE(cands.empty());
    EXPECT_DOUBLE_EQ(cands[0].confidence, 1.0);
}
