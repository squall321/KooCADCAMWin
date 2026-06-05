// @lat: [[process/test-strategy#stringer_run_out_cutout]]

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/stringer_run_out_cutout.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

using namespace koocadcam;

namespace {
double volumeOf(const TopoDS_Shape& s) {
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}

skill::stringer_run_out_cutout::Input goodInput()
{
    skill::stringer_run_out_cutout::Input in;
    in.face_id              = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.center_xy            = gp_Pnt(40.0, 40.0, 0.0);
    in.stringer_width_mm    = 24.0;
    in.stringer_height_mm   = 18.0;
    in.mouse_hole_radius_mm = 5.0;
    return in;
}
}  // namespace

TEST(SkillStringerRunOutCutout, ApplyRemovesMaterial)
{
    auto stock = skill::createCuboidStock(80.0, 80.0, 8.0);
    const double v0 = volumeOf(stock->shape());
    auto in  = goodInput();
    auto out = skill::stringer_run_out_cutout::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    const double v1 = volumeOf(out.workpiece->shape());

    EXPECT_GT(v0 - v1, 0.0);
    EXPECT_GT(out.workpiece->faceCount(), stock->faceCount());
}

TEST(SkillStringerRunOutCutout, ValidateRejectsBigRadius)
{
    auto stock = skill::createCuboidStock(80.0, 80.0, 8.0);
    auto in = goodInput();
    in.mouse_hole_radius_mm = 12.0;   // >= min(24,18)/2 = 9

    auto r = skill::stringer_run_out_cutout::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-RADIUS") { found = true; break; }
    EXPECT_TRUE(found);

    EXPECT_THROW(skill::stringer_run_out_cutout::apply(*stock, in),
                 skill::SkillError);
}

TEST(SkillStringerRunOutCutout, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(80.0, 80.0, 8.0);
    auto in = goodInput();
    in.stringer_width_mm = 0.0;

    auto r = skill::stringer_run_out_cutout::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-INPUT") { found = true; break; }
    EXPECT_TRUE(found);
}

TEST(SkillStringerRunOutCutout, SignatureCompound)
{
    auto stock = skill::createCuboidStock(80.0, 80.0, 8.0);
    auto in    = goodInput();
    auto out   = skill::stringer_run_out_cutout::apply(*stock, in);

    EXPECT_TRUE(out.signature.pattern.value("is_compound", false));
    EXPECT_EQ(out.signature.pattern.value("aerostruct_feature_type", std::string()),
              std::string("stringer_run_out_cutout"));
    EXPECT_EQ(out.signature.pattern.value("subfeature_count", 0), 2);
}

TEST(SkillStringerRunOutCutout, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(80.0, 80.0, 8.0);
    auto in    = goodInput();
    auto out   = skill::stringer_run_out_cutout::apply(*stock, in);
    auto cands = skill::stringer_run_out_cutout::recognize(*out.workpiece);
    ASSERT_FALSE(cands.empty());
    EXPECT_DOUBLE_EQ(cands[0].confidence, 1.0);
}
