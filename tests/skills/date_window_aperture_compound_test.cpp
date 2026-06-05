// @lat: [[process/test-strategy#date_window_aperture_compound]]

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/date_window_aperture_compound.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

using namespace koocadcam;

namespace {
double volumeOf(const TopoDS_Shape& s) {
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}

skill::date_window_aperture_compound::Input goodInput()
{
    skill::date_window_aperture_compound::Input in;
    in.face_id              = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.center_xy            = gp_Pnt(20.0, 20.0, 0.0);
    in.window_len_mm        = 4.0;
    in.window_wid_mm        = 3.0;
    in.bevel_mm             = 0.4;
    in.glass_step_depth_mm  = 0.6;
    in.glass_step_margin_mm = 1.0;
    return in;
}
}  // namespace

TEST(SkillDateWindowApertureCompound, ApplyRemovesMaterial)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 3.0);
    const double v0 = volumeOf(stock->shape());
    auto in  = goodInput();
    auto out = skill::date_window_aperture_compound::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    const double v1 = volumeOf(out.workpiece->shape());

    EXPECT_GT(v0 - v1, 0.0);
    EXPECT_GT(out.workpiece->faceCount(), stock->faceCount());
}

TEST(SkillDateWindowApertureCompound, ValidateRejectsBadBevel)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 3.0);
    auto in = goodInput();
    in.bevel_mm = 2.0;   // >= window_wid/2 = 1.5

    auto r = skill::date_window_aperture_compound::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-BEVEL") { found = true; break; }
    EXPECT_TRUE(found);

    EXPECT_THROW(skill::date_window_aperture_compound::apply(*stock, in),
                 skill::SkillError);
}

TEST(SkillDateWindowApertureCompound, ValidateRejectsSmallMargin)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 3.0);
    auto in = goodInput();
    in.glass_step_margin_mm = 0.2;   // <= bevel (0.4)

    auto r = skill::date_window_aperture_compound::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-MARGIN") { found = true; break; }
    EXPECT_TRUE(found);
}

TEST(SkillDateWindowApertureCompound, SignatureCompoundWatch)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 3.0);
    auto in    = goodInput();
    auto out   = skill::date_window_aperture_compound::apply(*stock, in);

    EXPECT_EQ(out.signature.skill_id,
              std::string("date_window_aperture_compound"));
    EXPECT_TRUE(out.signature.pattern.value("is_compound", false));
    EXPECT_EQ(out.signature.pattern.value("watch_feature_type", std::string()),
              std::string("date_window_aperture"));
    EXPECT_EQ(out.signature.pattern.value("subfeature_count", 0), 3);
}

TEST(SkillDateWindowApertureCompound, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 3.0);
    auto in    = goodInput();
    auto out   = skill::date_window_aperture_compound::apply(*stock, in);
    auto cands = skill::date_window_aperture_compound::recognize(*out.workpiece);
    ASSERT_FALSE(cands.empty());
    EXPECT_DOUBLE_EQ(cands[0].confidence, 1.0);
}
