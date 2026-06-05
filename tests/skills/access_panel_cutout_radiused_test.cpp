// @lat: [[process/test-strategy#access_panel_cutout_radiused]]

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/access_panel_cutout_radiused.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

using namespace koocadcam;

namespace {
double volumeOf(const TopoDS_Shape& s) {
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}

skill::access_panel_cutout_radiused::Input goodInput()
{
    skill::access_panel_cutout_radiused::Input in;
    in.face_id         = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.center_xy       = gp_Pnt(70.0, 60.0, 0.0);
    in.panel_w_mm      = 60.0;
    in.panel_h_mm      = 40.0;
    in.corner_radius_mm = 6.0;
    in.nutplate_count  = 8;
    in.rivet_dia_mm    = 3.2;
    return in;
}
}  // namespace

TEST(SkillAccessPanelCutoutRadiused, ApplyRemovesMaterial)
{
    auto stock = skill::createCuboidStock(140.0, 120.0, 5.0);
    const double v0 = volumeOf(stock->shape());
    auto in  = goodInput();
    auto out = skill::access_panel_cutout_radiused::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    const double v1 = volumeOf(out.workpiece->shape());

    EXPECT_GT(v0 - v1, 0.0);
    EXPECT_GT(out.workpiece->faceCount(), stock->faceCount());
}

TEST(SkillAccessPanelCutoutRadiused, ValidateRejectsBigCorner)
{
    auto stock = skill::createCuboidStock(140.0, 120.0, 5.0);
    auto in = goodInput();
    in.corner_radius_mm = 25.0;   // >= min(60,40)/2 = 20

    auto r = skill::access_panel_cutout_radiused::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-CORNER") { found = true; break; }
    EXPECT_TRUE(found);

    EXPECT_THROW(skill::access_panel_cutout_radiused::apply(*stock, in),
                 skill::SkillError);
}

TEST(SkillAccessPanelCutoutRadiused, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(140.0, 120.0, 5.0);
    auto in = goodInput();
    in.nutplate_count = 0;

    auto r = skill::access_panel_cutout_radiused::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-INPUT") { found = true; break; }
    EXPECT_TRUE(found);
}

TEST(SkillAccessPanelCutoutRadiused, SignatureCompound)
{
    auto stock = skill::createCuboidStock(140.0, 120.0, 5.0);
    auto in    = goodInput();
    auto out   = skill::access_panel_cutout_radiused::apply(*stock, in);

    EXPECT_TRUE(out.signature.pattern.value("is_compound", false));
    EXPECT_EQ(out.signature.pattern.value("aerostruct_feature_type", std::string()),
              std::string("access_panel_cutout_radiused"));
    EXPECT_EQ(out.signature.pattern.value("subfeature_count", 0), 1 + 8);
}

TEST(SkillAccessPanelCutoutRadiused, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(140.0, 120.0, 5.0);
    auto in    = goodInput();
    auto out   = skill::access_panel_cutout_radiused::apply(*stock, in);
    auto cands = skill::access_panel_cutout_radiused::recognize(*out.workpiece);
    ASSERT_FALSE(cands.empty());
    EXPECT_DOUBLE_EQ(cands[0].confidence, 1.0);
}
