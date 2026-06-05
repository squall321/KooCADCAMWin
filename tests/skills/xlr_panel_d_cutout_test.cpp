// @lat: [[process/test-strategy#xlr_panel_d_cutout]]

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/xlr_panel_d_cutout.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

using namespace koocadcam;

namespace {
double volumeOf(const TopoDS_Shape& s) {
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}

skill::xlr_panel_d_cutout::Input goodInput()
{
    skill::xlr_panel_d_cutout::Input in;
    in.face_id          = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.center_xy        = gp_Pnt(40.0, 40.0, 0.0);
    in.bore_dia_mm      = 24.0;
    in.mount_hole_dia_mm = 3.2;
    in.mount_spacing_mm  = 19.05;
    return in;
}
}  // namespace

TEST(SkillXlrPanelDCutout, ApplyRemovesMaterial)
{
    auto stock = skill::createCuboidStock(80.0, 80.0, 3.0);
    const double v0 = volumeOf(stock->shape());
    auto in  = goodInput();
    auto out = skill::xlr_panel_d_cutout::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    const double v1 = volumeOf(out.workpiece->shape());

    EXPECT_GT(v0 - v1, 0.0);   // bore + 2 mount holes removed
    EXPECT_GT(out.workpiece->faceCount(), stock->faceCount());
}

TEST(SkillXlrPanelDCutout, ValidateRejectsBoreOutOfRange)
{
    auto stock = skill::createCuboidStock(80.0, 80.0, 3.0);
    auto in = goodInput();
    in.bore_dia_mm = 35.0;   // outside [20, 30]

    auto r = skill::xlr_panel_d_cutout::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-BORE") { found = true; break; }
    EXPECT_TRUE(found);

    EXPECT_THROW(skill::xlr_panel_d_cutout::apply(*stock, in),
                 skill::SkillError);
}

TEST(SkillXlrPanelDCutout, ValidateRejectsNonPositiveMount)
{
    auto stock = skill::createCuboidStock(80.0, 80.0, 3.0);
    auto in = goodInput();
    in.mount_hole_dia_mm = 0.0;

    auto r = skill::xlr_panel_d_cutout::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-INPUT") { found = true; break; }
    EXPECT_TRUE(found);
}

TEST(SkillXlrPanelDCutout, SignatureCompoundXlrD)
{
    auto stock = skill::createCuboidStock(80.0, 80.0, 3.0);
    auto in    = goodInput();
    auto out   = skill::xlr_panel_d_cutout::apply(*stock, in);

    EXPECT_TRUE(out.signature.pattern.value("is_compound", false));
    EXPECT_EQ(out.signature.pattern.value("audio_feature_type", std::string()),
              std::string("neutrik_d_panel_cutout"));
    EXPECT_EQ(out.signature.pattern.value("subfeature_count", 0), 3);
}

TEST(SkillXlrPanelDCutout, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(80.0, 80.0, 3.0);
    auto in    = goodInput();
    auto out   = skill::xlr_panel_d_cutout::apply(*stock, in);
    auto cands = skill::xlr_panel_d_cutout::recognize(*out.workpiece);
    ASSERT_FALSE(cands.empty());
    EXPECT_DOUBLE_EQ(cands[0].confidence, 1.0);
}
