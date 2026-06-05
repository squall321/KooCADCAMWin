// @lat: [[process/test-strategy#cabinet_handle_dish_recess]]

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/cabinet_handle_dish_recess.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

using namespace koocadcam;

namespace {
double volumeOf(const TopoDS_Shape& s) {
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}

skill::cabinet_handle_dish_recess::Input goodInput()
{
    skill::cabinet_handle_dish_recess::Input in;
    in.face_id         = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.center_xy       = gp_Pnt(80.0, 40.0, 0.0);
    in.dish_length_mm  = 120.0;
    in.dish_width_mm   = 45.0;
    in.dish_depth_mm   = 18.0;
    in.bolt_spacing_mm = 90.0;
    in.bolt_thread_key = "M6";
    return in;
}
}  // namespace

TEST(SkillCabinetHandleDishRecess, ApplyRemovesMaterial)
{
    auto stock = skill::createCuboidStock(160.0, 80.0, 25.0);
    const double v0 = volumeOf(stock->shape());
    auto in  = goodInput();
    auto out = skill::cabinet_handle_dish_recess::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    const double v1 = volumeOf(out.workpiece->shape());

    EXPECT_GT(v0 - v1, 0.0);   // dish pocket + 2 bolt holes removed
    EXPECT_GT(out.workpiece->faceCount(), stock->faceCount());
}

TEST(SkillCabinetHandleDishRecess, ValidateRejectsUnknownThread)
{
    auto stock = skill::createCuboidStock(160.0, 80.0, 25.0);
    auto in = goodInput();
    in.bolt_thread_key = "M999";

    auto r = skill::cabinet_handle_dish_recess::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-THREAD") { found = true; break; }
    EXPECT_TRUE(found);

    EXPECT_THROW(skill::cabinet_handle_dish_recess::apply(*stock, in),
                 skill::SkillError);
}

TEST(SkillCabinetHandleDishRecess, ValidateRejectsBoltsOutsideDish)
{
    auto stock = skill::createCuboidStock(160.0, 80.0, 25.0);
    auto in = goodInput();
    in.bolt_spacing_mm = 130.0;   // wider than the 120 dish

    auto r = skill::cabinet_handle_dish_recess::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-BOLTFIT") { found = true; break; }
    EXPECT_TRUE(found);
}

TEST(SkillCabinetHandleDishRecess, SignatureCompoundDishRecess)
{
    auto stock = skill::createCuboidStock(160.0, 80.0, 25.0);
    auto in    = goodInput();
    auto out   = skill::cabinet_handle_dish_recess::apply(*stock, in);

    EXPECT_TRUE(out.signature.pattern.value("is_compound", false));
    EXPECT_EQ(out.signature.pattern.value("audio_feature_type", std::string()),
              std::string("recessed_bar_handle_dish"));
    EXPECT_EQ(out.signature.pattern.value("subfeature_count", 0), 3);
}

TEST(SkillCabinetHandleDishRecess, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(160.0, 80.0, 25.0);
    auto in    = goodInput();
    auto out   = skill::cabinet_handle_dish_recess::apply(*stock, in);
    auto cands = skill::cabinet_handle_dish_recess::recognize(*out.workpiece);
    ASSERT_FALSE(cands.empty());
    EXPECT_DOUBLE_EQ(cands[0].confidence, 1.0);
}
