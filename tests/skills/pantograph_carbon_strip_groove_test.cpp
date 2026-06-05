// @lat: [[process/test-strategy#pantograph_carbon_strip_groove]]

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/pantograph_carbon_strip_groove.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

using namespace koocadcam;

namespace {
double volumeOf(const TopoDS_Shape& s) {
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}

skill::pantograph_carbon_strip_groove::Input goodInput()
{
    skill::pantograph_carbon_strip_groove::Input in;
    in.face_id            = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.strip_origin       = gp_Pnt(40.0, 50.0, 0.0);
    in.strip_length_mm    = 1000.0;
    in.groove_width_mm    = 30.0;
    in.groove_depth_mm    = 12.0;
    in.clip_count         = 6;
    in.clip_slot_width_mm = 6.0;
    return in;
}
}  // namespace

TEST(SkillPantographCarbonStripGroove, ApplyRemovesMaterial)
{
    auto stock = skill::createCuboidStock(1100.0, 100.0, 40.0);
    const double v0 = volumeOf(stock->shape());
    auto in  = goodInput();
    auto out = skill::pantograph_carbon_strip_groove::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    const double v1 = volumeOf(out.workpiece->shape());

    EXPECT_GT(v0 - v1, 0.0);   // material removed (groove + clip slots)
    EXPECT_GT(out.workpiece->faceCount(), stock->faceCount());
}

TEST(SkillPantographCarbonStripGroove, ValidateRejectsClipCount)
{
    auto stock = skill::createCuboidStock(1100.0, 100.0, 40.0);
    auto in = goodInput();
    in.clip_count = 0;   // outside [1, 24]

    auto r = skill::pantograph_carbon_strip_groove::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-COUNT") { found = true; break; }
    EXPECT_TRUE(found);

    EXPECT_THROW(skill::pantograph_carbon_strip_groove::apply(*stock, in),
                 skill::SkillError);
}

TEST(SkillPantographCarbonStripGroove, ValidateRejectsWideClipSlot)
{
    auto stock = skill::createCuboidStock(1100.0, 100.0, 40.0);
    auto in = goodInput();
    in.clip_slot_width_mm = 40.0;   // >= groove_width_mm

    auto r = skill::pantograph_carbon_strip_groove::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-WIDTH") { found = true; break; }
    EXPECT_TRUE(found);
}

TEST(SkillPantographCarbonStripGroove, SignatureCompoundPantograph)
{
    auto stock = skill::createCuboidStock(1100.0, 100.0, 40.0);
    auto in    = goodInput();
    auto out   = skill::pantograph_carbon_strip_groove::apply(*stock, in);

    EXPECT_EQ(out.signature.skill_id,
              std::string("pantograph_carbon_strip_groove"));
    EXPECT_TRUE(out.signature.pattern.value("is_compound", false));
    EXPECT_EQ(out.signature.pattern.value("railway_feature_type", std::string()),
              std::string("pantograph_carbon_strip_groove"));
    EXPECT_EQ(out.signature.pattern.value("subfeature_count", 0),
              1 + in.clip_count);
}

TEST(SkillPantographCarbonStripGroove, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(1100.0, 100.0, 40.0);
    auto in    = goodInput();
    auto out   = skill::pantograph_carbon_strip_groove::apply(*stock, in);
    auto cands = skill::pantograph_carbon_strip_groove::recognize(*out.workpiece);
    ASSERT_FALSE(cands.empty());
    EXPECT_DOUBLE_EQ(cands[0].confidence, 1.0);
}
