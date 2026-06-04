// @lat: [[process/test-strategy#hose_barb_compound]]

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/hose_barb_compound.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

using namespace koocadcam;

namespace {
double volumeOf(const TopoDS_Shape& s) {
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}

skill::hose_barb_compound::Input goodInput()
{
    skill::hose_barb_compound::Input in;
    in.face_id        = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.center_x_mm    = 25.0;
    in.center_y_mm    = 25.0;
    in.axis_dir       = gp_Dir(0, 0, 1);
    in.base_dia_mm    = 8.0;
    in.barb_count     = 3;
    in.barb_pitch_mm  = 3.5;
    in.barb_max_dia_mm = 11.0;
    in.taper_to_tip_mm = 2.0;
    return in;
}
}  // namespace

// Test 1: fuses material (v1 - v0 > 0 because additive).  Per the task spec,
// "v0 - v1 > 0 (material removed)" applies to subtractive features; this skill
// is additive (FUSE chain), so we instead assert v1 > v0 (material added).
TEST(SkillHoseBarbCompound, ApplyFusesMaterial)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 5.0);
    const double v0 = volumeOf(stock->shape());
    auto in  = goodInput();
    auto out = skill::hose_barb_compound::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    const double v1 = volumeOf(out.workpiece->shape());

    EXPECT_GT(v1 - v0, 0.0);   // material added (body + barbs + tip)
    EXPECT_GT(out.workpiece->faceCount(), stock->faceCount());
}

TEST(SkillHoseBarbCompound, ValidateRejectsBarbCountOutOfRange)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 5.0);
    auto in = goodInput();
    in.barb_count = 9;   // > 8

    auto r = skill::hose_barb_compound::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-BARB-N") { found = true; break; }
    EXPECT_TRUE(found);

    EXPECT_THROW(skill::hose_barb_compound::apply(*stock, in), skill::SkillError);
}

TEST(SkillHoseBarbCompound, SignatureCompoundBarbCount)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 5.0);
    auto in    = goodInput();
    auto out   = skill::hose_barb_compound::apply(*stock, in);

    EXPECT_EQ(out.signature.skill_id, std::string("hose_barb_compound"));
    EXPECT_TRUE(out.signature.pattern.value("is_compound", false));
    EXPECT_EQ(out.signature.pattern.value("hvac_feature_type", std::string()),
              std::string("hose_barb"));
    EXPECT_EQ(out.signature.pattern.value("barb_count", 0), in.barb_count);
}

TEST(SkillHoseBarbCompound, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 5.0);
    auto in    = goodInput();
    auto out   = skill::hose_barb_compound::apply(*stock, in);
    auto cands = skill::hose_barb_compound::recognize(*out.workpiece);
    ASSERT_FALSE(cands.empty());
    EXPECT_DOUBLE_EQ(cands[0].confidence, 1.0);
}
