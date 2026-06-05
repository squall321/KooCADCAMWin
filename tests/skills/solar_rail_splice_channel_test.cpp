// @lat: [[process/test-strategy#solar_rail_splice_channel]]

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/solar_rail_splice_channel.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

using namespace koocadcam;

namespace {
double volumeOf(const TopoDS_Shape& s) {
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}

skill::solar_rail_splice_channel::Input goodInput()
{
    skill::solar_rail_splice_channel::Input in;
    in.face_id            = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.center_xy          = gp_Pnt(50.0, 30.0, 0.0);
    in.channel_width_mm   = 12.0;
    in.channel_depth_mm   = 5.0;
    in.channel_length_mm  = 60.0;
    in.screw_count        = 4;
    in.screw_pilot_dia_mm = 3.5;
    return in;
}
}  // namespace

TEST(SkillSolarRailSpliceChannel, ApplyRemovesMaterial)
{
    auto stock = skill::createCuboidStock(100.0, 60.0, 25.0);
    const double v0 = volumeOf(stock->shape());
    auto in  = goodInput();
    auto out = skill::solar_rail_splice_channel::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    const double v1 = volumeOf(out.workpiece->shape());

    EXPECT_GT(v0 - v1, 0.0);
    EXPECT_GT(out.workpiece->faceCount(), stock->faceCount());
}

TEST(SkillSolarRailSpliceChannel, ValidateRejectsBadScrewCount)
{
    auto stock = skill::createCuboidStock(100.0, 60.0, 25.0);
    auto in = goodInput();
    in.screw_count = 20;

    auto r = skill::solar_rail_splice_channel::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-SCREWS") { found = true; break; }
    EXPECT_TRUE(found);

    EXPECT_THROW(skill::solar_rail_splice_channel::apply(*stock, in),
                 skill::SkillError);
}

TEST(SkillSolarRailSpliceChannel, ValidateRejectsOversizePilot)
{
    auto stock = skill::createCuboidStock(100.0, 60.0, 25.0);
    auto in = goodInput();
    in.screw_pilot_dia_mm = 15.0;   // >= channel_width_mm

    auto r = skill::solar_rail_splice_channel::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-PILOT") { found = true; break; }
    EXPECT_TRUE(found);
}

TEST(SkillSolarRailSpliceChannel, SignatureCompound)
{
    auto stock = skill::createCuboidStock(100.0, 60.0, 25.0);
    auto in    = goodInput();
    auto out   = skill::solar_rail_splice_channel::apply(*stock, in);

    EXPECT_TRUE(out.signature.pattern.value("is_compound", false));
    EXPECT_EQ(out.signature.pattern.value("solar_feature_type", std::string()),
              std::string("rail_splice_channel"));
    EXPECT_EQ(out.signature.pattern.value("subfeature_count", 0),
              1 + in.screw_count);
}

TEST(SkillSolarRailSpliceChannel, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(100.0, 60.0, 25.0);
    auto in    = goodInput();
    auto out   = skill::solar_rail_splice_channel::apply(*stock, in);
    auto cands = skill::solar_rail_splice_channel::recognize(*out.workpiece);
    ASSERT_FALSE(cands.empty());
    EXPECT_DOUBLE_EQ(cands[0].confidence, 1.0);
}
