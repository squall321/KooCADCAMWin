// @lat: [[process/test-strategy#linear_rail_carriage_pocket]]

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/linear_rail_carriage_pocket.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

using namespace koocadcam;

namespace {
double volumeOf(const TopoDS_Shape& s) {
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}

skill::linear_rail_carriage_pocket::Input goodInput()
{
    skill::linear_rail_carriage_pocket::Input in;
    in.face_id          = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.center_xy        = gp_Pnt(40.0, 20.0, 0.0);   // centre of 80×40 stock
    in.rail_width_mm    = 15.0;
    in.groove_width_mm  = 3.0;
    in.groove_depth_mm  = 2.0;
    in.mount_hole_count = 4;
    in.mount_thread_key = "M4";
    in.mount_pitch_mm   = 20.0;
    return in;
}
}  // namespace

TEST(SkillLinearRailCarriagePocket, ApplyRemovesMaterial)
{
    auto stock = skill::createCuboidStock(80.0, 40.0, 20.0);
    const double v0 = volumeOf(stock->shape());
    auto in  = goodInput();
    auto out = skill::linear_rail_carriage_pocket::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    const double v1 = volumeOf(out.workpiece->shape());

    EXPECT_GT(v0 - v1, 0.0);   // 2 grooves + N mount holes removed
    EXPECT_GT(out.workpiece->faceCount(), stock->faceCount());
}

TEST(SkillLinearRailCarriagePocket, ValidateRejectsUnknownThread)
{
    auto stock = skill::createCuboidStock(80.0, 40.0, 20.0);
    auto in = goodInput();
    in.mount_thread_key = "M99";

    auto r = skill::linear_rail_carriage_pocket::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-THREAD") { found = true; break; }
    EXPECT_TRUE(found);

    EXPECT_THROW(skill::linear_rail_carriage_pocket::apply(*stock, in),
                 skill::SkillError);
}

TEST(SkillLinearRailCarriagePocket, ValidateRejectsGrooveOverrun)
{
    auto stock = skill::createCuboidStock(80.0, 40.0, 20.0);
    auto in = goodInput();
    in.groove_width_mm = 9.0;   // 2*9 = 18 >= rail_width 15

    auto r = skill::linear_rail_carriage_pocket::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-ROBOTICS-RAIL") { found = true; break; }
    EXPECT_TRUE(found);
}

TEST(SkillLinearRailCarriagePocket, SignatureCompoundCarriage)
{
    auto stock = skill::createCuboidStock(80.0, 40.0, 20.0);
    auto in    = goodInput();
    auto out   = skill::linear_rail_carriage_pocket::apply(*stock, in);

    EXPECT_EQ(out.signature.skill_id,
              std::string("linear_rail_carriage_pocket"));
    EXPECT_TRUE(out.signature.pattern.value("is_compound", false));
    EXPECT_EQ(out.signature.pattern.value("robotics_feature_type", std::string()),
              std::string("linear_guide_carriage_underside"));
    EXPECT_EQ(out.signature.pattern.value("subfeature_count", 0),
              2 + 4);
}

TEST(SkillLinearRailCarriagePocket, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(80.0, 40.0, 20.0);
    auto in    = goodInput();
    auto out   = skill::linear_rail_carriage_pocket::apply(*stock, in);
    auto cands = skill::linear_rail_carriage_pocket::recognize(*out.workpiece);
    ASSERT_FALSE(cands.empty());
    EXPECT_DOUBLE_EQ(cands[0].confidence, 1.0);
}
