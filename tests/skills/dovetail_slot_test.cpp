// @lat: [[process/test-strategy#skill round-trip]]

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/dovetail_slot.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

#include <cmath>

using namespace koocadcam;

namespace {
double volumeOf(const TopoDS_Shape& s) {
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}
}

TEST(SkillDovetailSlot, ApplyCreatesDovetail)
{
    auto stock = skill::createCuboidStock(80.0, 40.0, 10.0);

    skill::dovetail_slot::Input in;
    in.entry_face       = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.start_x_mm       = 20.0; in.start_y_mm = 20.0;
    in.end_x_mm         = 60.0; in.end_y_mm   = 20.0;
    in.top_width_mm     = 4.0;
    in.bottom_width_mm  = 8.0;
    in.depth_mm         = 3.0;
    in.wall_angle_deg   = 30.0;

    auto out = skill::dovetail_slot::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    // Trapezoidal cross-section area × length
    const double L = 40.0;
    const double crossArea = (4.0 + 8.0) / 2.0 * 3.0;  // = 18
    const double approxRem = crossArea * L;
    const double removed = volumeOf(stock->shape()) - volumeOf(out.workpiece->shape());
    EXPECT_NEAR(removed, approxRem, approxRem * 0.10);

    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_EQ(out.signature.skill_id, std::string("dovetail_slot"));
}

TEST(SkillDovetailSlot, ValidateRejectsSubMin)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);
    skill::dovetail_slot::Input in;
    in.entry_face       = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.start_x_mm = 10.0; in.start_y_mm = 25.0;
    in.end_x_mm   = 40.0; in.end_y_mm   = 25.0;
    in.top_width_mm     = 0.5;  // < 0.8 DFM-002
    in.bottom_width_mm  = 4.0;
    in.depth_mm         = 2.0;
    in.wall_angle_deg   = 30.0;

    auto r = skill::dovetail_slot::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_THROW(skill::dovetail_slot::apply(*stock, in), skill::SkillError);
}

TEST(SkillDovetailSlot, ValidateRejectsBadAngle)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);
    skill::dovetail_slot::Input in;
    in.entry_face       = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.start_x_mm = 10.0; in.start_y_mm = 25.0;
    in.end_x_mm   = 40.0; in.end_y_mm   = 25.0;
    in.top_width_mm     = 4.0;
    in.bottom_width_mm  = 6.0;
    in.depth_mm         = 2.0;
    in.wall_angle_deg   = 5.0;     // < 15 DFM-011

    auto r = skill::dovetail_slot::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-011") { found = true; break; }
    EXPECT_TRUE(found);
}

TEST(SkillDovetailSlot, ReverseDovetailWorks)
{
    // top wider than bottom = reverse dovetail (acceptable per spec)
    auto stock = skill::createCuboidStock(80.0, 40.0, 10.0);
    skill::dovetail_slot::Input in;
    in.entry_face       = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.start_x_mm = 20.0; in.start_y_mm = 20.0;
    in.end_x_mm   = 60.0; in.end_y_mm   = 20.0;
    in.top_width_mm     = 8.0;
    in.bottom_width_mm  = 4.0;
    in.depth_mm         = 3.0;
    in.wall_angle_deg   = 30.0;

    auto out = skill::dovetail_slot::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    // pattern should record reverse-dovetail flag
    EXPECT_TRUE(out.signature.pattern["is_reverse_dovetail"].get<bool>());
}

TEST(SkillDovetailSlot, RecognizeFinds)
{
    auto stock = skill::createCuboidStock(80.0, 40.0, 10.0);
    skill::dovetail_slot::Input in;
    in.entry_face       = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.start_x_mm = 20.0; in.start_y_mm = 20.0;
    in.end_x_mm   = 60.0; in.end_y_mm   = 20.0;
    in.top_width_mm     = 4.0;
    in.bottom_width_mm  = 8.0;
    in.depth_mm         = 3.0;
    in.wall_angle_deg   = 30.0;

    auto out = skill::dovetail_slot::apply(*stock, in);
    auto cands = skill::dovetail_slot::recognize(*out.workpiece);
    ASSERT_FALSE(cands.empty());
    EXPECT_NEAR(cands[0].recovered_params["depth_mm"].get<double>(), 3.0, 0.2);
}
