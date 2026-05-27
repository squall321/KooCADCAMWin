// @lat: [[process/test-strategy#skill round-trip]]

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/T_slot.hpp"

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

TEST(SkillTSlot, ApplyCreatesTSlot)
{
    auto stock = skill::createCuboidStock(80.0, 40.0, 20.0);

    skill::T_slot::Input in;
    in.entry_face       = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.start_x_mm = 20.0; in.start_y_mm = 20.0;
    in.end_x_mm   = 60.0; in.end_y_mm   = 20.0;
    in.neck_width_mm    = 4.0;
    in.flange_width_mm  = 10.0;
    in.neck_depth_mm    = 4.0;
    in.flange_depth_mm  = 5.0;

    auto out = skill::T_slot::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    // T-cross-section volume = (neck_w × neck_d + flange_w × flange_d) × length
    const double L = 40.0;
    const double crossArea = 4.0 * 4.0 + 10.0 * 5.0;  // 16 + 50 = 66
    const double approxRem = crossArea * L;
    const double removed = volumeOf(stock->shape()) - volumeOf(out.workpiece->shape());
    EXPECT_NEAR(removed, approxRem, approxRem * 0.05);

    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_EQ(out.signature.skill_id, std::string("T_slot"));
}

TEST(SkillTSlot, ValidateRejectsBadProportions)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);
    skill::T_slot::Input in;
    in.entry_face       = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.start_x_mm = 10.0; in.start_y_mm = 25.0;
    in.end_x_mm   = 40.0; in.end_y_mm   = 25.0;
    in.neck_width_mm    = 6.0;
    in.flange_width_mm  = 4.0;     // flange < neck → not a T-slot
    in.neck_depth_mm    = 3.0;
    in.flange_depth_mm  = 2.0;

    auto r = skill::T_slot::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_THROW(skill::T_slot::apply(*stock, in), skill::SkillError);
}

TEST(SkillTSlot, ValidateRejectsSubMin)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);
    skill::T_slot::Input in;
    in.entry_face       = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.start_x_mm = 10.0; in.start_y_mm = 25.0;
    in.end_x_mm   = 40.0; in.end_y_mm   = 25.0;
    in.neck_width_mm    = 0.5;     // < 0.8
    in.flange_width_mm  = 4.0;
    in.neck_depth_mm    = 2.0;
    in.flange_depth_mm  = 2.0;

    auto r = skill::T_slot::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-002") { found = true; break; }
    EXPECT_TRUE(found);
}

TEST(SkillTSlot, RecognizeFinds)
{
    auto stock = skill::createCuboidStock(80.0, 40.0, 20.0);
    skill::T_slot::Input in;
    in.entry_face       = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.start_x_mm = 20.0; in.start_y_mm = 20.0;
    in.end_x_mm   = 60.0; in.end_y_mm   = 20.0;
    in.neck_width_mm    = 4.0;
    in.flange_width_mm  = 10.0;
    in.neck_depth_mm    = 4.0;
    in.flange_depth_mm  = 5.0;

    auto out = skill::T_slot::apply(*stock, in);
    auto cands = skill::T_slot::recognize(*out.workpiece);
    // Recognition is heuristic; we only require that at least one candidate
    // is returned and that flange_depth_mm is in the right ballpark.
    ASSERT_FALSE(cands.empty());
    EXPECT_NEAR(cands[0].recovered_params["flange_depth_mm"].get<double>(), 5.0, 1.0);
}
