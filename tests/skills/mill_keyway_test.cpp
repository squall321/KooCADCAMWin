// @lat: [[process/test-strategy#skill round-trip]]

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/mill_keyway.hpp"

#include "io/StepIO.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

#include <cmath>
#include <filesystem>

namespace fs = std::filesystem;
using namespace koocadcam;

namespace {
double volumeOf(const TopoDS_Shape& s) {
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}
}

TEST(SkillMillKeyway, ApplyCreatesKeyway)
{
    auto stock = skill::createCuboidStock(80.0, 40.0, 10.0);

    skill::mill_keyway::Input in;
    in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.start_x_mm = 20.0; in.start_y_mm = 20.0;
    in.end_x_mm   = 60.0; in.end_y_mm   = 20.0;
    in.width_mm   = 5.0;  in.depth_mm    = 3.0;

    auto out = skill::mill_keyway::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    // Plain rectangular pocket volume = length × width × depth
    const double L = 40.0;
    const double approxRem = L * 5.0 * 3.0;
    const double removed = volumeOf(stock->shape()) - volumeOf(out.workpiece->shape());
    EXPECT_NEAR(removed, approxRem, approxRem * 0.05);

    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_EQ(out.signature.skill_id, std::string("mill_keyway"));
}

TEST(SkillMillKeyway, ValidateRejectsSubMin)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);
    skill::mill_keyway::Input in;
    in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.start_x_mm = 10.0; in.start_y_mm = 25.0;
    in.end_x_mm   = 40.0; in.end_y_mm   = 25.0;
    in.width_mm   = 0.5; in.depth_mm    = 2.0;  // width < 0.8

    auto r = skill::mill_keyway::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-002") { found = true; break; }
    EXPECT_TRUE(found);
    EXPECT_THROW(skill::mill_keyway::apply(*stock, in), skill::SkillError);
}

TEST(SkillMillKeyway, RecognizeFinds)
{
    auto stock = skill::createCuboidStock(80.0, 40.0, 10.0);
    skill::mill_keyway::Input in;
    in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.start_x_mm = 20.0; in.start_y_mm = 20.0;
    in.end_x_mm   = 60.0; in.end_y_mm   = 20.0;
    in.width_mm   = 5.0; in.depth_mm    = 3.0;

    auto out = skill::mill_keyway::apply(*stock, in);
    auto cands = skill::mill_keyway::recognize(*out.workpiece);
    ASSERT_FALSE(cands.empty());
    EXPECT_NEAR(cands[0].recovered_params["width_mm"].get<double>(), 5.0, 0.2);
    EXPECT_NEAR(cands[0].recovered_params["depth_mm"].get<double>(), 3.0, 0.1);
}

TEST(SkillMillKeyway, DiagonalKeyway)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);
    skill::mill_keyway::Input in;
    in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.start_x_mm = 15.0; in.start_y_mm = 15.0;
    in.end_x_mm   = 35.0; in.end_y_mm   = 35.0;
    in.width_mm   = 4.0; in.depth_mm    = 2.0;

    auto out = skill::mill_keyway::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
}
