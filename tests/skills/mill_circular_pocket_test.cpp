// @lat: [[process/test-strategy#skill round-trip]]

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/mill_circular_pocket.hpp"

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

TEST(SkillMillCircularPocket, ApplyCreatesPocket)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 12.0);

    skill::mill_circular_pocket::Input in;
    in.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 30.0;
    in.position_y_mm = 30.0;
    in.diameter_mm   = 10.0;
    in.depth_mm      = 3.0;

    auto out = skill::mill_circular_pocket::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    const double expectedRem = M_PI * 5.0 * 5.0 * 3.0;
    EXPECT_NEAR(volumeOf(stock->shape()) - volumeOf(out.workpiece->shape()),
                expectedRem, expectedRem * 0.05);
}

TEST(SkillMillCircularPocket, ValidateRejectsSubMin)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);
    skill::mill_circular_pocket::Input in;
    in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 25.0; in.position_y_mm = 25.0;
    in.diameter_mm = 0.5; in.depth_mm = 2.0;

    auto r = skill::mill_circular_pocket::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_THROW(skill::mill_circular_pocket::apply(*stock, in), skill::SkillError);
}

TEST(SkillMillCircularPocket, RecognizeFinds)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 12.0);
    skill::mill_circular_pocket::Input in;
    in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 30.0; in.position_y_mm = 30.0;
    in.diameter_mm = 10.0; in.depth_mm = 3.0;

    auto out = skill::mill_circular_pocket::apply(*stock, in);
    auto cands = skill::mill_circular_pocket::recognize(*out.workpiece);
    ASSERT_FALSE(cands.empty());
    EXPECT_NEAR(cands[0].recovered_params["diameter_mm"].get<double>(), 10.0, 0.1);
    EXPECT_NEAR(cands[0].recovered_params["depth_mm"].get<double>(),     3.0, 0.1);
}

TEST(SkillMillCircularPocket, RoundTripViaStep)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 12.0);
    skill::mill_circular_pocket::Input in;
    in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 20.0; in.position_y_mm = 35.0;
    in.diameter_mm = 8.0; in.depth_mm = 2.5;

    auto synth = skill::mill_circular_pocket::apply(*stock, in);

    const fs::path stepPath = fs::temp_directory_path() / "mill_circ_pkt_rt.step";
    std::string err;
    ASSERT_TRUE(io::StepIO::write(synth.workpiece->shape(), stepPath, err));
    auto reimp = io::StepIO::read(stepPath, err);
    ASSERT_TRUE(reimp.has_value());
    skill::Workpiece reim(*reimp);

    auto cands = skill::mill_circular_pocket::recognize(reim);
    ASSERT_FALSE(cands.empty());
    EXPECT_NEAR(cands[0].recovered_params["diameter_mm"].get<double>(), in.diameter_mm, 0.1);
    EXPECT_NEAR(cands[0].recovered_params["depth_mm"].get<double>(),    in.depth_mm,    0.1);
}

TEST(SkillMillCircularPocket, BottomCornerFillet)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 12.0);
    skill::mill_circular_pocket::Input in;
    in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 25.0; in.position_y_mm = 25.0;
    in.diameter_mm = 12.0; in.depth_mm = 4.0;
    in.bottom_corner_r_mm = 0.5;

    auto out = skill::mill_circular_pocket::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    // Corner-fillet variant introduces a toroidal face → faceCount increases
    // by more than the no-fillet variant.
    EXPECT_GT(out.workpiece->faceCount(), stock->faceCount());
}
