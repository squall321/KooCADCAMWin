// @lat: [[process/test-strategy#skill round-trip]]

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/mill_slot.hpp"

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

TEST(SkillMillSlot, ApplyCreatesSlot)
{
    auto stock = skill::createCuboidStock(80.0, 40.0, 10.0);

    skill::mill_slot::Input in;
    in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.start_x_mm = 20.0; in.start_y_mm = 20.0;
    in.end_x_mm   = 60.0; in.end_y_mm   = 20.0;
    in.width_mm   = 6.0; in.depth_mm    = 3.0;

    auto out = skill::mill_slot::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    // Stadium volume = π r² × depth + length × width × depth
    const double r = 3.0;
    const double L = 40.0;
    const double approxRem = (M_PI * r * r + L * 6.0) * 3.0;
    const double removed = volumeOf(stock->shape()) - volumeOf(out.workpiece->shape());
    EXPECT_NEAR(removed, approxRem, approxRem * 0.10);
}

TEST(SkillMillSlot, ValidateRejectsSubMin)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);
    skill::mill_slot::Input in;
    in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.start_x_mm = 10.0; in.start_y_mm = 25.0;
    in.end_x_mm   = 40.0; in.end_y_mm   = 25.0;
    in.width_mm   = 0.5; in.depth_mm    = 2.0;

    auto r = skill::mill_slot::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_THROW(skill::mill_slot::apply(*stock, in), skill::SkillError);
}

TEST(SkillMillSlot, RecognizeFinds)
{
    auto stock = skill::createCuboidStock(80.0, 40.0, 10.0);
    skill::mill_slot::Input in;
    in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.start_x_mm = 20.0; in.start_y_mm = 20.0;
    in.end_x_mm   = 60.0; in.end_y_mm   = 20.0;
    in.width_mm   = 6.0; in.depth_mm    = 3.0;

    auto out = skill::mill_slot::apply(*stock, in);
    auto cands = skill::mill_slot::recognize(*out.workpiece);
    ASSERT_FALSE(cands.empty());
    EXPECT_NEAR(cands[0].recovered_params["width_mm"].get<double>(), 6.0, 0.1);
    EXPECT_NEAR(cands[0].recovered_params["depth_mm"].get<double>(), 3.0, 0.1);
}

TEST(SkillMillSlot, RoundTripViaStep)
{
    auto stock = skill::createCuboidStock(80.0, 40.0, 10.0);
    skill::mill_slot::Input in;
    in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.start_x_mm = 25.0; in.start_y_mm = 20.0;
    in.end_x_mm   = 55.0; in.end_y_mm   = 20.0;
    in.width_mm   = 5.0; in.depth_mm    = 2.5;

    auto synth = skill::mill_slot::apply(*stock, in);

    const fs::path stepPath = fs::temp_directory_path() / "mill_slot_rt.step";
    std::string err;
    ASSERT_TRUE(io::StepIO::write(synth.workpiece->shape(), stepPath, err));
    auto reimp = io::StepIO::read(stepPath, err);
    ASSERT_TRUE(reimp.has_value());
    skill::Workpiece reim(*reimp);

    auto cands = skill::mill_slot::recognize(reim);
    ASSERT_FALSE(cands.empty());
    EXPECT_NEAR(cands[0].recovered_params["width_mm"].get<double>(), 5.0, 0.1);
    EXPECT_NEAR(cands[0].recovered_params["depth_mm"].get<double>(), 2.5, 0.1);
}

TEST(SkillMillSlot, DiagonalSlot)
{
    // A slot at 45° to verify the orientation math (stadium aligned with the
    // start→end vector, not world axes).
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);
    skill::mill_slot::Input in;
    in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.start_x_mm = 15.0; in.start_y_mm = 15.0;
    in.end_x_mm   = 35.0; in.end_y_mm   = 35.0;
    in.width_mm   = 4.0; in.depth_mm    = 2.0;

    auto out = skill::mill_slot::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
}
