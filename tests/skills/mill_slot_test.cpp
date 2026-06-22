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

// ── SIDE obround port (USB-C / SIM tray) — axis along -Y, not -Z ───────────

// A stadium port cut into the +Y SIDE face: apply removes the right volume.
TEST(SkillMillSlot, SideObroundApplyVolume)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 30.0);   // spans Y in [0,60]
    skill::mill_slot::Input in;
    // entry on the +Y face; bore along -Y.  start/end step along X at mid height.
    in.entry_face = skill::FaceByNormal{ gp_Dir(0, 1, 0) };
    in.start_x_mm = 24.0; in.start_y_mm = 60.0; in.start_z_mm = 15.0;
    in.end_x_mm   = 36.0; in.end_y_mm   = 60.0; in.end_z_mm   = 15.0;
    in.axis_dir   = gp_Dir(0, -1, 0);    // into the +Y face
    in.width_mm   = 5.0; in.depth_mm = 4.0;

    auto out = skill::mill_slot::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    const double r = 2.5, L = 12.0;       // |end-start| = 12
    const double approxRem = (M_PI * r * r + L * 5.0) * 4.0;
    const double removed = volumeOf(stock->shape()) - volumeOf(out.workpiece->shape());
    EXPECT_NEAR(removed, approxRem, approxRem * 0.12)
        << "side obround must remove ~ a stadium prism into the -Y face";
}

// The side port is recognised geometrically with the correct axis + 3-D ends.
TEST(SkillMillSlot, SideObroundRecognized)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 30.0);
    skill::mill_slot::Input in;
    in.entry_face = skill::FaceByNormal{ gp_Dir(0, 1, 0) };
    in.start_x_mm = 24.0; in.start_y_mm = 60.0; in.start_z_mm = 15.0;
    in.end_x_mm   = 36.0; in.end_y_mm   = 60.0; in.end_z_mm   = 15.0;
    in.axis_dir   = gp_Dir(0, -1, 0);
    in.width_mm   = 5.0; in.depth_mm = 4.0;
    auto built = skill::mill_slot::apply(*stock, in);

    skill::Workpiece foreign(built.workpiece->shape());   // NO history
    auto cands = skill::mill_slot::recognize(foreign);
    ASSERT_FALSE(cands.empty()) << "a side obround must be recognised";
    const auto& p = cands[0].recovered_params;
    EXPECT_NEAR(p.value("width_mm", 0.0), 5.0, 0.2);
    EXPECT_NEAR(p.value("depth_mm", 0.0), 4.0, 0.3);
    // axis_dir must be the -Y machining direction, not the legacy -Z.
    ASSERT_TRUE(p.contains("axis_dir") && p["axis_dir"].is_array());
    EXPECT_NEAR(p["axis_dir"][1].get<double>(), -1.0, 0.05)
        << "recovered axis must point into the +Y face (-Y)";
    // ends carry their real Z (mid-height ~15), not 0.
    EXPECT_NEAR(p.value("start_z_mm", -1.0), 15.0, 0.5);
}

// Round-trip: recover the side port, regenerate it, get the same volume.
TEST(SkillMillSlot, SideObroundRoundTrip)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 30.0);
    skill::mill_slot::Input in;
    in.entry_face = skill::FaceByNormal{ gp_Dir(0, 1, 0) };
    in.start_x_mm = 22.0; in.start_y_mm = 60.0; in.start_z_mm = 16.0;
    in.end_x_mm   = 38.0; in.end_y_mm   = 60.0; in.end_z_mm   = 16.0;
    in.axis_dir   = gp_Dir(0, -1, 0);
    in.width_mm   = 6.0; in.depth_mm = 5.0;
    auto built = skill::mill_slot::apply(*stock, in);
    const double vOrig = volumeOf(built.workpiece->shape());

    skill::Workpiece foreign(built.workpiece->shape());
    auto cands = skill::mill_slot::recognize(foreign);
    ASSERT_FALSE(cands.empty());
    const auto& p = cands[0].recovered_params;

    skill::mill_slot::Input in2;
    in2.entry_face = skill::FaceByNormal{ gp_Dir(0, 1, 0) };
    in2.start_x_mm = p["start_x_mm"].get<double>();
    in2.start_y_mm = p["start_y_mm"].get<double>();
    in2.start_z_mm = p["start_z_mm"].get<double>();
    in2.end_x_mm   = p["end_x_mm"].get<double>();
    in2.end_y_mm   = p["end_y_mm"].get<double>();
    in2.end_z_mm   = p["end_z_mm"].get<double>();
    in2.axis_dir   = gp_Dir(p["axis_dir"][0].get<double>(),
                            p["axis_dir"][1].get<double>(),
                            p["axis_dir"][2].get<double>());
    in2.width_mm   = p["width_mm"].get<double>();
    in2.depth_mm   = p["depth_mm"].get<double>();
    auto fresh = skill::createCuboidStock(60.0, 60.0, 30.0);
    auto out2 = skill::mill_slot::apply(*fresh, in2);
    EXPECT_NEAR(volumeOf(out2.workpiece->shape()), vOrig, vOrig * 0.02)
        << "the regenerated side port reproduces the original geometry";
}
