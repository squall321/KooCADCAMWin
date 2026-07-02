// @lat: [[process/test-strategy#skill round-trip]]

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/mill_rect_pocket.hpp"

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

TEST(SkillMillRectPocket, ApplyCreatesPocket)
{
    auto stock = skill::createCuboidStock(60.0, 40.0, 10.0);

    skill::mill_rect_pocket::Input in;
    in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.center_x_mm = 30.0; in.center_y_mm = 20.0;
    in.length_mm = 20.0; in.width_mm = 10.0; in.depth_mm = 3.0;
    in.corner_r_mm = 1.0;

    auto out = skill::mill_rect_pocket::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    const double approx = 20.0 * 10.0 * 3.0;
    const double removed = volumeOf(stock->shape()) - volumeOf(out.workpiece->shape());
    // Corners are rounded → removed slightly less than the box approximation.
    EXPECT_NEAR(removed, approx, approx * 0.10);
}

TEST(SkillMillRectPocket, ValidateRejectsLargeCornerR)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);
    skill::mill_rect_pocket::Input in;
    in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.length_mm = 8.0; in.width_mm = 6.0; in.depth_mm = 2.0;
    in.corner_r_mm = 5.0;  // >= width/2 → collapse

    auto r = skill::mill_rect_pocket::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_THROW(skill::mill_rect_pocket::apply(*stock, in), skill::SkillError);
}

TEST(SkillMillRectPocket, RecognizeFinds)
{
    auto stock = skill::createCuboidStock(60.0, 40.0, 10.0);
    skill::mill_rect_pocket::Input in;
    in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.center_x_mm = 30.0; in.center_y_mm = 20.0;
    in.length_mm = 20.0; in.width_mm = 10.0; in.depth_mm = 3.0;
    in.corner_r_mm = 1.0;

    auto out = skill::mill_rect_pocket::apply(*stock, in);
    auto cands = skill::mill_rect_pocket::recognize(*out.workpiece);
    ASSERT_FALSE(cands.empty());
    EXPECT_NEAR(cands[0].recovered_params["corner_r_mm"].get<double>(), 1.0, 0.1);
    EXPECT_NEAR(cands[0].recovered_params["depth_mm"].get<double>(),     3.0, 0.1);
    // The entry-plane Z must be recovered (top face Z=10) so CAM machines from the
    // real surface, not Z=0.  (Previously omitted → a recovered pocket cut at Z=0.)
    ASSERT_TRUE(cands[0].recovered_params.contains("center_z_mm"));
    EXPECT_NEAR(cands[0].recovered_params["center_z_mm"].get<double>(), 10.0, 0.1)
        << "recovered rect-pocket entry plane must be the top face (10), not Z=0";
}

// Rewritten recognize() now collapses STEP-roundtrip-split corner-fillet
// cylindrical sub-faces via axis-XY clustering, and groups corner clusters
// into pockets by shared (zLo, zHi) Z-range — so the 4 corner fillets are
// reliably re-grouped regardless of how STEP roundtrip decomposed them.
TEST(SkillMillRectPocket, RoundTripViaStep)
{
    auto stock = skill::createCuboidStock(60.0, 40.0, 10.0);
    skill::mill_rect_pocket::Input in;
    in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.center_x_mm = 25.0; in.center_y_mm = 15.0;
    in.length_mm = 14.0; in.width_mm = 8.0; in.depth_mm = 2.0;
    in.corner_r_mm = 1.0;

    auto synth = skill::mill_rect_pocket::apply(*stock, in);

    const fs::path stepPath = fs::temp_directory_path() / "mill_rect_pkt_rt.step";
    std::string err;
    ASSERT_TRUE(io::StepIO::write(synth.workpiece->shape(), stepPath, err));
    auto reimp = io::StepIO::read(stepPath, err);
    ASSERT_TRUE(reimp.has_value());
    skill::Workpiece reim(*reimp);

    auto cands = skill::mill_rect_pocket::recognize(reim);
    ASSERT_FALSE(cands.empty());
    EXPECT_NEAR(cands[0].recovered_params["depth_mm"].get<double>(), 2.0, 0.1);
}

TEST(SkillMillRectPocket, SharpCornersZeroR)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);
    skill::mill_rect_pocket::Input in;
    in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.center_x_mm = 25.0; in.center_y_mm = 25.0;
    in.length_mm = 15.0; in.width_mm = 8.0; in.depth_mm = 2.0;
    in.corner_r_mm = 0.0;  // sharp corners — valid for end-mill?  In reality
                           // end-mills always leave a corner-R = tool radius,
                           // but the synthesis accepts 0 as a documented choice.

    auto out = skill::mill_rect_pocket::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
}
