// @lat: [[process/test-strategy#compound-step-roundtrip]]
//
// Roundtrip recognition smoke for 8 compound mechanical-structure skills.
// Writes the synthesized workpiece to STEP, reads it back (which strips
// FeatureSignature metadata), then calls recognize() — must return ≥ 1
// candidate via the geometric_fallback() path.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"

#include "skills/i_beam_compound_section.hpp"
#include "skills/box_section_with_endplate.hpp"
#include "skills/gusset_plate_compound.hpp"
#include "skills/linear_rail_seat_compound.hpp"
#include "skills/ball_screw_nut_pocket.hpp"
#include "skills/threaded_npt_port.hpp"
#include "skills/coil_spring_seat_compound.hpp"
#include "skills/wave_spring_groove.hpp"

#include "io/StepIO.hpp"

#include <gp_Ax1.hxx>

#include <filesystem>

namespace fs = std::filesystem;
using namespace koocadcam;

namespace {
skill::Workpiece roundtripViaStep(const TopoDS_Shape& s, const std::string& name)
{
    const fs::path stepPath = fs::temp_directory_path() / (name + "_rt.step");
    std::string err;
    [&] {
        ASSERT_TRUE(io::StepIO::write(s, stepPath, err)) << err;
    }();
    auto reimp = io::StepIO::read(stepPath, err);
    [&] {
        ASSERT_TRUE(reimp.has_value()) << err;
    }();
    return skill::Workpiece(*reimp);
}
}  // namespace

TEST(CompoundStepRT, IBeam)
{
    auto stock = skill::createCuboidStock(100.0, 100.0, 100.0);
    skill::i_beam_compound_section::Input in;
    in.length_mm   = 1000.0;
    in.height_mm   = 200.0;
    in.flange_w_mm = 150.0;
    in.flange_t_mm = 10.0;
    in.web_t_mm    = 6.0;
    auto out = skill::i_beam_compound_section::apply(*stock, in);
    auto reim = roundtripViaStep(out.workpiece->shape(), "ibeam");
    auto cands = skill::i_beam_compound_section::recognize(reim);
    EXPECT_FALSE(cands.empty());
}

TEST(CompoundStepRT, BoxSection)
{
    auto stock = skill::createCuboidStock(100.0, 100.0, 100.0);
    skill::box_section_with_endplate::Input in;
    in.length_mm     = 600.0;
    in.width_mm      = 100.0;
    in.height_mm     = 80.0;
    in.wall_t_mm     = 5.0;
    in.endplate_t_mm = 10.0;
    in.bolt_grid     = { 2, 2 };
    in.bolt_dia_mm   = 12.0;
    in.edge_dist_mm  = 25.0;
    auto out = skill::box_section_with_endplate::apply(*stock, in);
    auto reim = roundtripViaStep(out.workpiece->shape(), "box_section");
    auto cands = skill::box_section_with_endplate::recognize(reim);
    EXPECT_FALSE(cands.empty());
}

TEST(CompoundStepRT, GussetPlate)
{
    auto stock = skill::createCuboidStock(100.0, 100.0, 100.0);
    skill::gusset_plate_compound::Input in;
    in.leg_a_mm         = 400.0;
    in.leg_b_mm         = 400.0;
    in.plate_t_mm       = 12.0;
    in.lift_hole_dia_mm = 25.0;
    in.bevel_mm         = 5.0;
    in.notch_r_mm       = 8.0;
    auto out = skill::gusset_plate_compound::apply(*stock, in);
    auto reim = roundtripViaStep(out.workpiece->shape(), "gusset");
    auto cands = skill::gusset_plate_compound::recognize(reim);
    EXPECT_FALSE(cands.empty());
}

TEST(CompoundStepRT, LinearRailSeat)
{
    auto stock = skill::createCuboidStock(120.0, 40.0, 12.0);
    skill::linear_rail_seat_compound::Input in;
    in.entry_face         = skill::FaceByNormal{ gp_Dir(0, 0, 1), 5.0, "largest" };
    in.length_mm          = 100.0;
    in.rail_centerline_mm = 20.0;
    in.pitch_mm           = 30.0;
    in.start_x_mm         = 10.0;
    auto out = skill::linear_rail_seat_compound::apply(*stock, in);
    auto reim = roundtripViaStep(out.workpiece->shape(), "linear_rail");
    auto cands = skill::linear_rail_seat_compound::recognize(reim);
    EXPECT_FALSE(cands.empty());
}

TEST(CompoundStepRT, BallScrewNut)
{
    auto stock = skill::createCuboidStock(80.0, 80.0, 50.0);
    skill::ball_screw_nut_pocket::Input in;
    in.entry_face          = skill::FaceByNormal{ gp_Dir(0, 0, 1), 5.0, "largest" };
    in.position_x_mm       = 40.0;
    in.position_y_mm       = 40.0;
    in.axis_dir            = gp_Dir(0, 0, -1);
    in.nut_od_mm           = 25.0;
    in.nut_l_mm            = 20.0;
    in.mounting_pattern    = "square4";
    auto out = skill::ball_screw_nut_pocket::apply(*stock, in);
    auto reim = roundtripViaStep(out.workpiece->shape(), "ball_screw");
    auto cands = skill::ball_screw_nut_pocket::recognize(reim);
    EXPECT_FALSE(cands.empty());
}

TEST(CompoundStepRT, ThreadedNpt)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 20.0);
    skill::threaded_npt_port::Input in;
    in.face_id           = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm     = 25.0;
    in.position_y_mm     = 25.0;
    in.axis_dir          = gp_Dir(0, 0, -1);
    in.npt_size          = "1/4";
    in.recess_depth_mm   = 1.5;
    in.chamfer_leg_mm    = 0.5;
    in.part_thickness_mm = 20.0;
    auto out = skill::threaded_npt_port::apply(*stock, in);
    auto reim = roundtripViaStep(out.workpiece->shape(), "npt");
    auto cands = skill::threaded_npt_port::recognize(reim);
    EXPECT_FALSE(cands.empty());
}

TEST(CompoundStepRT, CoilSpringSeat)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 40.0);
    skill::coil_spring_seat_compound::Input in;
    in.face_id        = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm  = 30.0;
    in.position_y_mm  = 30.0;
    in.axis_dir       = gp_Dir(0, 0, -1);
    in.spring_od      = 12.0;
    in.free_height    = 20.0;
    in.pilot_dia      = 4.0;
    in.slip_fit_mm    = 0.2;
    auto out = skill::coil_spring_seat_compound::apply(*stock, in);
    auto reim = roundtripViaStep(out.workpiece->shape(), "coil_seat");
    auto cands = skill::coil_spring_seat_compound::recognize(reim);
    EXPECT_FALSE(cands.empty());
}

TEST(CompoundStepRT, WaveSpringGroove)
{
    auto stock = skill::createCylindricalStock(40.0, 30.0);
    skill::wave_spring_groove::Input in;
    in.bore_axis          = gp_Ax1(gp_Pnt(0.0, 0.0, 0.0), gp_Dir(0.0, 0.0, 1.0));
    in.mean_dia           = 30.0;
    in.wave_spring_id     = 28.0;
    in.wave_spring_height = 2.0;
    in.axial_position_mm  = 15.0;
    auto out = skill::wave_spring_groove::apply(*stock, in);
    auto reim = roundtripViaStep(out.workpiece->shape(), "wave_groove");
    auto cands = skill::wave_spring_groove::recognize(reim);
    EXPECT_FALSE(cands.empty());
}
