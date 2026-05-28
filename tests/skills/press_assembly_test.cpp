// @lat: [[process/test-strategy#skill round-trip]]
//
// press_assembly — assembly-level interference fit through-hole.
//
// Cases (6):
//   1. Apply cuts an undersized through-hole = pin + interference (signed µm).
//   2. holeDiameterFor helper.
//   3. DFM rejects positive interference (= slip fit).
//   4. DFM rejects missing press_force_N.
//   5. DFM rejects pin diameter outside [1.5, 50] mm.
//   6. Recognize replays history.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/press_assembly.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

#include <cmath>

using namespace koocadcam;

namespace {

double volumeOf(const TopoDS_Shape& s)
{
    GProp_GProps p;
    BRepGProp::VolumeProperties(s, p);
    return p.Mass();
}

}  // namespace

// ─── 1. Apply cuts an undersized through-hole ────────────────────────────
TEST(SkillPressAssembly, ApplyMakesUndersizedThroughHole)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);
    const double volBefore = volumeOf(stock->shape());

    skill::press_assembly::Input in;
    in.entry_face       = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm    = 25.0;
    in.position_y_mm    = 25.0;
    in.axis_dir         = gp_Dir(0, 0, -1);
    in.pin_diameter_mm  = 6.0;
    in.interference_um  = -30.0;     // hole 5.970 mm
    in.press_force_N    = 1500.0;
    in.partner_part     = "dowel";

    auto out = skill::press_assembly::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    const double holeDia = 6.0 - 0.030;
    const double expected = M_PI * (holeDia / 2.0) * (holeDia / 2.0) * 10.0;
    const double removed  = volBefore - volumeOf(out.workpiece->shape());
    EXPECT_NEAR(removed, expected, expected * 0.05);

    EXPECT_EQ(out.signature.skill_id, std::string("press_assembly"));
    EXPECT_NEAR(out.signature.params["hole_dia_mm"].get<double>(), holeDia, 1e-6);
    EXPECT_NEAR(out.signature.params["pin_diameter_mm"].get<double>(),  6.0, 1e-9);
    EXPECT_NEAR(out.signature.params["interference_um"].get<double>(), -30.0, 1e-9);
    EXPECT_NEAR(out.signature.params["press_force_N"].get<double>(), 1500.0, 1e-9);
    EXPECT_EQ(out.signature.params["partner_part"].get<std::string>(), std::string("dowel"));
    EXPECT_TRUE(out.signature.params["through_hole"].get<bool>());
}

// ─── 2. holeDiameterFor helper ───────────────────────────────────────────
TEST(SkillPressAssembly, HoleDiameterFormula)
{
    EXPECT_NEAR(skill::press_assembly::holeDiameterFor(5.0,  -25.0), 4.975, 1e-9);
    EXPECT_NEAR(skill::press_assembly::holeDiameterFor(10.0, -50.0), 9.950, 1e-9);
    EXPECT_NEAR(skill::press_assembly::holeDiameterFor(3.0,  -10.0), 2.990, 1e-9);
}

// ─── 3. DFM rejects positive interference ────────────────────────────────
TEST(SkillPressAssembly, ValidateRejectsLooseFit)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::press_assembly::Input in;
    in.entry_face       = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm    = 25.0;
    in.position_y_mm    = 25.0;
    in.pin_diameter_mm  = 5.0;
    in.interference_um  = +20.0;        // slip fit
    in.press_force_N    = 1000.0;

    auto r = skill::press_assembly::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool sawCode = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-PA-INT") { sawCode = true; break; }
    EXPECT_TRUE(sawCode);
    EXPECT_THROW(skill::press_assembly::apply(*stock, in), skill::SkillError);
}

// ─── 4. DFM rejects missing press_force_N ────────────────────────────────
TEST(SkillPressAssembly, ValidateRejectsZeroForce)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::press_assembly::Input in;
    in.entry_face       = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm    = 25.0;
    in.position_y_mm    = 25.0;
    in.pin_diameter_mm  = 5.0;
    in.interference_um  = -25.0;
    in.press_force_N    = 0.0;          // invalid

    auto r = skill::press_assembly::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool sawCode = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-PA-FORCE") { sawCode = true; break; }
    EXPECT_TRUE(sawCode);
}

// ─── 5. DFM rejects out-of-range pin diameter ────────────────────────────
TEST(SkillPressAssembly, ValidateRejectsTinyPin)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::press_assembly::Input in;
    in.entry_face       = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm    = 25.0;
    in.position_y_mm    = 25.0;
    in.pin_diameter_mm  = 1.0;          // < 1.5
    in.interference_um  = -20.0;
    in.press_force_N    = 200.0;

    auto r = skill::press_assembly::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool sawCode = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-PA-PIN") { sawCode = true; break; }
    EXPECT_TRUE(sawCode);
}

// ─── 6. Recognize replays history ────────────────────────────────────────
TEST(SkillPressAssembly, RecognizeReplaysFromHistory)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::press_assembly::Input in;
    in.entry_face       = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm    = 20.0;
    in.position_y_mm    = 30.0;
    in.pin_diameter_mm  = 8.0;
    in.interference_um  = -40.0;
    in.press_force_N    = 3500.0;
    in.partner_part     = "shaft";

    auto out = skill::press_assembly::apply(*stock, in);
    auto cands = skill::press_assembly::recognize(*out.workpiece);

    ASSERT_FALSE(cands.empty());
    const auto& c = cands.front();
    EXPECT_NEAR(c.confidence, 1.0, 1e-9);
    EXPECT_NEAR(c.recovered_params["pin_diameter_mm"].get<double>(),  8.0, 1e-6);
    EXPECT_NEAR(c.recovered_params["interference_um"].get<double>(), -40.0, 1e-6);
    EXPECT_NEAR(c.recovered_params["hole_dia_mm"].get<double>(),      8.0 - 0.040, 1e-6);
    EXPECT_NEAR(c.recovered_params["press_force_N"].get<double>(),    3500.0, 1e-6);
    EXPECT_EQ(c.recovered_params["partner_part"].get<std::string>(), std::string("shaft"));
}
