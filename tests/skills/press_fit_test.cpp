// @lat: [[process/test-strategy#skill round-trip]]
//
// press_fit skill — verifies under-sized hole synthesis, signed-interference
// conversion, DFM bounds, and metadata-driven recognition.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/press_fit.hpp"

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

// ─── 1. Apply drills an UNDER-sized cylindrical hole ──────────────────────
TEST(SkillPressFit, ApplyUndersizesHole)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::press_fit::Input in;
    in.entry_face         = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm      = 25.0;
    in.position_y_mm      = 25.0;
    in.axis_dir           = gp_Dir(0, 0, -1);
    in.nominal_pin_dia_mm = 5.0;
    in.interference_um    = -25.0;       // medium press fit
    in.depth_mm           = 6.0;
    in.through_hole       = false;

    auto out = skill::press_fit::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    // Hole diameter = 5.0 + (-25) × 0.001 = 4.975 mm
    const double expectedHoleDia = 5.0 - 0.025;
    const double approx = M_PI *
        (expectedHoleDia / 2.0) * (expectedHoleDia / 2.0) * 6.0;
    const double removed = volumeOf(stock->shape()) - volumeOf(out.workpiece->shape());
    EXPECT_NEAR(removed, approx, approx * 0.05)
        << "press_fit must cut an undersized cylinder (pin - interference)";

    EXPECT_EQ(out.signature.skill_id, std::string("press_fit"));
    EXPECT_EQ(out.signature.tooling.tool_type, std::string("reamer"));
    EXPECT_NEAR(out.signature.params["hole_dia_mm"].get<double>(), expectedHoleDia, 1e-6);
    EXPECT_NEAR(out.signature.params["nominal_pin_dia_mm"].get<double>(), 5.0, 1e-6);
    EXPECT_NEAR(out.signature.params["interference_um"].get<double>(), -25.0, 1e-6);
}

// ─── 2. holeDiameterFor helper: signed interference math ─────────────────
TEST(SkillPressFit, HoleDiameterFormula)
{
    // -25 µm on a 5 mm pin → 4.975 mm hole
    EXPECT_NEAR(skill::press_fit::holeDiameterFor(5.0,  -25.0), 4.975, 1e-9);
    EXPECT_NEAR(skill::press_fit::holeDiameterFor(10.0, -50.0), 9.950, 1e-9);
    EXPECT_NEAR(skill::press_fit::holeDiameterFor(3.0,  -10.0), 2.990, 1e-9);
    // Lower bound (-5 µm) still legal numerically
    EXPECT_NEAR(skill::press_fit::holeDiameterFor(2.0,  -5.0),  1.995, 1e-9);
}

// ─── 3. DFM rejects positive interference (= clearance fit) ───────────────
TEST(SkillPressFit, ValidateRejectsLooseFit)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::press_fit::Input in;
    in.entry_face         = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm      = 25.0;
    in.position_y_mm      = 25.0;
    in.nominal_pin_dia_mm = 5.0;
    in.interference_um    = +20.0;       // hole BIGGER than pin → slip fit
    in.depth_mm           = 5.0;

    auto r = skill::press_fit::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-PF-INT") { found = true; break; }
    EXPECT_TRUE(found);
    EXPECT_THROW(skill::press_fit::apply(*stock, in), skill::SkillError);
}

// ─── 4. DFM rejects out-of-range pin diameter ─────────────────────────────
TEST(SkillPressFit, ValidateRejectsTinyPin)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::press_fit::Input in;
    in.entry_face         = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm      = 25.0;
    in.position_y_mm      = 25.0;
    in.nominal_pin_dia_mm = 0.5;         // below 1.5 mm minimum
    in.interference_um    = -25.0;
    in.depth_mm           = 3.0;

    auto r = skill::press_fit::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-PF-PIN") { found = true; break; }
    EXPECT_TRUE(found);
}

// ─── 5. Recognize replays from feature history ────────────────────────────
TEST(SkillPressFit, RecognizeReplaysFromHistory)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::press_fit::Input in;
    in.entry_face         = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm      = 20.0;
    in.position_y_mm      = 30.0;
    in.nominal_pin_dia_mm = 6.0;
    in.interference_um    = -30.0;
    in.depth_mm           = 5.0;

    auto out = skill::press_fit::apply(*stock, in);
    auto cands = skill::press_fit::recognize(*out.workpiece);

    ASSERT_FALSE(cands.empty());
    const auto& c = cands.front();
    EXPECT_GT(c.confidence, 0.9);                       // history replay → high
    EXPECT_NEAR(c.recovered_params["nominal_pin_dia_mm"].get<double>(), 6.0, 1e-6);
    EXPECT_NEAR(c.recovered_params["interference_um"].get<double>(),   -30.0, 1e-6);
    EXPECT_NEAR(c.recovered_params["hole_dia_mm"].get<double>(),        6.0 - 0.030, 1e-6);
    EXPECT_EQ  (c.recovered_params["through_hole"].get<bool>(),         false);
}

// ─── 6. Through-hole press fit ────────────────────────────────────────────
TEST(SkillPressFit, ThroughHoleVariant)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 8.0);

    skill::press_fit::Input in;
    in.entry_face         = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm      = 25.0;
    in.position_y_mm      = 25.0;
    in.nominal_pin_dia_mm = 4.0;
    in.interference_um    = -20.0;
    in.through_hole       = true;

    auto out = skill::press_fit::apply(*stock, in);
    EXPECT_TRUE(out.signature.params["through_hole"].get<bool>());

    auto cands = skill::press_fit::recognize(*out.workpiece);
    ASSERT_FALSE(cands.empty());
    EXPECT_TRUE(cands.front().recovered_params["through_hole"].get<bool>());
}
