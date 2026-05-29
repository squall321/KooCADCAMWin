// @lat: [[process/test-strategy#compound macro]]
//
// check_valve_seat_with_stop — 4 sub-feature compound, API 594.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/check_valve_seat_with_stop.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using namespace koocadcam;

namespace {
double volumeOf(const TopoDS_Shape& s) {
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}
}  // namespace

// ─── T1: apply produces non-null shape with expected volume delta ────────
TEST(SkillCheckValveSeatWithStop, ApplyProducesValidVolumeDelta)
{
    // Stock: 60 × 60 × 60 mm; center the body in stock.
    auto stock = skill::createCuboidStock(60.0, 60.0, 60.0);
    skill::check_valve_seat_with_stop::Input in;
    in.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1), 5.0, "largest" };
    in.center_x_mm  = 30.0;
    in.center_y_mm  = 30.0;
    in.center_z_mm  = 30.0;
    in.inlet_dia_mm     = 30.0;
    in.inlet_length_mm  = 22.0;
    in.seat_angle_deg   = 45.0;
    in.seat_drop_mm     = 6.0;
    in.outlet_dia_mm    = 18.0;
    in.outlet_length_mm = 22.0;
    in.stop_dia_mm      = 8.0;
    in.stop_length_mm   = 6.0;
    in.stop_offset_mm   = 4.0;

    const double v0 = volumeOf(stock->shape());
    auto out = skill::check_valve_seat_with_stop::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    const double v1 = volumeOf(out.workpiece->shape());

    // Expected (mm³):
    //   inletBore: π × 15² × 30 = 21206 (xInletEnd=30, xMin=0)
    //   coneSeat:  (45°) seatLen = 6 / tan45 = 6 → vol ≈ π × 6/3 × (225+135+81) = 2771
    //   outletBore: π × 9² × 24 (60 − 36) = 6107
    //   stop boss (subtracted): π × 4² × 6 = 301
    const double approx = 21206 + 2771 + 6107 - 301;
    const double diff = v0 - v1;
    EXPECT_GT(diff, 0.0);
    EXPECT_NEAR(diff, approx, approx * 0.25)
        << "removed=" << diff << " expected~" << approx;
}

// ─── T2: validate rejects bad seat angle + apply throws ──────────────────
TEST(SkillCheckValveSeatWithStop, ValidateRejectsBadSeatAngle)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 60.0);
    skill::check_valve_seat_with_stop::Input in;
    in.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.center_x_mm  = 30.0;
    in.center_y_mm  = 30.0;
    in.center_z_mm  = 30.0;
    in.inlet_dia_mm     = 30.0;
    in.inlet_length_mm  = 22.0;
    in.seat_angle_deg   = 75.0;   // > 60° — outside API 594 range
    in.seat_drop_mm     = 6.0;
    in.outlet_dia_mm    = 18.0;
    in.outlet_length_mm = 22.0;
    in.stop_dia_mm      = 8.0;
    in.stop_length_mm   = 6.0;
    in.stop_offset_mm   = 4.0;

    auto r = skill::check_valve_seat_with_stop::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-CHECK-VALVE-SEAT-ANGLE") { found = true; break; }
    EXPECT_TRUE(found);
    EXPECT_THROW(skill::check_valve_seat_with_stop::apply(*stock, in),
                 skill::SkillError);
}

// ─── T3: signature carries compound info ─────────────────────────────────
TEST(SkillCheckValveSeatWithStop, SignatureRecordsCompound)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 60.0);
    skill::check_valve_seat_with_stop::Input in;
    in.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.center_x_mm  = 30.0;
    in.center_y_mm  = 30.0;
    in.center_z_mm  = 30.0;
    in.inlet_dia_mm     = 30.0;
    in.inlet_length_mm  = 22.0;
    in.seat_angle_deg   = 45.0;
    in.seat_drop_mm     = 6.0;
    in.outlet_dia_mm    = 18.0;
    in.outlet_length_mm = 22.0;
    in.stop_dia_mm      = 8.0;
    in.stop_length_mm   = 6.0;
    in.stop_offset_mm   = 4.0;

    auto out = skill::check_valve_seat_with_stop::apply(*stock, in);
    EXPECT_EQ(out.signature.skill_id, std::string("check_valve_seat_with_stop"));
    EXPECT_TRUE(out.signature.pattern.at("is_compound").get<bool>());
    EXPECT_EQ(out.signature.pattern.at("subfeature_count").get<int>(), 4);
    EXPECT_NEAR(out.signature.pattern.at("seat_angle_deg").get<double>(),
                45.0, 1e-6);
    EXPECT_TRUE(out.signature.pattern.at("has_disc_stop").get<bool>());
}

// ─── T4: recognize returns ≥1 candidate via metadata replay ──────────────
TEST(SkillCheckValveSeatWithStop, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 60.0);
    skill::check_valve_seat_with_stop::Input in;
    in.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.center_x_mm  = 30.0;
    in.center_y_mm  = 30.0;
    in.center_z_mm  = 30.0;
    in.inlet_dia_mm     = 30.0;
    in.inlet_length_mm  = 22.0;
    in.seat_angle_deg   = 45.0;
    in.seat_drop_mm     = 6.0;
    in.outlet_dia_mm    = 18.0;
    in.outlet_length_mm = 22.0;
    in.stop_dia_mm      = 8.0;
    in.stop_length_mm   = 6.0;
    in.stop_offset_mm   = 4.0;

    auto out = skill::check_valve_seat_with_stop::apply(*stock, in);
    auto cands = skill::check_valve_seat_with_stop::recognize(*out.workpiece);

    ASSERT_GE(cands.size(), 1u);
    EXPECT_EQ(cands.front().skill_id, std::string("check_valve_seat_with_stop"));
    EXPECT_NEAR(cands.front().confidence, 1.0, 1e-9);
    EXPECT_NEAR(cands.front().recovered_params.at("seat_angle_deg")
                .get<double>(), 45.0, 1e-6);
}

// ─── T5: inlet dia ≥ outlet dia — anti-throttle property of an NRV ───────
TEST(SkillCheckValveSeatWithStop, InletDiameterExceedsOutletDiameter)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 60.0);
    skill::check_valve_seat_with_stop::Input in;
    in.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.center_x_mm  = 30.0;
    in.center_y_mm  = 30.0;
    in.center_z_mm  = 30.0;
    in.inlet_dia_mm     = 30.0;
    in.inlet_length_mm  = 22.0;
    in.seat_angle_deg   = 45.0;
    in.seat_drop_mm     = 6.0;
    in.outlet_dia_mm    = 18.0;     // strictly smaller than inlet
    in.outlet_length_mm = 22.0;
    in.stop_dia_mm      = 8.0;
    in.stop_length_mm   = 6.0;
    in.stop_offset_mm   = 4.0;

    auto out = skill::check_valve_seat_with_stop::apply(*stock, in);
    const double inDia  = out.signature.params.at("inlet_dia_mm").get<double>();
    const double outDia = out.signature.params.at("outlet_dia_mm").get<double>();
    EXPECT_GT(inDia, outDia);

    // Try inverted geometry — must be rejected by DFM.
    in.inlet_dia_mm  = 18.0;
    in.outlet_dia_mm = 30.0;
    auto r = skill::check_valve_seat_with_stop::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool flowFlagFound = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-CHECK-VALVE-FLOW") { flowFlagFound = true; break; }
    EXPECT_TRUE(flowFlagFound);
}
