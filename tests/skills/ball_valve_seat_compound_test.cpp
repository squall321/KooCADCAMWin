// @lat: [[process/test-strategy#compound macro]]
//
// ball_valve_seat_compound — 6 sub-feature compound, spec-driven (API 608).

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/ball_valve_seat_compound.hpp"

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
TEST(SkillBallValveSeatCompound, ApplyProducesValidVolumeDelta)
{
    auto stock = skill::createCuboidStock(80.0, 60.0, 60.0);
    skill::ball_valve_seat_compound::Input in;
    in.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1), 5.0, "largest" };
    in.center_x_mm  = 40.0;
    in.center_y_mm  = 30.0;
    in.center_z_mm  = 30.0;
    in.size_spec    = "DN25";   // ball=32, port=20, seat 21..31×4, stem 12×22, flange 44×5

    const double v0 = volumeOf(stock->shape());
    auto out = skill::ball_valve_seat_compound::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    const double v1 = volumeOf(out.workpiece->shape());

    // Expected: sphere + through-bore + 2×annular groove + stem + flange.
    // Dominant terms (mm³):
    //   sphere  = (4/3)π × (16.1)³ ≈ 17481  (ball=32, clearance=0.10 → R=16.1)
    //   bore    = π × 10² × 80     ≈ 25133
    //   seat×2  = 2 × π × ((15.5)² − (10.5)²) × 4 ≈ 3267
    //   stem    = π × 6² × 22     ≈ 2488
    //   flange  = π × 22² × 5     ≈ 7603
    // Some overlap (sphere/bore concentric).  Use a wide ±25% band.
    const double approx = 17481 + 25133 + 3267 + 2488 + 7603 - 5000;  // ~ -5000 overlap
    const double diff = v0 - v1;
    EXPECT_GT(diff, 0.0);
    EXPECT_NEAR(diff, approx, approx * 0.25)
        << "removed=" << diff << " expected~" << approx;
}

// ─── T2: validate rejects unknown spec key + apply throws ───────────────
TEST(SkillBallValveSeatCompound, ValidateRejectsUnknownSpec)
{
    auto stock = skill::createCuboidStock(80.0, 60.0, 60.0);
    skill::ball_valve_seat_compound::Input in;
    in.entry_face  = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.center_x_mm = 40.0;
    in.center_y_mm = 30.0;
    in.center_z_mm = 30.0;
    in.size_spec   = "DN999";   // unknown

    auto r = skill::ball_valve_seat_compound::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-BALL-VALVE-SIZE") { found = true; break; }
    EXPECT_TRUE(found);
    EXPECT_THROW(skill::ball_valve_seat_compound::apply(*stock, in),
                 skill::SkillError);
}

// ─── T3: signature carries spec key + is_compound=true ───────────────────
TEST(SkillBallValveSeatCompound, SignatureRecordsCompoundAndSpec)
{
    auto stock = skill::createCuboidStock(80.0, 60.0, 60.0);
    skill::ball_valve_seat_compound::Input in;
    in.entry_face  = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.center_x_mm = 40.0;
    in.center_y_mm = 30.0;
    in.center_z_mm = 30.0;
    in.size_spec   = "DN20";

    auto out = skill::ball_valve_seat_compound::apply(*stock, in);
    EXPECT_EQ(out.signature.skill_id, std::string("ball_valve_seat_compound"));
    EXPECT_EQ(out.signature.pattern.at("kind"),
              std::string("ball_valve_seat_compound"));
    EXPECT_TRUE(out.signature.pattern.at("is_compound").get<bool>());
    EXPECT_EQ(out.signature.pattern.at("subfeature_count").get<int>(), 6);
    EXPECT_EQ(out.signature.pattern.at("size_spec"), std::string("DN20"));
    EXPECT_EQ(out.signature.params.at("size_spec"), std::string("DN20"));
}

// ─── T4: recognize returns ≥1 candidate via metadata replay ──────────────
TEST(SkillBallValveSeatCompound, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(80.0, 60.0, 60.0);
    skill::ball_valve_seat_compound::Input in;
    in.entry_face  = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.center_x_mm = 40.0;
    in.center_y_mm = 30.0;
    in.center_z_mm = 30.0;
    in.size_spec   = "DN32";

    auto out = skill::ball_valve_seat_compound::apply(*stock, in);
    auto cands = skill::ball_valve_seat_compound::recognize(*out.workpiece);

    ASSERT_GE(cands.size(), 1u);
    EXPECT_EQ(cands.front().skill_id, std::string("ball_valve_seat_compound"));
    EXPECT_NEAR(cands.front().confidence, 1.0, 1e-9);
    EXPECT_EQ(cands.front().recovered_params.at("size_spec"),
              std::string("DN32"));
}

// ─── T5: spec-table values applied — DN50 → port=40, ball=60 ─────────────
TEST(SkillBallValveSeatCompound, SpecTableValuesDriveDimensions)
{
    auto stock = skill::createCuboidStock(100.0, 90.0, 90.0);
    skill::ball_valve_seat_compound::Input in;
    in.entry_face  = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.center_x_mm = 50.0;
    in.center_y_mm = 45.0;
    in.center_z_mm = 45.0;
    in.size_spec   = "DN50";   // per table: ball=60, port=40

    auto out = skill::ball_valve_seat_compound::apply(*stock, in);
    EXPECT_NEAR(out.signature.params.at("ball_dia_mm").get<double>(),
                60.0, 1e-6);
    EXPECT_NEAR(out.signature.params.at("port_dia_mm").get<double>(),
                40.0, 1e-6);
    // Ball must be > port.
    EXPECT_GT(out.signature.params.at("ball_dia_mm").get<double>(),
              out.signature.params.at("port_dia_mm").get<double>());
}
