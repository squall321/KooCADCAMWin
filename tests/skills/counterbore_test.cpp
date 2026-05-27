// @lat: [[process/test-strategy#skill round-trip]]
//
// counterbore skill — full round-trip verification mirroring drill_hole_test:
//   1. Stock cuboid 50 × 50 × 10 mm aluminum.
//   2. SYNTHESIS: apply counterbore(pilot 3×8, seat 6×3) → workpiece with
//      pilot + seat coaxial bores.
//   3. EXPORT to STEP, then RE-IMPORT (forgets all metadata).
//   4. ANALYSIS: counterbore::recognize() should find 1 candidate.
//   5. VERIFY recovered params within tolerance + volume equality.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/counterbore.hpp"

#include "io/StepIO.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

#include <cmath>
#include <filesystem>
#include <vector>

namespace fs = std::filesystem;
using namespace koocadcam;

namespace {

double volumeOf(const TopoDS_Shape& s)
{
    GProp_GProps p;
    BRepGProp::VolumeProperties(s, p);
    return p.Mass();
}

}  // namespace

// ─── 1. Basic synthesis works ─────────────────────────────────────────────
TEST(SkillCounterbore, ApplyCreatesValidCounterbore)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);
    ASSERT_FALSE(stock->shape().IsNull());

    skill::counterbore::Input in;
    in.entry_face     = skill::FaceByNormal{ gp_Dir(0, 0, 1), 5.0, "largest" };
    in.position_x_mm  = 25.0;
    in.position_y_mm  = 25.0;
    in.axis_dir       = gp_Dir(0, 0, -1);
    in.pilot_dia_mm   = 3.0;
    in.pilot_depth_mm = 8.0;
    in.seat_dia_mm    = 6.0;
    in.seat_depth_mm  = 3.0;

    auto out = skill::counterbore::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    // Expected volume removed:
    //   pilot cylinder (full depth) + seat annulus (seat_depth × annulus area)
    const double rPilot = in.pilot_dia_mm / 2.0;
    const double rSeat  = in.seat_dia_mm  / 2.0;
    const double approx =
        M_PI * rPilot * rPilot * in.pilot_depth_mm +
        M_PI * (rSeat * rSeat - rPilot * rPilot) * in.seat_depth_mm;
    const double diff = volumeOf(stock->shape()) - volumeOf(out.workpiece->shape());
    EXPECT_NEAR(diff, approx, approx * 0.05)
        << "counterbore should remove pilot column + seat annulus";

    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_EQ(out.signature.skill_id, std::string("counterbore"));
}

// ─── 2. DFM rejects bad geometry (seat dia ≤ pilot dia) ───────────────────
TEST(SkillCounterbore, ValidateRejectsBadGeometry)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    // Sub-min pilot diameter (DFM-002)
    {
        skill::counterbore::Input in;
        in.entry_face     = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
        in.position_x_mm  = 25.0;
        in.position_y_mm  = 25.0;
        in.pilot_dia_mm   = 0.5;      // violates DFM-002
        in.pilot_depth_mm = 6.0;
        in.seat_dia_mm    = 4.0;
        in.seat_depth_mm  = 2.0;

        auto r = skill::counterbore::validate(*stock, in);
        EXPECT_FALSE(r.passed);
        bool found = false;
        for (const auto& f : r.findings)
            if (f.code == "DFM-002") { found = true; break; }
        EXPECT_TRUE(found);
        EXPECT_THROW(skill::counterbore::apply(*stock, in), skill::SkillError);
    }

    // seat_dia ≤ pilot_dia (custom counterbore geom rule)
    {
        skill::counterbore::Input in;
        in.entry_face     = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
        in.position_x_mm  = 25.0;
        in.position_y_mm  = 25.0;
        in.pilot_dia_mm   = 4.0;
        in.pilot_depth_mm = 6.0;
        in.seat_dia_mm    = 4.0;       // equal to pilot — not a counterbore
        in.seat_depth_mm  = 2.0;

        auto r = skill::counterbore::validate(*stock, in);
        EXPECT_FALSE(r.passed);
        EXPECT_THROW(skill::counterbore::apply(*stock, in), skill::SkillError);
    }
}

// ─── 3. Recognize finds the counterbore ───────────────────────────────────
TEST(SkillCounterbore, RecognizeFindsCounterbore)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::counterbore::Input in;
    in.entry_face     = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm  = 25.0;
    in.position_y_mm  = 25.0;
    in.pilot_dia_mm   = 3.0;
    in.pilot_depth_mm = 8.0;
    in.seat_dia_mm    = 6.0;
    in.seat_depth_mm  = 3.0;

    auto out = skill::counterbore::apply(*stock, in);
    auto candidates = skill::counterbore::recognize(*out.workpiece);

    ASSERT_EQ(candidates.size(), 1u) << "expected exactly one counterbore candidate";
    const auto& c = candidates.front();
    EXPECT_EQ(c.skill_id, std::string("counterbore"));
    EXPECT_GT(c.confidence, 0.8);

    EXPECT_NEAR(c.recovered_params["pilot_dia_mm"].get<double>(),   3.0, 1e-3);
    EXPECT_NEAR(c.recovered_params["seat_dia_mm"].get<double>(),    6.0, 1e-3);
    EXPECT_NEAR(c.recovered_params["pilot_depth_mm"].get<double>(), 8.0, 1e-2);
    EXPECT_NEAR(c.recovered_params["seat_depth_mm"].get<double>(),  3.0, 1e-2);
    EXPECT_NEAR(c.recovered_params["position_x_mm"].get<double>(), 25.0, 1e-3);
    EXPECT_NEAR(c.recovered_params["position_y_mm"].get<double>(), 25.0, 1e-3);
}

// ─── 4. Full round-trip via STEP file ─────────────────────────────────────
TEST(SkillCounterbore, RoundTripViaStep)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::counterbore::Input in;
    in.entry_face     = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm  = 30.0;
    in.position_y_mm  = 15.0;
    in.pilot_dia_mm   = 4.0;
    in.pilot_depth_mm = 7.5;
    in.seat_dia_mm    = 8.0;
    in.seat_depth_mm  = 2.5;

    auto synth = skill::counterbore::apply(*stock, in);

    // Export → reimport
    const fs::path stepPath = fs::temp_directory_path() / "counterbore_roundtrip.step";
    std::string err;
    ASSERT_TRUE(io::StepIO::write(synth.workpiece->shape(), stepPath, err)) << err;

    auto reimportedOpt = io::StepIO::read(stepPath, err);
    ASSERT_TRUE(reimportedOpt.has_value()) << err;
    skill::Workpiece reim(*reimportedOpt);

    auto candidates = skill::counterbore::recognize(reim);
    ASSERT_EQ(candidates.size(), 1u)
        << "STEP-imported shape should still expose the counterbore pattern";
    const auto& c = candidates.front();

    EXPECT_NEAR(c.recovered_params["pilot_dia_mm"].get<double>(),   in.pilot_dia_mm,   1e-2);
    EXPECT_NEAR(c.recovered_params["seat_dia_mm"].get<double>(),    in.seat_dia_mm,    1e-2);
    EXPECT_NEAR(c.recovered_params["pilot_depth_mm"].get<double>(), in.pilot_depth_mm, 5e-2);
    EXPECT_NEAR(c.recovered_params["seat_depth_mm"].get<double>(),  in.seat_depth_mm,  5e-2);
    EXPECT_NEAR(c.recovered_params["position_x_mm"].get<double>(),  in.position_x_mm,  1e-2);
    EXPECT_NEAR(c.recovered_params["position_y_mm"].get<double>(),  in.position_y_mm,  1e-2);

    // Topology hash: re-synthesis from recovered params should produce
    // identical face/edge/vertex counts and volume.
    skill::counterbore::Input recovered = in;
    recovered.pilot_dia_mm   = c.recovered_params["pilot_dia_mm"].get<double>();
    recovered.seat_dia_mm    = c.recovered_params["seat_dia_mm"].get<double>();
    recovered.pilot_depth_mm = c.recovered_params["pilot_depth_mm"].get<double>();
    recovered.seat_depth_mm  = c.recovered_params["seat_depth_mm"].get<double>();
    recovered.position_x_mm  = c.recovered_params["position_x_mm"].get<double>();
    recovered.position_y_mm  = c.recovered_params["position_y_mm"].get<double>();

    auto stock2 = skill::createCuboidStock(50.0, 50.0, 10.0);
    auto resynth = skill::counterbore::apply(*stock2, recovered);

    EXPECT_EQ(synth.workpiece->faceCount(),   resynth.workpiece->faceCount());
    EXPECT_EQ(synth.workpiece->edgeCount(),   resynth.workpiece->edgeCount());
    EXPECT_EQ(synth.workpiece->vertexCount(), resynth.workpiece->vertexCount());

    EXPECT_NEAR(volumeOf(synth.workpiece->shape()),
                volumeOf(resynth.workpiece->shape()),
                1e-2);
}

// ─── 5. Edge case: seat_depth ≥ pilot_depth must be rejected ──────────────
TEST(SkillCounterbore, RejectsSeatDeeperThanPilot)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::counterbore::Input in;
    in.entry_face     = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm  = 25.0;
    in.position_y_mm  = 25.0;
    in.pilot_dia_mm   = 3.0;
    in.pilot_depth_mm = 3.0;
    in.seat_dia_mm    = 6.0;
    in.seat_depth_mm  = 4.0;          // seat deeper than pilot — invalid

    auto r = skill::counterbore::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_THROW(skill::counterbore::apply(*stock, in), skill::SkillError);
}
