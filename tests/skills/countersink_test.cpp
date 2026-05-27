// @lat: [[process/test-strategy#skill round-trip]]
//
// countersink skill — full round-trip verification mirroring drill_hole_test:
//   1. Stock cuboid 50 × 50 × 10 mm aluminum.
//   2. SYNTHESIS: apply countersink(pilot 3×8, cone 6mm@90°) → workpiece
//      with cylindrical pilot + conical seat at entry.
//   3. EXPORT to STEP, then RE-IMPORT (forgets all metadata).
//   4. ANALYSIS: countersink::recognize() should find 1 candidate.
//   5. VERIFY recovered params within tolerance + volume equality.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/countersink.hpp"

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
TEST(SkillCountersink, ApplyCreatesValidCountersink)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);
    ASSERT_FALSE(stock->shape().IsNull());

    skill::countersink::Input in;
    in.entry_face      = skill::FaceByNormal{ gp_Dir(0, 0, 1), 5.0, "largest" };
    in.position_x_mm   = 25.0;
    in.position_y_mm   = 25.0;
    in.axis_dir        = gp_Dir(0, 0, -1);
    in.pilot_dia_mm    = 3.0;
    in.pilot_depth_mm  = 8.0;
    in.cone_top_dia_mm = 6.0;
    in.cone_angle_deg  = 90.0;

    auto out = skill::countersink::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    // Expected volume removed:
    //   cylinder (full pilot depth) +
    //   cone frustum extra-over-cylinder portion
    const double rPilot   = in.pilot_dia_mm    / 2.0;
    const double rConeTop = in.cone_top_dia_mm / 2.0;
    const double coneDepth = skill::countersink::computeConeDepth(in);
    EXPECT_GT(coneDepth, 0.0);

    const double cylVol = M_PI * rPilot * rPilot * in.pilot_depth_mm;
    const double frust  = (M_PI * coneDepth / 3.0) *
                          (rConeTop * rConeTop + rConeTop * rPilot + rPilot * rPilot);
    const double coreInCone = M_PI * rPilot * rPilot * coneDepth;
    const double approx     = cylVol + (frust - coreInCone);

    const double diff = volumeOf(stock->shape()) - volumeOf(out.workpiece->shape());
    EXPECT_NEAR(diff, approx, approx * 0.05)
        << "countersink should remove pilot cylinder + cone-extra volume";

    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_EQ(out.signature.skill_id, std::string("countersink"));
}

// ─── 2. DFM rejects bad geometry ──────────────────────────────────────────
TEST(SkillCountersink, ValidateRejectsBadCountersink)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    // Sub-min pilot diameter (DFM-002)
    {
        skill::countersink::Input in;
        in.entry_face      = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
        in.position_x_mm   = 25.0;
        in.position_y_mm   = 25.0;
        in.pilot_dia_mm    = 0.5;         // violates DFM-002
        in.pilot_depth_mm  = 6.0;
        in.cone_top_dia_mm = 3.0;
        in.cone_angle_deg  = 90.0;

        auto r = skill::countersink::validate(*stock, in);
        EXPECT_FALSE(r.passed);
        bool found = false;
        for (const auto& f : r.findings)
            if (f.code == "DFM-002") { found = true; break; }
        EXPECT_TRUE(found);
        EXPECT_THROW(skill::countersink::apply(*stock, in), skill::SkillError);
    }

    // cone_angle out of range
    {
        skill::countersink::Input in;
        in.entry_face      = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
        in.position_x_mm   = 25.0;
        in.position_y_mm   = 25.0;
        in.pilot_dia_mm    = 3.0;
        in.pilot_depth_mm  = 6.0;
        in.cone_top_dia_mm = 6.0;
        in.cone_angle_deg  = 30.0;        // < 45° lower bound

        auto r = skill::countersink::validate(*stock, in);
        EXPECT_FALSE(r.passed);
        EXPECT_THROW(skill::countersink::apply(*stock, in), skill::SkillError);
    }
}

// ─── 3. Recognize finds the countersink ───────────────────────────────────
TEST(SkillCountersink, RecognizeFindsCountersink)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::countersink::Input in;
    in.entry_face      = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm   = 25.0;
    in.position_y_mm   = 25.0;
    in.pilot_dia_mm    = 3.0;
    in.pilot_depth_mm  = 8.0;
    in.cone_top_dia_mm = 6.0;
    in.cone_angle_deg  = 90.0;

    auto out = skill::countersink::apply(*stock, in);
    auto candidates = skill::countersink::recognize(*out.workpiece);

    ASSERT_EQ(candidates.size(), 1u) << "expected exactly one countersink candidate";
    const auto& c = candidates.front();
    EXPECT_EQ(c.skill_id, std::string("countersink"));
    EXPECT_GT(c.confidence, 0.8);

    EXPECT_NEAR(c.recovered_params["pilot_dia_mm"].get<double>(),    3.0, 1e-3);
    EXPECT_NEAR(c.recovered_params["cone_top_dia_mm"].get<double>(), 6.0, 1e-3);
    EXPECT_NEAR(c.recovered_params["pilot_depth_mm"].get<double>(),  8.0, 1e-2);
    EXPECT_NEAR(c.recovered_params["cone_angle_deg"].get<double>(),  90.0, 0.5);
    EXPECT_NEAR(c.recovered_params["position_x_mm"].get<double>(),  25.0, 1e-3);
    EXPECT_NEAR(c.recovered_params["position_y_mm"].get<double>(),  25.0, 1e-3);
}

// ─── 4. Full round-trip via STEP file ─────────────────────────────────────
TEST(SkillCountersink, RoundTripViaStep)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::countersink::Input in;
    in.entry_face      = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm   = 30.0;
    in.position_y_mm   = 15.0;
    in.pilot_dia_mm    = 4.0;
    in.pilot_depth_mm  = 7.5;
    in.cone_top_dia_mm = 8.0;
    in.cone_angle_deg  = 82.0;             // standard UNC

    auto synth = skill::countersink::apply(*stock, in);

    // Export → reimport
    const fs::path stepPath = fs::temp_directory_path() / "countersink_roundtrip.step";
    std::string err;
    ASSERT_TRUE(io::StepIO::write(synth.workpiece->shape(), stepPath, err)) << err;

    auto reimportedOpt = io::StepIO::read(stepPath, err);
    ASSERT_TRUE(reimportedOpt.has_value()) << err;
    skill::Workpiece reim(*reimportedOpt);

    auto candidates = skill::countersink::recognize(reim);
    ASSERT_EQ(candidates.size(), 1u)
        << "STEP-imported shape should still expose the countersink pattern";
    const auto& c = candidates.front();

    EXPECT_NEAR(c.recovered_params["pilot_dia_mm"].get<double>(),    in.pilot_dia_mm,    1e-2);
    EXPECT_NEAR(c.recovered_params["cone_top_dia_mm"].get<double>(), in.cone_top_dia_mm, 1e-2);
    EXPECT_NEAR(c.recovered_params["pilot_depth_mm"].get<double>(),  in.pilot_depth_mm,  5e-2);
    EXPECT_NEAR(c.recovered_params["cone_angle_deg"].get<double>(),  in.cone_angle_deg,  0.5);
    EXPECT_NEAR(c.recovered_params["position_x_mm"].get<double>(),   in.position_x_mm,   1e-2);
    EXPECT_NEAR(c.recovered_params["position_y_mm"].get<double>(),   in.position_y_mm,   1e-2);

    // Topology hash: re-synthesis from recovered params should produce
    // identical face/edge/vertex counts and volume.
    skill::countersink::Input recovered = in;
    recovered.pilot_dia_mm    = c.recovered_params["pilot_dia_mm"].get<double>();
    recovered.cone_top_dia_mm = c.recovered_params["cone_top_dia_mm"].get<double>();
    recovered.pilot_depth_mm  = c.recovered_params["pilot_depth_mm"].get<double>();
    recovered.cone_angle_deg  = c.recovered_params["cone_angle_deg"].get<double>();
    recovered.position_x_mm   = c.recovered_params["position_x_mm"].get<double>();
    recovered.position_y_mm   = c.recovered_params["position_y_mm"].get<double>();

    auto stock2 = skill::createCuboidStock(50.0, 50.0, 10.0);
    auto resynth = skill::countersink::apply(*stock2, recovered);

    EXPECT_EQ(synth.workpiece->faceCount(),   resynth.workpiece->faceCount());
    EXPECT_EQ(synth.workpiece->edgeCount(),   resynth.workpiece->edgeCount());
    EXPECT_EQ(synth.workpiece->vertexCount(), resynth.workpiece->vertexCount());

    EXPECT_NEAR(volumeOf(synth.workpiece->shape()),
                volumeOf(resynth.workpiece->shape()),
                1e-2);
}

// ─── 5. Edge case: cone deeper than pilot must be rejected ────────────────
TEST(SkillCountersink, RejectsConeDeeperThanPilot)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::countersink::Input in;
    in.entry_face      = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm   = 25.0;
    in.position_y_mm   = 25.0;
    in.pilot_dia_mm    = 2.0;
    in.pilot_depth_mm  = 1.0;             // very shallow pilot
    in.cone_top_dia_mm = 8.0;             // huge cone top → cone_depth >> pilot_depth
    in.cone_angle_deg  = 60.0;

    // cone_depth = (8 − 2) / 2 / tan(30°) = 3 / 0.577 ≈ 5.196 mm > pilot_depth=1 → reject
    const double cd = skill::countersink::computeConeDepth(in);
    EXPECT_GT(cd, in.pilot_depth_mm);

    auto r = skill::countersink::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_THROW(skill::countersink::apply(*stock, in), skill::SkillError);
}
