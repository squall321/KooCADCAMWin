// @lat: [[process/test-strategy#skill round-trip]]
//
// drill_hole skill — full round-trip verification per slice 1 spec:
//   1. Stock cuboid 50 × 50 × 10 mm aluminum.
//   2. SYNTHESIS: apply drill_hole(dia=3, depth=6) → workpiece with hole.
//   3. EXPORT to STEP, then RE-IMPORT (forgets all metadata).
//   4. ANALYSIS: drill_hole::recognize() should find 1 candidate.
//   5. VERIFY:
//      (a) recovered parameters within tolerance of input.
//      (b) topology hash: re-synthesis from recovered params gives the
//          same face/edge/vertex counts as the original.
//      (c) volume equality within ε.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/drill_hole.hpp"

#include "io/StepIO.hpp"

#include <BRepBndLib.hxx>
#include <BRepGProp.hxx>
#include <Bnd_Box.hxx>
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
TEST(SkillDrillHole, ApplyCreatesValidHole)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);
    ASSERT_FALSE(stock->shape().IsNull());

    skill::drill_hole::Input in;
    in.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1), 5.0, "largest" };
    in.position_x_mm = 25.0;
    in.position_y_mm = 25.0;
    in.axis_dir      = gp_Dir(0, 0, -1);
    in.diameter_mm   = 3.0;
    in.depth_mm      = 6.0;
    in.through_hole  = false;

    auto out = skill::drill_hole::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    // Volume must decrease by ~ π × r² × depth.
    const double approx = M_PI * 1.5 * 1.5 * 6.0;
    const double diff = volumeOf(stock->shape()) - volumeOf(out.workpiece->shape());
    EXPECT_NEAR(diff, approx, approx * 0.05) << "blind drill should remove ~π r² depth";

    // Feature history was recorded.
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_EQ(out.signature.skill_id, std::string("drill_hole"));
}

// ─── 2. DFM-002 catches sub-0.8 mm diameter ───────────────────────────────
TEST(SkillDrillHole, ValidateRejectsSubMinHole)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::drill_hole::Input in;
    in.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 25.0;
    in.position_y_mm = 25.0;
    in.diameter_mm   = 0.5;     // violates DFM-002
    in.depth_mm      = 3.0;

    auto r = skill::drill_hole::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-002") { found = true; break; }
    EXPECT_TRUE(found);

    // apply() must also throw.
    EXPECT_THROW(skill::drill_hole::apply(*stock, in), skill::SkillError);
}

// ─── 3. Recognize finds the hole ──────────────────────────────────────────
TEST(SkillDrillHole, RecognizeFindsHole)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::drill_hole::Input in;
    in.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 25.0;
    in.position_y_mm = 25.0;
    in.diameter_mm   = 3.0;
    in.depth_mm      = 6.0;

    auto out = skill::drill_hole::apply(*stock, in);
    auto candidates = skill::drill_hole::recognize(*out.workpiece);

    ASSERT_EQ(candidates.size(), 1u) << "expected exactly one drill_hole candidate";
    const auto& c = candidates.front();
    EXPECT_EQ(c.skill_id, std::string("drill_hole"));
    EXPECT_GT(c.confidence, 0.8);

    EXPECT_NEAR(c.recovered_params["diameter_mm"].get<double>(), 3.0, 1e-3);
    EXPECT_NEAR(c.recovered_params["depth_mm"].get<double>(),    6.0, 1e-3);
    EXPECT_NEAR(c.recovered_params["position_x_mm"].get<double>(), 25.0, 1e-3);
    EXPECT_NEAR(c.recovered_params["position_y_mm"].get<double>(), 25.0, 1e-3);
    EXPECT_FALSE(c.recovered_params["through_hole"].get<bool>());
}

// ─── 4. Full round-trip via STEP file (the headline test) ─────────────────
TEST(SkillDrillHole, RoundTripViaStep)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::drill_hole::Input in;
    in.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 30.0;
    in.position_y_mm = 15.0;
    in.diameter_mm   = 4.0;
    in.depth_mm      = 5.5;

    auto synth = skill::drill_hole::apply(*stock, in);

    // Export → reimport
    const fs::path stepPath = fs::temp_directory_path() / "drill_hole_roundtrip.step";
    std::string err;
    ASSERT_TRUE(io::StepIO::write(synth.workpiece->shape(), stepPath, err)) << err;

    auto reimportedOpt = io::StepIO::read(stepPath, err);
    ASSERT_TRUE(reimportedOpt.has_value()) << err;
    skill::Workpiece reim(*reimportedOpt);

    // Recognize on the re-imported (metadata-free) shape
    auto candidates = skill::drill_hole::recognize(reim);
    ASSERT_EQ(candidates.size(), 1u) << "STEP-imported shape should still expose the drill_hole pattern";
    const auto& c = candidates.front();

    EXPECT_NEAR(c.recovered_params["diameter_mm"].get<double>(), in.diameter_mm, 1e-2);
    EXPECT_NEAR(c.recovered_params["depth_mm"].get<double>(),    in.depth_mm,    1e-2);
    EXPECT_NEAR(c.recovered_params["position_x_mm"].get<double>(), in.position_x_mm, 1e-2);
    EXPECT_NEAR(c.recovered_params["position_y_mm"].get<double>(), in.position_y_mm, 1e-2);

    // Topology hash: re-synthesis from recovered params should produce
    // identical face/edge counts and volume.
    skill::drill_hole::Input recovered = in;
    recovered.diameter_mm   = c.recovered_params["diameter_mm"].get<double>();
    recovered.depth_mm      = c.recovered_params["depth_mm"].get<double>();
    recovered.position_x_mm = c.recovered_params["position_x_mm"].get<double>();
    recovered.position_y_mm = c.recovered_params["position_y_mm"].get<double>();

    auto stock2 = skill::createCuboidStock(50.0, 50.0, 10.0);
    auto resynth = skill::drill_hole::apply(*stock2, recovered);

    EXPECT_EQ(synth.workpiece->faceCount(),   resynth.workpiece->faceCount());
    EXPECT_EQ(synth.workpiece->edgeCount(),   resynth.workpiece->edgeCount());
    EXPECT_EQ(synth.workpiece->vertexCount(), resynth.workpiece->vertexCount());

    EXPECT_NEAR(volumeOf(synth.workpiece->shape()),
                volumeOf(resynth.workpiece->shape()),
                1e-3);
}

// ─── 5. Through-hole variant ──────────────────────────────────────────────
TEST(SkillDrillHole, ThroughHoleProducesNoBottomFace)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::drill_hole::Input in;
    in.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 25.0;
    in.position_y_mm = 25.0;
    in.diameter_mm   = 3.0;
    in.through_hole  = true;

    auto out = skill::drill_hole::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    auto cands = skill::drill_hole::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_TRUE(cands.front().recovered_params["through_hole"].get<bool>());
}
