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

#include "engine/primitives/Tools.hpp"
#include "engine/primitives/Cuts.hpp"

#include <BRepBndLib.hxx>
#include <BRepGProp.hxx>
#include <Bnd_Box.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>
#include <gp_Ax2.hxx>

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

// ─── 6. Entry face recovered from ADJACENCY, not Z order ──────────────────
//
// Two blind holes, one drilled from the TOP (entry z=20) and one from the
// BOTTOM (entry z=0).  The entry point must be the ring opening onto the large
// workpiece surface — independent of the cylinder-axis orientation OCCT picks.
// The old "highest-Z circle = entry" heuristic reported the bottom hole's entry
// at z=6 (its blind floor); the adjacency rule recovers z=0.
TEST(SkillDrillHole, EntryFaceFromAdjacencyNotZOrder)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 20.0);

    skill::drill_hole::Input a;
    a.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    a.position_x_mm = 15.0; a.position_y_mm = 15.0;
    a.axis_dir = gp_Dir(0, 0, -1);
    a.diameter_mm = 4.0; a.depth_mm = 6.0; a.through_hole = false;
    auto wA = skill::drill_hole::apply(*stock, a);

    skill::drill_hole::Input b;
    b.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, -1) };
    b.position_x_mm = 35.0; b.position_y_mm = 35.0;
    b.axis_dir = gp_Dir(0, 0, 1);
    b.diameter_mm = 4.0; b.depth_mm = 6.0; b.through_hole = false;
    auto wB = skill::drill_hole::apply(*wA.workpiece, b);

    auto cands = skill::drill_hole::recognize(*wB.workpiece);
    bool gotTop = false, gotBottom = false;
    for (const auto& c : cands) {
        const double x = c.recovered_params.value("position_x_mm", 0.0);
        const double y = c.recovered_params.value("position_y_mm", 0.0);
        const double z = c.recovered_params.value("position_z_mm", -999.0);
        const bool blind = !c.recovered_params.value("through_hole", true);
        if (std::abs(x - 15.0) < 0.5 && std::abs(y - 15.0) < 0.5) {
            gotTop = true; EXPECT_NEAR(z, 20.0, 0.1); EXPECT_TRUE(blind);
        }
        if (std::abs(x - 35.0) < 0.5 && std::abs(y - 35.0) < 0.5) {
            gotBottom = true; EXPECT_NEAR(z, 0.0, 0.1); EXPECT_TRUE(blind);
        }
    }
    EXPECT_TRUE(gotTop)    << "top-entry blind hole not recovered";
    EXPECT_TRUE(gotBottom) << "bottom-entry blind hole not recovered at z=0";
}

// ─── 7. Machinability gate: a convex BOSS (rod) is NOT a hole ──────────────
//
// drill_hole::recognize must reject convex cylinders. Drill a real hole at
// (15,15) and fuse a cylindrical boss at (35,35); only the concave hole should
// be recovered — the boss (material inside the cylinder) must not.
TEST(SkillDrillHole, RejectsConvexBossAsHole)
{
    namespace pr = koocadcam::engine::prim;
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    // A real drilled hole at (15,15).
    skill::drill_hole::Input h;
    h.entry_face   = skill::FaceByNormal{ gp_Dir(0,0,1) };
    h.position_x_mm = 15.0; h.position_y_mm = 15.0;
    h.axis_dir = gp_Dir(0,0,-1); h.diameter_mm = 4.0; h.depth_mm = 6.0; h.through_hole = false;
    auto holed = skill::drill_hole::apply(*stock, h);

    // A convex cylindrical boss standing up from the top face at (35,35).
    const TopoDS_Shape boss = pr::cylinder(
        gp_Ax2(gp_Pnt(35.0, 35.0, 10.0), gp_Dir(0,0,1)), 5.0, 8.0);
    const TopoDS_Shape withBoss = pr::fuse(holed.workpiece->shape(), boss);
    skill::Workpiece wp(withBoss);

    auto cands = skill::drill_hole::recognize(wp);
    int atHole = 0, atBoss = 0;
    for (const auto& c : cands) {
        const double x = c.recovered_params.value("position_x_mm", 0.0);
        const double y = c.recovered_params.value("position_y_mm", 0.0);
        if (std::abs(x-15.0) < 1.0 && std::abs(y-15.0) < 1.0) ++atHole;
        if (std::abs(x-35.0) < 1.0 && std::abs(y-35.0) < 1.0) ++atBoss;
    }
    EXPECT_GT(atHole, 0) << "the real concave hole must still be recovered";
    EXPECT_EQ(atBoss, 0) << "the convex boss must NOT be recognized as a hole";
}
