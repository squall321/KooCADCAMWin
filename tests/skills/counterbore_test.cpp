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

#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include "io/StepIO.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>
#include <gp_Ax2.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>

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

// ─── 6. GEOMETRIC path: measured dims from a FRESH workpiece (top entry) ───
//
// A FRESH Workpiece built straight from the cut shape has NO feature history,
// so recognize() is forced down the pure GEOMETRIC path (no metadata replay).
// We assert the recovered seat/pilot diameters + entry position match the
// authored values to tight tolerances measured off BRepAdaptor surfaces.
TEST(SkillCounterbore, GeometricRecoversMeasuredDimsTopEntry)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::counterbore::Input in;
    in.entry_face     = skill::FaceByNormal{ gp_Dir(0, 0, 1), 5.0, "largest" };
    in.position_x_mm  = 20.0;
    in.position_y_mm  = 28.0;
    in.axis_dir       = gp_Dir(0, 0, -1);
    in.pilot_dia_mm   = 3.0;
    in.pilot_depth_mm = 8.0;
    in.seat_dia_mm    = 7.0;
    in.seat_depth_mm  = 2.5;

    auto synth = skill::counterbore::apply(*stock, in);

    // FRESH workpiece — drops the FeatureSignature history, forcing geometry.
    skill::Workpiece fresh(synth.workpiece->shape());
    EXPECT_TRUE(fresh.features().empty()) << "fresh wp must have no feature history";

    auto candidates = skill::counterbore::recognize(fresh);
    ASSERT_EQ(candidates.size(), 1u) << "geometric path should find one counterbore";
    const auto& c = candidates.front();

    EXPECT_NEAR(c.recovered_params["seat_dia_mm"].get<double>(),  in.seat_dia_mm,  0.1);
    EXPECT_NEAR(c.recovered_params["pilot_dia_mm"].get<double>(), in.pilot_dia_mm, 0.1);
    EXPECT_NEAR(c.recovered_params["seat_depth_mm"].get<double>(),  in.seat_depth_mm,  0.1);
    EXPECT_NEAR(c.recovered_params["pilot_depth_mm"].get<double>(), in.pilot_depth_mm, 0.1);
    EXPECT_NEAR(c.recovered_params["position_x_mm"].get<double>(), in.position_x_mm, 0.2);
    EXPECT_NEAR(c.recovered_params["position_y_mm"].get<double>(), in.position_y_mm, 0.2);

    // Entry is the TOP face → z ≈ 10 (stock top), drill axis points -Z.
    EXPECT_NEAR(c.recovered_params["position_z_mm"].get<double>(), 10.0, 0.2);
    EXPECT_LT(c.recovered_params["axis_dir"][2].get<double>(), -0.9)
        << "top-entry counterbore drills downward (-Z)";
}

// ─── 7. AXIS-INDEPENDENCE: counterbore from the BOTTOM face ────────────────
//
// Identical bore, but entered from the -Z (bottom) face with the drill axis
// pointing +Z.  A +Z-only recognizer would mis-identify entry vs step here.
// Proves the adjacency-based entry pick is orientation independent.
TEST(SkillCounterbore, GeometricRecoversMeasuredDimsBottomEntry)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::counterbore::Input in;
    in.entry_face     = skill::FaceByNormal{ gp_Dir(0, 0, -1), 5.0, "largest" };
    in.position_x_mm  = 30.0;
    in.position_y_mm  = 18.0;
    in.axis_dir       = gp_Dir(0, 0, 1);     // drill UPWARD into the block
    in.pilot_dia_mm   = 4.0;
    in.pilot_depth_mm = 7.0;
    in.seat_dia_mm    = 9.0;
    in.seat_depth_mm  = 2.0;

    auto synth = skill::counterbore::apply(*stock, in);

    skill::Workpiece fresh(synth.workpiece->shape());
    EXPECT_TRUE(fresh.features().empty());

    auto candidates = skill::counterbore::recognize(fresh);
    ASSERT_EQ(candidates.size(), 1u)
        << "bottom-entry counterbore must still be recognized (axis-independent)";
    const auto& c = candidates.front();

    EXPECT_NEAR(c.recovered_params["seat_dia_mm"].get<double>(),  in.seat_dia_mm,  0.1);
    EXPECT_NEAR(c.recovered_params["pilot_dia_mm"].get<double>(), in.pilot_dia_mm, 0.1);
    EXPECT_NEAR(c.recovered_params["seat_depth_mm"].get<double>(),  in.seat_depth_mm,  0.1);
    EXPECT_NEAR(c.recovered_params["pilot_depth_mm"].get<double>(), in.pilot_depth_mm, 0.1);
    EXPECT_NEAR(c.recovered_params["position_x_mm"].get<double>(), in.position_x_mm, 0.2);
    EXPECT_NEAR(c.recovered_params["position_y_mm"].get<double>(), in.position_y_mm, 0.2);

    // Entry is the BOTTOM face → z ≈ 0, drill axis points +Z.
    EXPECT_NEAR(c.recovered_params["position_z_mm"].get<double>(), 0.0, 0.2);
    EXPECT_GT(c.recovered_params["axis_dir"][2].get<double>(), 0.9)
        << "bottom-entry counterbore drills upward (+Z)";
}

// ─── 8. REJECTION: a convex stepped SHAFT is NOT a counterbore ─────────────
//
// Mirrors drill_hole's RejectsConvexBossAsHole.  Two coaxial cylinders of
// different radii FUSED into a positive solid (a stepped boss/shaft) present
// two coaxial cylindrical faces — but they are CONVEX (solid inside).  The
// concavity gate must reject them so a shaft is never reported as a bore.
TEST(SkillCounterbore, RejectsConvexSteppedShaftAsCounterbore)
{
    namespace pr = koocadcam::engine::prim;

    // Large short cylinder (would-be "seat") + small tall cylinder (would-be
    // "pilot"), coaxial along +Z, fused into one convex solid.
    const gp_Ax2 ax(gp_Pnt(0.0, 0.0, 0.0), gp_Dir(0.0, 0.0, 1.0));
    const TopoDS_Shape big   = pr::cylinder(ax, 6.0, 3.0);   // r=6, h=3
    const TopoDS_Shape small = pr::cylinder(ax, 3.0, 12.0);  // r=3, h=12
    const TopoDS_Shape shaft = pr::fuse(big, small);

    skill::Workpiece wp(shaft);
    auto candidates = skill::counterbore::recognize(wp);
    EXPECT_TRUE(candidates.empty())
        << "a convex stepped shaft must NOT be recognized as a counterbore";
}

// ─── Through-counterbore recovery (resolves the AS1 0-recall question) ─────
//
// A counterbore whose pilot goes THROUGH the plate (no blind floor). The
// hardened recognizer must still recover it geometrically; if it cannot, the
// hardening lost through-counterbore recall (the AS1 regression risk).
TEST(SkillCounterbore, RecognizesThroughCounterbore)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);
    skill::counterbore::Input in;
    in.entry_face    = skill::FaceByNormal{ gp_Dir(0,0,1) };
    in.position_x_mm = 25.0; in.position_y_mm = 25.0;
    in.axis_dir      = gp_Dir(0,0,-1);
    in.pilot_dia_mm  = 4.0; in.pilot_depth_mm = 12.0;   // through the 10mm plate
    in.seat_dia_mm   = 9.0; in.seat_depth_mm  = 3.0;
    auto out = skill::counterbore::apply(*stock, in);

    skill::Workpiece fresh(out.workpiece->shape());
    auto cands = skill::counterbore::recognize(fresh);

    bool found = false;
    for (const auto& c : cands) {
        const auto& p = c.recovered_params;
        if (std::abs(p.value("seat_dia_mm", 0.0)  - 9.0) < 0.2 &&
            std::abs(p.value("pilot_dia_mm", 0.0) - 4.0) < 0.2) { found = true; break; }
    }
    EXPECT_TRUE(found)
        << "hardened counterbore must still recover a THROUGH counterbore "
           "(seat 9 + pilot 4) — else AS1's counterbore recall was lost";
}
