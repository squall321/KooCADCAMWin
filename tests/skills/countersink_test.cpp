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

// ─── 6. MEASURED-dimension recovery from a FRESH workpiece ─────────────────
//
// This forces the GEOMETRIC fallback (a fresh Workpiece built from the cut
// shape has NO feature history, so the metadata fast-path is skipped).  The
// recovered MAJOR (rim) diameter and the INCLUDED cone angle must be MEASURED
// off the OCCT cone/cylinder surfaces, not replayed from metadata.
TEST(SkillCountersink, MeasuresDimensionsFromFreshWorkpiece)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 12.0);

    skill::countersink::Input in;
    in.entry_face      = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm   = 18.0;
    in.position_y_mm   = 32.0;
    in.axis_dir        = gp_Dir(0, 0, -1);
    in.pilot_dia_mm    = 5.0;
    in.pilot_depth_mm  = 9.0;
    in.cone_top_dia_mm = 10.0;
    in.cone_angle_deg  = 100.0;            // SAE/AS aerospace flat-head
    auto synth = skill::countersink::apply(*stock, in);

    // FRESH workpiece from the resulting SHAPE only → no feature history,
    // forcing recognize() down the geometric path.
    skill::Workpiece fresh(synth.workpiece->shape());
    ASSERT_TRUE(fresh.features().empty())
        << "fresh workpiece must carry no feature history";

    auto candidates = skill::countersink::recognize(fresh);
    ASSERT_EQ(candidates.size(), 1u)
        << "geometric path should recover exactly one countersink";
    const auto& c = candidates.front();
    EXPECT_GT(c.confidence, 0.8);

    // MEASURED major (rim) diameter ± 0.1 mm.
    EXPECT_NEAR(c.recovered_params["cone_top_dia_mm"].get<double>(),
                in.cone_top_dia_mm, 0.1);
    // MEASURED pilot diameter ± 0.1 mm.
    EXPECT_NEAR(c.recovered_params["pilot_dia_mm"].get<double>(),
                in.pilot_dia_mm, 0.1);
    // MEASURED included cone angle ± 1 deg (from Cone().SemiAngle()).
    EXPECT_NEAR(c.recovered_params["cone_angle_deg"].get<double>(),
                in.cone_angle_deg, 1.0);
    // Recovered 3-D position ± 0.2 mm.
    EXPECT_NEAR(c.recovered_params["position_x_mm"].get<double>(),
                in.position_x_mm, 0.2);
    EXPECT_NEAR(c.recovered_params["position_y_mm"].get<double>(),
                in.position_y_mm, 0.2);
    ASSERT_TRUE(c.recovered_params.contains("position_z_mm"));
    EXPECT_NEAR(c.recovered_params["position_z_mm"].get<double>(),
                12.0, 0.2);                // entry on the +Z top face
}

// ─── 7. Axis-independence: countersink machined along the X axis ───────────
//
// The recovery must NOT assume a Z axis.  We build the feature DIRECTLY with
// primitives (a fused pilot cylinder + cone frustum cut into the +X face of a
// block) so the test exercises recognize()'s axis-independence without relying
// on apply()'s tool placement.  A FRESH workpiece forces the geometric path.
TEST(SkillCountersink, MeasuresCountersinkOnXAxis)
{
    namespace pr = koocadcam::engine::prim;

    auto stock = skill::createCuboidStock(40.0, 40.0, 40.0);

    const double pilotR   = 2.0;           // pilot_dia 4 mm
    const double coneTopR = 4.0;           // cone_top_dia 8 mm
    const double pilotDepth = 9.0;
    // 90° included → 45° half → cone_depth = (4 − 2)/tan(45°) = 2 mm.
    const double coneDepth  = (coneTopR - pilotR);  // tan45 = 1
    const double entryX     = 40.0;        // +X face
    const gp_Dir intoX(-1, 0, 0);          // into the material

    // Pilot cylinder: starts just outside +X, runs in −X for pilotDepth (+ a
    // little overhang for a clean entry cut).
    const gp_Ax2 pilotAx(gp_Pnt(entryX + 0.1, 12.0, 28.0), intoX);
    const TopoDS_Shape pilot = pr::cylinder(pilotAx, pilotR, pilotDepth + 0.1);

    // Cone frustum: large rim (coneTopR) AT the entry plane, taper to pilotR
    // at +coneDepth into the material.
    const gp_Ax2 coneAx(gp_Pnt(entryX, 12.0, 28.0), intoX);
    const TopoDS_Shape cone = pr::coneFrustum(coneAx, coneTopR, pilotR, coneDepth);

    const TopoDS_Shape cut = pr::cut(stock->shape(), pr::fuse(pilot, cone));

    skill::Workpiece fresh(cut);
    auto candidates = skill::countersink::recognize(fresh);
    ASSERT_EQ(candidates.size(), 1u)
        << "X-axis countersink must still be recovered (axis-independent)";
    const auto& c = candidates.front();

    EXPECT_NEAR(c.recovered_params["cone_top_dia_mm"].get<double>(),
                2.0 * coneTopR, 0.1);
    EXPECT_NEAR(c.recovered_params["pilot_dia_mm"].get<double>(),
                2.0 * pilotR, 0.1);
    EXPECT_NEAR(c.recovered_params["cone_angle_deg"].get<double>(),
                90.0, 1.0);

    // Recovered entry position: on the +X face, at (Y, Z) = (12, 28).
    EXPECT_NEAR(c.recovered_params["position_x_mm"].get<double>(), entryX, 0.2);
    EXPECT_NEAR(c.recovered_params["position_y_mm"].get<double>(), 12.0, 0.2);
    ASSERT_TRUE(c.recovered_params.contains("position_z_mm"));
    EXPECT_NEAR(c.recovered_params["position_z_mm"].get<double>(), 28.0, 0.2);

    // Axis direction must be parallel to ±X (Y and Z components ≈ 0).
    auto ad = c.recovered_params["axis_dir"];
    EXPECT_NEAR(std::abs(ad[0].get<double>()), 1.0, 1e-3);
    EXPECT_NEAR(ad[1].get<double>(), 0.0, 1e-3);
    EXPECT_NEAR(ad[2].get<double>(), 0.0, 1e-3);
}

// ─── 8. Rejection: a convex pointed cone boss is NOT a countersink ─────────
//
// Mirrors drill_hole's RejectsConvexBossAsHole.  A protruding spike (a CONVEX
// cone fused onto a block, sitting on a coaxial cylindrical rod) must never be
// reported as a countersink — the concavity gate has to reject it.
TEST(SkillCountersink, RejectsConvexConeBossAsCountersink)
{
    namespace pr = koocadcam::engine::prim;

    // Base block 40 × 40 × 10, top at Z = 10.
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    // A cylindrical rod (convex) standing on the top face, capped by a CONVEX
    // pointed cone (a spike) — geometrically the "inverse" of a countersink:
    // both surfaces are convex (solid INSIDE), so neither is machinable seat.
    const gp_Pnt base(20.0, 20.0, 10.0);
    const gp_Ax2 upAx(base, gp_Dir(0, 0, 1));

    // Rod: r = 2.5, height 6 (convex cylinder wall).
    const TopoDS_Shape rod = pr::cylinder(upAx, 2.5, 6.0);

    // Cone tip on top of the rod: large radius 2.5 at base, taper to 0.2 at
    // the top over 4 mm → a convex pointed cone (boss/spike).
    const gp_Ax2 coneAx(gp_Pnt(20.0, 20.0, 16.0), gp_Dir(0, 0, 1));
    const TopoDS_Shape spike = pr::coneFrustum(coneAx, 2.5, 0.2, 4.0);

    const TopoDS_Shape bossShape =
        pr::fuse(pr::fuse(stock->shape(), rod), spike);

    skill::Workpiece boss(bossShape);
    auto candidates = skill::countersink::recognize(boss);
    EXPECT_TRUE(candidates.empty())
        << "a convex pointed cone boss must NOT be recognized as a countersink";
}
