// @lat: [[process/test-strategy#skill round-trip]]
//
// bore_cylindrical — precision-bore variant of drill_hole.  Same topology,
// different metadata + DFM, distinguished from a drill_hole by a recovered
// diameter ≥ 6 mm and a depth/diameter ratio ≤ 4.
//
// Cases (5):
//   1. apply produces correct volume removal (large dia, low ratio).
//   2. DFM rejects sub-0.8 mm diameter (DFM-002 error) AND warns when dia < 6.
//   3. recognize finds the bore on the synthesized workpiece.
//   4. STEP round-trip: re-recognize after export/import → identical params.
//   5. Edge: a small-diameter (< 6 mm) hole still passes DFM (only warns)
//      and recognize returns reduced confidence < 0.6 (i.e. ambiguous w/ drill).

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/bore_cylindrical.hpp"

#include "io/StepIO.hpp"

#include "engine/primitives/Tools.hpp"
#include "engine/primitives/Cuts.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>
#include <gp_Ax2.hxx>

#include <cmath>
#include <cstdio>
#include <filesystem>

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

// ─── 1. Apply: large precision bore removes π r² depth ────────────────────
TEST(SkillBoreCylindrical, ApplyRemovesExpectedVolume)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 20.0);
    ASSERT_FALSE(stock->shape().IsNull());

    skill::bore_cylindrical::Input in;
    in.entry_face     = skill::FaceByNormal{ gp_Dir(0, 0, 1), 5.0, "largest" };
    in.position_x_mm  = 25.0;
    in.position_y_mm  = 25.0;
    in.axis_dir       = gp_Dir(0, 0, -1);
    in.diameter_mm    = 10.0;
    in.depth_mm       = 12.0;
    in.tolerance_class = "H7";

    auto out = skill::bore_cylindrical::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    const double approx = M_PI * 5.0 * 5.0 * 12.0;
    const double diff = volumeOf(stock->shape()) - volumeOf(out.workpiece->shape());
    EXPECT_NEAR(diff, approx, approx * 0.05);

    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_EQ(out.signature.skill_id, std::string("bore_cylindrical"));
    EXPECT_EQ(out.signature.tooling.tool_type, std::string("boring_bar"));
    EXPECT_NEAR(out.signature.tooling.cutting_speed_sfm, 100.0, 1e-6);
}

// ─── 2. Validate: DFM-002 on sub-0.8 mm, warning on sub-6 mm ──────────────
TEST(SkillBoreCylindrical, ValidateDFM)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 20.0);

    // Sub-0.8 mm → DFM-002 ERROR
    {
        skill::bore_cylindrical::Input in;
        in.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
        in.position_x_mm = 25.0;
        in.position_y_mm = 25.0;
        in.diameter_mm   = 0.5;
        in.depth_mm      = 3.0;
        auto r = skill::bore_cylindrical::validate(*stock, in);
        EXPECT_FALSE(r.passed);
        bool found = false;
        for (const auto& f : r.findings)
            if (f.code == "DFM-002") { found = true; break; }
        EXPECT_TRUE(found);
        EXPECT_THROW(skill::bore_cylindrical::apply(*stock, in), skill::SkillError);
    }

    // 4 mm (< 6) → still passes (warning only)
    {
        skill::bore_cylindrical::Input in;
        in.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
        in.position_x_mm = 25.0;
        in.position_y_mm = 25.0;
        in.diameter_mm   = 4.0;
        in.depth_mm      = 8.0;
        auto r = skill::bore_cylindrical::validate(*stock, in);
        EXPECT_TRUE(r.passed);
        bool warn = false;
        for (const auto& f : r.findings)
            if (f.code == "DFM-BORE-DIA" && f.severity == "warning") { warn = true; break; }
        EXPECT_TRUE(warn);
    }
}

// ─── 3. Recognize: precision bore yields high-confidence candidate ────────
TEST(SkillBoreCylindrical, RecognizeFindsBore)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 25.0);

    skill::bore_cylindrical::Input in;
    in.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 30.0;
    in.position_y_mm = 30.0;
    in.diameter_mm   = 12.0;
    in.depth_mm      = 15.0;
    in.tolerance_class = "H7";

    auto out = skill::bore_cylindrical::apply(*stock, in);
    auto cands = skill::bore_cylindrical::recognize(*out.workpiece);
    ASSERT_GE(cands.size(), 1u);

    // Largest-confidence match should look like our bore.
    auto best = cands.front();
    for (const auto& c : cands)
        if (c.confidence > best.confidence) best = c;
    EXPECT_GT(best.confidence, 0.8);
    EXPECT_NEAR(best.recovered_params["diameter_mm"].get<double>(), 12.0, 1e-3);
    EXPECT_NEAR(best.recovered_params["depth_mm"].get<double>(),    15.0, 1e-3);
}

// ─── 4. STEP round-trip preserves recognition ─────────────────────────────
TEST(SkillBoreCylindrical, RoundTripViaStep)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 25.0);

    skill::bore_cylindrical::Input in;
    in.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 28.0;
    in.position_y_mm = 32.0;
    in.diameter_mm   = 10.0;
    in.depth_mm      = 14.0;
    in.tolerance_class = "H7";

    auto synth = skill::bore_cylindrical::apply(*stock, in);
    const fs::path stepPath = fs::temp_directory_path() / "bore_cyl_roundtrip.step";
    std::string err;
    ASSERT_TRUE(io::StepIO::write(synth.workpiece->shape(), stepPath, err)) << err;

    auto reimportedOpt = io::StepIO::read(stepPath, err);
    ASSERT_TRUE(reimportedOpt.has_value()) << err;
    skill::Workpiece reim(*reimportedOpt);

    auto cands = skill::bore_cylindrical::recognize(reim);
    ASSERT_GE(cands.size(), 1u);
    auto best = cands.front();
    for (const auto& c : cands)
        if (c.confidence > best.confidence) best = c;
    EXPECT_NEAR(best.recovered_params["diameter_mm"].get<double>(), in.diameter_mm, 1e-2);
    EXPECT_NEAR(best.recovered_params["depth_mm"].get<double>(),    in.depth_mm,    1e-2);

    // Topology hash via re-synthesis with the same params.
    auto stock2 = skill::createCuboidStock(60.0, 60.0, 25.0);
    auto resynth = skill::bore_cylindrical::apply(*stock2, in);
    EXPECT_EQ(synth.workpiece->faceCount(),   resynth.workpiece->faceCount());
    EXPECT_NEAR(volumeOf(synth.workpiece->shape()),
                volumeOf(resynth.workpiece->shape()), 1e-3);
}

// ─── 5. Edge: small-diameter bore is ambiguous (lower confidence) ─────────
TEST(SkillBoreCylindrical, SmallDiaIsAmbiguousVsDrill)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 12.0);

    skill::bore_cylindrical::Input in;
    in.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 20.0;
    in.position_y_mm = 20.0;
    in.diameter_mm   = 3.0;     // < 6 mm — boring tool unusual at this size
    in.depth_mm      = 6.0;     // ratio = 2 — within bore range
    in.tolerance_class = "H7";

    // DFM passes (warning only for dia < 6).
    auto r = skill::bore_cylindrical::validate(*stock, in);
    EXPECT_TRUE(r.passed);

    auto out = skill::bore_cylindrical::apply(*stock, in);
    auto cands = skill::bore_cylindrical::recognize(*out.workpiece);

    // recognize() should either NOT return a candidate (criteria fail) or
    // return one with confidence < 0.6 (ambiguous with drill_hole).
    if (!cands.empty()) {
        for (const auto& c : cands) {
            EXPECT_LT(c.confidence, 0.6)
                << "small-dia bore should be ambiguous vs drill_hole";
        }
    }
}

// ─── 6. Measured geometric recovery from a FRESH workpiece ────────────────
//
// Build the bore with apply(), then construct a BRAND-NEW skill::Workpiece
// straight from the resulting shape.  A fresh Workpiece carries NO feature
// history, so recognize() is forced down the pure-GEOMETRIC path (no
// metadata replay).  Assert that the measured diameter, depth, 3-D position
// and axis direction all match the authored values tightly — this is what
// has to hold on a foreign downloaded STEP, where there is no metadata at all.
TEST(SkillBoreCylindrical, RecognizeRecoversMeasuredDimensions)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 25.0);

    skill::bore_cylindrical::Input in;
    in.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 27.0;
    in.position_y_mm = 33.0;
    in.axis_dir      = gp_Dir(0, 0, -1);
    in.diameter_mm   = 14.0;
    in.depth_mm      = 16.0;
    in.tolerance_class = "H7";

    auto synth = skill::bore_cylindrical::apply(*stock, in);

    // FRESH workpiece from the bare shape — no feature history → geometric path.
    skill::Workpiece fresh(synth.workpiece->shape());
    auto cands = skill::bore_cylindrical::recognize(fresh);
    ASSERT_GE(cands.size(), 1u);

    auto best = cands.front();
    for (const auto& c : cands)
        if (c.confidence > best.confidence) best = c;

    // Measured diameter / depth within a tight band of the authored values.
    EXPECT_NEAR(best.recovered_params["diameter_mm"].get<double>(), 14.0, 0.1);
    EXPECT_NEAR(best.recovered_params["depth_mm"].get<double>(),    16.0, 0.2);

    // Full 3-D entry position recovered (top face is at Z = 25 mm).
    EXPECT_NEAR(best.recovered_params["position_x_mm"].get<double>(), 27.0, 0.2);
    EXPECT_NEAR(best.recovered_params["position_y_mm"].get<double>(), 33.0, 0.2);
    ASSERT_TRUE(best.recovered_params.contains("position_z_mm"));
    EXPECT_NEAR(best.recovered_params["position_z_mm"].get<double>(), 25.0, 0.2);

    // Axis direction points entry → into the material (−Z here), recovered
    // orientation-independently from adjacency.
    auto axis = best.recovered_params["axis_dir"];
    ASSERT_EQ(axis.size(), 3u);
    EXPECT_NEAR(axis[0].get<double>(),  0.0, 1e-3);
    EXPECT_NEAR(axis[1].get<double>(),  0.0, 1e-3);
    EXPECT_NEAR(axis[2].get<double>(), -1.0, 1e-3);

    EXPECT_GT(best.confidence, 0.8);
}

// ─── 6b. RADIAL bore: a blind bore into the +X SIDE face (axis -X, e.g. a
// watch crown stem or a phone side-key bore) must recover with the ±X axis and
// full 3-D entry — the recognizer is orientation-independent, not Z-biased. ──
TEST(SkillBoreCylindrical, RecognizesRadialSideBore)
{
    // Stock corner at origin: x∈[0,40], y∈[0,60], z∈[0,30].  Bore into the +X
    // face (x=40) along -X, at mid-height.
    auto stock = skill::createCuboidStock(40.0, 60.0, 30.0);

    skill::bore_cylindrical::Input in;
    in.entry_face    = skill::FaceByNormal{ gp_Dir(1, 0, 0), 5.0, "largest" };  // +X side
    in.position_x_mm = 40.0;          // on the +X face plane
    in.position_y_mm = 30.0;
    in.position_z_mm = 15.0;          // mid-height entry point
    in.axis_dir      = gp_Dir(-1, 0, 0);   // bore inward along -X
    in.diameter_mm   = 12.0;
    in.depth_mm      = 10.0;
    in.tolerance_class = "H7";
    auto synth = skill::bore_cylindrical::apply(*stock, in);

    skill::Workpiece fresh(synth.workpiece->shape());   // geometric path
    const auto cands = skill::bore_cylindrical::recognize(fresh);
    ASSERT_GE(cands.size(), 1u) << "a radial side bore must be recognised";
    auto best = cands.front();
    for (const auto& c : cands)
        if (c.confidence > best.confidence) best = c;

    EXPECT_NEAR(best.recovered_params["diameter_mm"].get<double>(), 12.0, 0.2);
    EXPECT_NEAR(best.recovered_params["depth_mm"].get<double>(),    10.0, 0.3);
    // The axis must be ±X (radial), NOT ±Z.
    const auto axis = best.recovered_params["axis_dir"];
    ASSERT_EQ(axis.size(), 3u);
    EXPECT_NEAR(std::abs(axis[0].get<double>()), 1.0, 1e-2) << "bore axis is ±X (radial)";
    EXPECT_NEAR(axis[2].get<double>(), 0.0, 1e-2);
    // Entry X is the +X face plane (x=40).
    ASSERT_TRUE(best.recovered_params.contains("position_x_mm"));
    EXPECT_NEAR(best.recovered_params["position_x_mm"].get<double>(), 40.0, 0.3)
        << "entry on the +X face";

    // REGENERATE from the recovered params (incl. the 3-D entry Z) → same removed
    // volume, in place.  This proves apply() places a radial bore from the real
    // entry point (the old code pulled the start back by the bbox diagonal, so
    // the cutter never reached the stock and removed nothing).
    auto fresh2 = skill::createCuboidStock(40.0, 60.0, 30.0);
    const double vFresh = volumeOf(fresh2->shape());
    skill::bore_cylindrical::Input in2;
    in2.entry_face    = skill::FaceByNormal{ gp_Dir(1, 0, 0), 5.0, "largest" };
    in2.position_x_mm = best.recovered_params["position_x_mm"].get<double>();
    in2.position_y_mm = best.recovered_params["position_y_mm"].get<double>();
    in2.position_z_mm = best.recovered_params["position_z_mm"].get<double>();
    in2.axis_dir      = gp_Dir(axis[0].get<double>(), axis[1].get<double>(), axis[2].get<double>());
    in2.diameter_mm   = best.recovered_params["diameter_mm"].get<double>();
    in2.depth_mm      = best.recovered_params["depth_mm"].get<double>();
    const auto regen = skill::bore_cylindrical::apply(*fresh2, in2);
    EXPECT_NEAR(vFresh - volumeOf(regen.workpiece->shape()),
                M_PI * 6.0 * 6.0 * 10.0, M_PI * 6.0 * 6.0 * 10.0 * 0.05)
        << "the recovered radial bore regenerates the same removed volume";
}

// ─── 7. Concavity gate: a convex BOSS (rod) is NOT a bore ─────────────────
//
// An internal bore is a CONCAVE cylinder (solid OUTSIDE the wall).  A convex
// boss / shaft (solid INSIDE the cylinder) must be rejected, mirroring
// drill_hole::RejectsConvexBossAsHole.  We make a real bore at (20,20) and
// fuse a cylindrical boss at (45,45); only the bore may be recovered.
TEST(SkillBoreCylindrical, RejectsConvexBoss)
{
    namespace pr = koocadcam::engine::prim;
    auto stock = skill::createCuboidStock(60.0, 60.0, 25.0);

    // A real precision bore at (20,20): concave, dia 12 mm, depth 15 mm.
    skill::bore_cylindrical::Input b;
    b.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    b.position_x_mm = 20.0;
    b.position_y_mm = 20.0;
    b.axis_dir      = gp_Dir(0, 0, -1);
    b.diameter_mm   = 12.0;
    b.depth_mm      = 15.0;
    b.tolerance_class = "H7";
    auto bored = skill::bore_cylindrical::apply(*stock, b);

    // A convex cylindrical boss standing up from the top face at (45,45):
    // a 12 mm-dia rod — same diameter as the bore, so only the concavity gate
    // can tell them apart.
    const TopoDS_Shape boss = pr::cylinder(
        gp_Ax2(gp_Pnt(45.0, 45.0, 25.0), gp_Dir(0, 0, 1)), 6.0, 10.0);
    const TopoDS_Shape withBoss = pr::fuse(bored.workpiece->shape(), boss);
    skill::Workpiece wp(withBoss);

    auto cands = skill::bore_cylindrical::recognize(wp);
    int atBore = 0, atBoss = 0;
    for (const auto& c : cands) {
        const double x = c.recovered_params.value("position_x_mm", 0.0);
        const double y = c.recovered_params.value("position_y_mm", 0.0);
        if (std::abs(x - 20.0) < 1.5 && std::abs(y - 20.0) < 1.5) ++atBore;
        if (std::abs(x - 45.0) < 1.5 && std::abs(y - 45.0) < 1.5) ++atBoss;
    }
    EXPECT_GT(atBore, 0) << "the real concave bore must still be recovered";
    EXPECT_EQ(atBoss, 0) << "the convex boss must NOT be recognized as a bore";
}
