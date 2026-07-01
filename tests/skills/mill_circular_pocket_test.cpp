// @lat: [[process/test-strategy#skill round-trip]]

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/mill_circular_pocket.hpp"

#include "io/StepIO.hpp"

#include "engine/primitives/Tools.hpp"
#include "engine/primitives/Cuts.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>
#include <gp_Ax2.hxx>

#include <cmath>
#include <filesystem>

namespace fs = std::filesystem;
using namespace koocadcam;

namespace {
double volumeOf(const TopoDS_Shape& s) {
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}
}

TEST(SkillMillCircularPocket, ApplyCreatesPocket)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 12.0);

    skill::mill_circular_pocket::Input in;
    in.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 30.0;
    in.position_y_mm = 30.0;
    in.diameter_mm   = 10.0;
    in.depth_mm      = 3.0;

    auto out = skill::mill_circular_pocket::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    const double expectedRem = M_PI * 5.0 * 5.0 * 3.0;
    EXPECT_NEAR(volumeOf(stock->shape()) - volumeOf(out.workpiece->shape()),
                expectedRem, expectedRem * 0.05);
}

TEST(SkillMillCircularPocket, ValidateRejectsSubMin)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);
    skill::mill_circular_pocket::Input in;
    in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 25.0; in.position_y_mm = 25.0;
    in.diameter_mm = 0.5; in.depth_mm = 2.0;

    auto r = skill::mill_circular_pocket::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_THROW(skill::mill_circular_pocket::apply(*stock, in), skill::SkillError);
}

TEST(SkillMillCircularPocket, RecognizeFinds)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 12.0);
    skill::mill_circular_pocket::Input in;
    in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 30.0; in.position_y_mm = 30.0;
    in.diameter_mm = 10.0; in.depth_mm = 3.0;

    auto out = skill::mill_circular_pocket::apply(*stock, in);
    auto cands = skill::mill_circular_pocket::recognize(*out.workpiece);
    ASSERT_FALSE(cands.empty());
    EXPECT_NEAR(cands[0].recovered_params["diameter_mm"].get<double>(), 10.0, 0.1);
    EXPECT_NEAR(cands[0].recovered_params["depth_mm"].get<double>(),     3.0, 0.1);
}

TEST(SkillMillCircularPocket, RoundTripViaStep)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 12.0);
    skill::mill_circular_pocket::Input in;
    in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 20.0; in.position_y_mm = 35.0;
    in.diameter_mm = 8.0; in.depth_mm = 2.5;

    auto synth = skill::mill_circular_pocket::apply(*stock, in);

    const fs::path stepPath = fs::temp_directory_path() / "mill_circ_pkt_rt.step";
    std::string err;
    ASSERT_TRUE(io::StepIO::write(synth.workpiece->shape(), stepPath, err));
    auto reimp = io::StepIO::read(stepPath, err);
    ASSERT_TRUE(reimp.has_value());
    skill::Workpiece reim(*reimp);

    auto cands = skill::mill_circular_pocket::recognize(reim);
    ASSERT_FALSE(cands.empty());
    EXPECT_NEAR(cands[0].recovered_params["diameter_mm"].get<double>(), in.diameter_mm, 0.1);
    EXPECT_NEAR(cands[0].recovered_params["depth_mm"].get<double>(),    in.depth_mm,    0.1);
}

TEST(SkillMillCircularPocket, BottomCornerFillet)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 12.0);
    skill::mill_circular_pocket::Input in;
    in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 25.0; in.position_y_mm = 25.0;
    in.diameter_mm = 12.0; in.depth_mm = 4.0;
    in.bottom_corner_r_mm = 0.5;

    auto out = skill::mill_circular_pocket::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    // Corner-fillet variant introduces a toroidal face → faceCount increases
    // by more than the no-fillet variant.
    EXPECT_GT(out.workpiece->faceCount(), stock->faceCount());
}

// ─── Measured geometric recovery from a FRESH workpiece ────────────────────
//
// Build a fresh skill::Workpiece directly from the cut shape (NOT the apply
// output) so it carries NO feature history — this forces recognize() down the
// GEOMETRIC fallback path.  Every recovered dimension is then MEASURED from the
// B-rep: diameter = 2·cylinder.Radius(), depth = wall-to-floor span, and the
// full 3-D entry centre (x,y,z) from the ring opening onto the large top face.
TEST(SkillMillCircularPocket, MeasuredGeometricRecovery)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 12.0);
    skill::mill_circular_pocket::Input in;
    in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 22.0; in.position_y_mm = 38.0;
    in.diameter_mm = 14.0; in.depth_mm = 4.5;

    auto out = skill::mill_circular_pocket::apply(*stock, in);

    // Fresh workpiece from the geometry alone — no addFeature() history.
    skill::Workpiece fresh(out.workpiece->shape());
    ASSERT_TRUE(fresh.features().empty());

    auto cands = skill::mill_circular_pocket::recognize(fresh);
    ASSERT_FALSE(cands.empty());
    const auto& rp = cands[0].recovered_params;

    EXPECT_NEAR(rp["diameter_mm"].get<double>(), in.diameter_mm, 0.1);
    EXPECT_NEAR(rp["depth_mm"].get<double>(),    in.depth_mm,    0.1);
    // Entry centre lies on the top face (z=12) at the authored XY.
    EXPECT_NEAR(rp["position_x_mm"].get<double>(), in.position_x_mm, 0.2);
    EXPECT_NEAR(rp["position_y_mm"].get<double>(), in.position_y_mm, 0.2);
    EXPECT_NEAR(rp["position_z_mm"].get<double>(), 12.0,             0.2);
    // Plunge axis points into the material (−Z) for a top-entry pocket.
    EXPECT_NEAR(rp["axis_dir"][2].get<double>(), -1.0, 1e-3);
    EXPECT_NEAR(rp["bottom_corner_r_mm"].get<double>(), 0.0, 1e-6);
}

// ─── RADIAL side pocket: a round recess on the +X SIDE face (axis -X — a watch
// case-side well or a phone side sensor window) must apply, recover the ±X axis
// + 3-D entry, and regenerate the same volume.  The old apply() started the tool
// a bbox-diagonal outside the +X face and removed nothing. ────────────────────
TEST(SkillMillCircularPocket, RadialSidePocketRoundTrips)
{
    // Stock corner at origin: x∈[0,40], y∈[0,60], z∈[0,30].  Pocket into +X.
    auto stock = skill::createCuboidStock(40.0, 60.0, 30.0);
    const double vStock = volumeOf(stock->shape());
    skill::mill_circular_pocket::Input in;
    in.entry_face    = skill::FaceByNormal{ gp_Dir(1, 0, 0), 5.0, "largest" };  // +X side
    in.position_x_mm = 40.0;          // on the +X face plane
    in.position_y_mm = 30.0;
    in.position_z_mm = 15.0;          // mid-height entry
    in.axis_dir      = gp_Dir(-1, 0, 0);
    in.diameter_mm   = 12.0;
    in.depth_mm      = 5.0;
    auto out = skill::mill_circular_pocket::apply(*stock, in);
    const double removed = vStock - volumeOf(out.workpiece->shape());
    ASSERT_NEAR(removed, M_PI * 6.0 * 6.0 * 5.0, M_PI * 6.0 * 6.0 * 5.0 * 0.05)
        << "the radial side pocket must actually remove material";

    skill::Workpiece fresh(out.workpiece->shape());
    auto cands = skill::mill_circular_pocket::recognize(fresh);
    ASSERT_FALSE(cands.empty()) << "a radial side pocket must be recognised";
    const auto& rp = cands[0].recovered_params;
    EXPECT_NEAR(rp["diameter_mm"].get<double>(), 12.0, 0.2);
    EXPECT_NEAR(rp["depth_mm"].get<double>(),    5.0,  0.3);
    // Axis is ±X (radial), entry on the +X face.
    EXPECT_NEAR(std::abs(rp["axis_dir"][0].get<double>()), 1.0, 1e-2) << "axis is ±X";
    EXPECT_NEAR(rp["axis_dir"][2].get<double>(), 0.0, 1e-2);
    EXPECT_NEAR(rp["position_x_mm"].get<double>(), 40.0, 0.3) << "entry on +X face";

    // Regenerate from the recovered params → same removed volume.
    auto fresh2 = skill::createCuboidStock(40.0, 60.0, 30.0);
    const double vFresh = volumeOf(fresh2->shape());
    skill::mill_circular_pocket::Input in2;
    in2.entry_face    = skill::FaceByNormal{ gp_Dir(1, 0, 0), 5.0, "largest" };
    in2.position_x_mm = rp["position_x_mm"].get<double>();
    in2.position_y_mm = rp["position_y_mm"].get<double>();
    in2.position_z_mm = rp["position_z_mm"].get<double>();
    in2.axis_dir      = gp_Dir(rp["axis_dir"][0].get<double>(),
                               rp["axis_dir"][1].get<double>(),
                               rp["axis_dir"][2].get<double>());
    in2.diameter_mm   = rp["diameter_mm"].get<double>();
    in2.depth_mm      = rp["depth_mm"].get<double>();
    auto regen = skill::mill_circular_pocket::apply(*fresh2, in2);
    EXPECT_NEAR(vFresh - volumeOf(regen.workpiece->shape()), removed, removed * 0.05)
        << "the recovered radial pocket regenerates the same removed volume";
}

// ─── Concavity gate: a convex BOSS is NOT a pocket ─────────────────────────
//
// recognize() must reject convex cylinders.  Mill a real pocket at (20,20) and
// fuse a cylindrical boss at (40,40); only the concave flat-bottom pocket may
// be recovered — the boss (material INSIDE the cylinder) must not.  Modelled on
// drill_hole's RejectsConvexBossAsHole.
TEST(SkillMillCircularPocket, RejectsConvexBossAsPocket)
{
    namespace pr = koocadcam::engine::prim;
    auto stock = skill::createCuboidStock(60.0, 60.0, 12.0);

    // A real circular pocket at (20,20): wide + shallow (aspect ratio < 2).
    skill::mill_circular_pocket::Input p;
    p.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    p.position_x_mm = 20.0; p.position_y_mm = 20.0;
    p.diameter_mm = 12.0; p.depth_mm = 3.0;
    auto pocketed = skill::mill_circular_pocket::apply(*stock, p);

    // A convex cylindrical boss standing up from the top face at (40,40).  It
    // butts onto the top plane (a small adjacent planar) yet must be rejected.
    const TopoDS_Shape boss = pr::cylinder(
        gp_Ax2(gp_Pnt(40.0, 40.0, 12.0), gp_Dir(0, 0, 1)), 6.0, 5.0);
    const TopoDS_Shape withBoss = pr::fuse(pocketed.workpiece->shape(), boss);
    skill::Workpiece wp(withBoss);

    auto cands = skill::mill_circular_pocket::recognize(wp);
    int atPocket = 0, atBoss = 0;
    for (const auto& c : cands) {
        const double x = c.recovered_params.value("position_x_mm", 0.0);
        const double y = c.recovered_params.value("position_y_mm", 0.0);
        if (std::abs(x - 20.0) < 1.0 && std::abs(y - 20.0) < 1.0) ++atPocket;
        if (std::abs(x - 40.0) < 1.0 && std::abs(y - 40.0) < 1.0) ++atBoss;
    }
    EXPECT_GT(atPocket, 0) << "the real concave pocket must still be recovered";
    EXPECT_EQ(atBoss, 0)   << "the convex boss must NOT be recognized as a pocket";
}
