// @lat: [[process/test-strategy#skill round-trip]]
//
// deep_hole_peck — twist drill peck cycle.  Topology = drill_hole.  Sweet
// spot depth/dia between 4 and 10.
//
// Cases (5):
//   1. Synthesis removes the expected π r² depth.
//   2. DFM rejects sub-0.8 mm diameter AND peck_depth < dia × 0.5.
//   3. Recognize finds the hole when depth/dia is in [4, 10].
//   4. STEP round-trip preserves recognition.
//   5. peck_count is correctly ceil(depth / peck_depth).

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/deep_hole_peck.hpp"

#include "io/StepIO.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

#include <cmath>
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

// ─── 1. Synthesis removes the expected volume ─────────────────────────────
TEST(SkillDeepHolePeck, ApplyRemovesExpectedVolume)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 30.0);

    skill::deep_hole_peck::Input in;
    in.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1), 5.0, "largest" };
    in.position_x_mm = 20.0;
    in.position_y_mm = 20.0;
    in.axis_dir      = gp_Dir(0, 0, -1);
    in.diameter_mm   = 5.0;
    in.depth_mm      = 25.0;         // ratio = 5 → peck range
    in.peck_depth_mm = 8.0;          // > 0.5 × 5 = 2.5 → OK

    auto out = skill::deep_hole_peck::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    const double approx = M_PI * 2.5 * 2.5 * 25.0;
    const double diff = volumeOf(stock->shape()) - volumeOf(out.workpiece->shape());
    EXPECT_NEAR(diff, approx, approx * 0.05);

    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_EQ(out.signature.skill_id, std::string("deep_hole_peck"));
    // peck_count = ceil(25 / 8) = 4
    EXPECT_EQ(out.signature.tooling.extra["peck_count"].get<int>(), 4);
}

// ─── 2. DFM: sub-0.8 mm dia AND peck_depth too small ──────────────────────
TEST(SkillDeepHolePeck, ValidateDFM)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 30.0);

    // Sub-0.8 mm → error
    {
        skill::deep_hole_peck::Input in;
        in.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
        in.position_x_mm = 20.0;
        in.position_y_mm = 20.0;
        in.diameter_mm   = 0.5;
        in.depth_mm      = 4.0;
        auto r = skill::deep_hole_peck::validate(*stock, in);
        EXPECT_FALSE(r.passed);
        bool found = false;
        for (const auto& f : r.findings)
            if (f.code == "DFM-002") { found = true; break; }
        EXPECT_TRUE(found);
    }

    // peck_depth < dia × 0.5 → error
    {
        skill::deep_hole_peck::Input in;
        in.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
        in.position_x_mm = 20.0;
        in.position_y_mm = 20.0;
        in.diameter_mm   = 5.0;       // 0.5×5 = 2.5
        in.depth_mm      = 25.0;
        in.peck_depth_mm = 1.0;       // < 2.5
        auto r = skill::deep_hole_peck::validate(*stock, in);
        EXPECT_FALSE(r.passed);
        bool found = false;
        for (const auto& f : r.findings)
            if (f.code == "DFM-PECK-DEPTH") { found = true; break; }
        EXPECT_TRUE(found);
    }
}

// ─── 3. Recognize finds the deep-hole candidate ───────────────────────────
TEST(SkillDeepHolePeck, RecognizeFindsHole)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 30.0);

    skill::deep_hole_peck::Input in;
    in.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 20.0;
    in.position_y_mm = 20.0;
    in.diameter_mm   = 4.0;
    in.depth_mm      = 20.0;         // ratio = 5
    in.peck_depth_mm = 6.0;

    auto out = skill::deep_hole_peck::apply(*stock, in);
    auto cands = skill::deep_hole_peck::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);

    const auto& c = cands.front();
    EXPECT_EQ(c.skill_id, std::string("deep_hole_peck"));
    EXPECT_NEAR(c.confidence, 0.5, 1e-6);
    EXPECT_NEAR(c.recovered_params["diameter_mm"].get<double>(), 4.0, 1e-3);
    EXPECT_NEAR(c.recovered_params["depth_mm"].get<double>(),    20.0, 1e-3);
    EXPECT_NEAR(c.matched_geometry["depth_dia_ratio"].get<double>(), 5.0, 1e-3);
}

// ─── 4. STEP round-trip ──────────────────────────────────────────────────
TEST(SkillDeepHolePeck, RoundTripViaStep)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 30.0);

    skill::deep_hole_peck::Input in;
    in.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 18.0;
    in.position_y_mm = 22.0;
    in.diameter_mm   = 4.0;
    in.depth_mm      = 24.0;         // ratio = 6
    in.peck_depth_mm = 6.0;

    auto synth = skill::deep_hole_peck::apply(*stock, in);
    const fs::path stepPath = fs::temp_directory_path() / "deep_peck_roundtrip.step";
    std::string err;
    ASSERT_TRUE(io::StepIO::write(synth.workpiece->shape(), stepPath, err)) << err;

    auto reimported = io::StepIO::read(stepPath, err);
    ASSERT_TRUE(reimported.has_value()) << err;
    skill::Workpiece reim(*reimported);

    auto cands = skill::deep_hole_peck::recognize(reim);
    ASSERT_GE(cands.size(), 1u);
    const auto& c = cands.front();
    EXPECT_NEAR(c.recovered_params["diameter_mm"].get<double>(), in.diameter_mm, 1e-2);
    EXPECT_NEAR(c.recovered_params["depth_mm"].get<double>(),    in.depth_mm,    1e-2);

    auto stock2 = skill::createCuboidStock(40.0, 40.0, 30.0);
    auto resynth = skill::deep_hole_peck::apply(*stock2, in);
    EXPECT_EQ(synth.workpiece->faceCount(), resynth.workpiece->faceCount());
    EXPECT_NEAR(volumeOf(synth.workpiece->shape()),
                volumeOf(resynth.workpiece->shape()), 1e-3);
}

// ─── 5. peck_count calculation ────────────────────────────────────────────
TEST(SkillDeepHolePeck, PeckCountIsCorrect)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 30.0);

    // depth = 20, peck_depth = 7 → ceil(20/7) = 3
    skill::deep_hole_peck::Input in;
    in.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 20.0;
    in.position_y_mm = 20.0;
    in.diameter_mm   = 4.0;
    in.depth_mm      = 20.0;
    in.peck_depth_mm = 7.0;
    auto out1 = skill::deep_hole_peck::apply(*stock, in);
    EXPECT_EQ(out1.signature.tooling.extra["peck_count"].get<int>(), 3);

    // Auto peck_depth (= 1.5 × dia = 6) → ceil(20/6) = 4
    auto stock2 = skill::createCuboidStock(40.0, 40.0, 30.0);
    in.peck_depth_mm = 0.0;          // sentinel → auto
    auto out2 = skill::deep_hole_peck::apply(*stock2, in);
    EXPECT_EQ(out2.signature.tooling.extra["peck_count"].get<int>(), 4);
    EXPECT_NEAR(out2.signature.tooling.extra["peck_depth_mm"].get<double>(),
                6.0, 1e-9);
}
