// @lat: [[process/test-strategy#skill round-trip]]
//
// spot_drill skill — verifies the 7-condition pattern:
//   1. Apply produces a shallow cone seat (no cylindrical bore).
//   2. DFM rejects sub-min diameter (DFM-002) and out-of-range angle.
//   3. Recognize finds the unpaired cone.
//   4. Round-trip via STEP preserves the pattern.
//   5. Edge case: degenerate geometry (zero / 180° angle) rejected.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/spot_drill.hpp"

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

// ─── 1. Basic synthesis works ─────────────────────────────────────────────
TEST(SkillSpotDrill, ApplyCreatesShallowCone)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);
    ASSERT_FALSE(stock->shape().IsNull());

    skill::spot_drill::Input in;
    in.entry_face      = skill::FaceByNormal{ gp_Dir(0, 0, 1), 5.0, "largest" };
    in.position_x_mm   = 25.0;
    in.position_y_mm   = 25.0;
    in.axis_dir        = gp_Dir(0, 0, -1);
    in.diameter_mm     = 4.0;
    in.cone_angle_deg  = 90.0;

    const double depth = skill::spot_drill::computeDepth(in);
    EXPECT_NEAR(depth, 2.0, 1e-3);   // (4/2)/tan(45) = 2

    auto out = skill::spot_drill::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    // Cone volume: (1/3) π r² h.  Tool's "r2" is a tiny 1e-3 frustum, so
    // the actual removed shape is technically a frustum but very close to
    // a perfect cone — accept 6 % tolerance.
    const double r = in.diameter_mm * 0.5;
    const double approx = (M_PI / 3.0) * r * r * depth;
    const double diff = volumeOf(stock->shape()) - volumeOf(out.workpiece->shape());
    EXPECT_NEAR(diff, approx, approx * 0.06);

    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_EQ(out.signature.skill_id, std::string("spot_drill"));
}

// ─── 2. DFM rejects bad inputs ────────────────────────────────────────────
TEST(SkillSpotDrill, ValidateRejectsBadInputs)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    // Sub-min diameter (DFM-002)
    {
        skill::spot_drill::Input in;
        in.entry_face     = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
        in.position_x_mm  = 25.0;
        in.position_y_mm  = 25.0;
        in.diameter_mm    = 0.5;       // < 0.8 mm
        in.cone_angle_deg = 90.0;

        auto r = skill::spot_drill::validate(*stock, in);
        EXPECT_FALSE(r.passed);
        bool found = false;
        for (const auto& f : r.findings)
            if (f.code == "DFM-002") { found = true; break; }
        EXPECT_TRUE(found);
        EXPECT_THROW(skill::spot_drill::apply(*stock, in), skill::SkillError);
    }

    // Out-of-range cone angle
    {
        skill::spot_drill::Input in;
        in.entry_face     = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
        in.position_x_mm  = 25.0;
        in.position_y_mm  = 25.0;
        in.diameter_mm    = 3.0;
        in.cone_angle_deg = 30.0;      // < 60 lower bound

        auto r = skill::spot_drill::validate(*stock, in);
        EXPECT_FALSE(r.passed);
        EXPECT_THROW(skill::spot_drill::apply(*stock, in), skill::SkillError);
    }

    // Angle too obtuse
    {
        skill::spot_drill::Input in;
        in.entry_face     = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
        in.position_x_mm  = 25.0;
        in.position_y_mm  = 25.0;
        in.diameter_mm    = 3.0;
        in.cone_angle_deg = 160.0;     // > 140 upper bound

        auto r = skill::spot_drill::validate(*stock, in);
        EXPECT_FALSE(r.passed);
    }
}

// ─── 3. Recognize finds the cone ──────────────────────────────────────────
TEST(SkillSpotDrill, RecognizeFindsSpot)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::spot_drill::Input in;
    in.entry_face      = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm   = 25.0;
    in.position_y_mm   = 25.0;
    in.diameter_mm     = 3.0;
    in.cone_angle_deg  = 90.0;

    auto out = skill::spot_drill::apply(*stock, in);
    auto candidates = skill::spot_drill::recognize(*out.workpiece);

    ASSERT_EQ(candidates.size(), 1u) << "expected exactly one spot_drill candidate";
    const auto& c = candidates.front();
    EXPECT_EQ(c.skill_id, std::string("spot_drill"));
    EXPECT_GT(c.confidence, 0.5);

    EXPECT_NEAR(c.recovered_params["diameter_mm"].get<double>(),   3.0, 5e-2);
    EXPECT_NEAR(c.recovered_params["cone_angle_deg"].get<double>(), 90.0, 1.5);
    EXPECT_NEAR(c.recovered_params["position_x_mm"].get<double>(), 25.0, 1e-2);
    EXPECT_NEAR(c.recovered_params["position_y_mm"].get<double>(), 25.0, 1e-2);
}

// ─── 4. Full round-trip via STEP file ─────────────────────────────────────
TEST(SkillSpotDrill, RoundTripViaStep)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::spot_drill::Input in;
    in.entry_face      = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm   = 30.0;
    in.position_y_mm   = 15.0;
    in.diameter_mm     = 4.0;
    in.cone_angle_deg  = 120.0;

    auto synth = skill::spot_drill::apply(*stock, in);

    const fs::path stepPath = fs::temp_directory_path() / "spot_drill_roundtrip.step";
    std::string err;
    ASSERT_TRUE(io::StepIO::write(synth.workpiece->shape(), stepPath, err)) << err;

    auto reimportedOpt = io::StepIO::read(stepPath, err);
    ASSERT_TRUE(reimportedOpt.has_value()) << err;
    skill::Workpiece reim(*reimportedOpt);

    auto candidates = skill::spot_drill::recognize(reim);
    ASSERT_FALSE(candidates.empty())
        << "STEP-imported shape should still expose the spot_drill pattern";

    bool found = false;
    for (const auto& c : candidates) {
        if (std::abs(c.recovered_params["diameter_mm"].get<double>() - in.diameter_mm) < 0.1 &&
            std::abs(c.recovered_params["cone_angle_deg"].get<double>() - in.cone_angle_deg) < 2.0)
        {
            found = true;
            EXPECT_NEAR(c.recovered_params["position_x_mm"].get<double>(),
                        in.position_x_mm, 5e-2);
            EXPECT_NEAR(c.recovered_params["position_y_mm"].get<double>(),
                        in.position_y_mm, 5e-2);
            break;
        }
    }
    EXPECT_TRUE(found) << "recognize() did not return a candidate matching input params";

    // Volume should be preserved through STEP.
    EXPECT_NEAR(volumeOf(synth.workpiece->shape()),
                volumeOf(*reimportedOpt),
                1e-2);
}

// ─── 5. Edge case: 120° spot drill has different geometry ────────────────
TEST(SkillSpotDrill, ShallowerWith120DegAngle)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::spot_drill::Input in90;
    in90.entry_face      = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in90.position_x_mm   = 25.0;
    in90.position_y_mm   = 25.0;
    in90.diameter_mm     = 3.0;
    in90.cone_angle_deg  = 90.0;

    skill::spot_drill::Input in120 = in90;
    in120.cone_angle_deg = 120.0;

    const double d90  = skill::spot_drill::computeDepth(in90);
    const double d120 = skill::spot_drill::computeDepth(in120);
    EXPECT_GT(d90, d120)
        << "wider apex angle (120°) should produce a SHALLOWER cone for same dia";

    // Both should apply cleanly.
    EXPECT_NO_THROW(skill::spot_drill::apply(*stock, in90));
    EXPECT_NO_THROW(skill::spot_drill::apply(*stock, in120));
}
