// @lat: [[process/test-strategy#skill round-trip]]
//
// gun_drill — deep, straight, precision drilling.  Geometrically identical
// to a deep drill_hole, differentiated through tooling metadata (single
// flute + through-tool coolant) and a depth/diameter ratio ≥ 10.
//
// Cases (5):
//   1. Synthesis on a tall stock removes the expected π r² depth.
//   2. DFM rejects sub-min (< 1 mm) and super-max (> 50 mm) diameters.
//   3. Recognize finds the cylindrical face when ratio ≥ 10.
//   4. STEP round-trip: re-recognize after export/import.
//   5. DFM rejects depth/dia < 10 (gun_drill not justified — warns).

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/gun_drill.hpp"

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
TEST(SkillGunDrill, ApplyRemovesExpectedVolume)
{
    // Tall stock so depth/dia = 60/4 = 15 → in gun-drill range.
    auto stock = skill::createCuboidStock(30.0, 30.0, 70.0);
    ASSERT_FALSE(stock->shape().IsNull());

    skill::gun_drill::Input in;
    in.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1), 5.0, "largest" };
    in.position_x_mm = 15.0;
    in.position_y_mm = 15.0;
    in.axis_dir      = gp_Dir(0, 0, -1);
    in.diameter_mm   = 4.0;
    in.depth_mm      = 60.0;
    in.through_hole  = false;
    in.straightness_class = "0.05_per_100mm";

    auto out = skill::gun_drill::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    const double approx = M_PI * 2.0 * 2.0 * 60.0;
    const double diff = volumeOf(stock->shape()) - volumeOf(out.workpiece->shape());
    EXPECT_NEAR(diff, approx, approx * 0.05);

    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_EQ(out.signature.skill_id, std::string("gun_drill"));
    EXPECT_EQ(out.signature.tooling.tool_type, std::string("gun_drill"));
    EXPECT_EQ(out.signature.tooling.flute_count, 1);
}

// ─── 2. DFM rejects sub-min and super-max diameters ───────────────────────
TEST(SkillGunDrill, ValidateRejectsOutOfRangeDia)
{
    auto stock = skill::createCuboidStock(30.0, 30.0, 70.0);

    // 0.5 mm → too small (min 1 mm)
    {
        skill::gun_drill::Input in;
        in.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
        in.position_x_mm = 15.0;
        in.position_y_mm = 15.0;
        in.diameter_mm   = 0.5;
        in.depth_mm      = 30.0;
        auto r = skill::gun_drill::validate(*stock, in);
        EXPECT_FALSE(r.passed);
        bool found = false;
        for (const auto& f : r.findings)
            if (f.code == "DFM-GUNDRILL-DIAMIN") { found = true; break; }
        EXPECT_TRUE(found);
        EXPECT_THROW(skill::gun_drill::apply(*stock, in), skill::SkillError);
    }

    // 60 mm → too large (max 50 mm)
    {
        skill::gun_drill::Input in;
        in.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
        in.position_x_mm = 15.0;
        in.position_y_mm = 15.0;
        in.diameter_mm   = 60.0;
        in.depth_mm      = 600.0;
        auto r = skill::gun_drill::validate(*stock, in);
        EXPECT_FALSE(r.passed);
        bool found = false;
        for (const auto& f : r.findings)
            if (f.code == "DFM-GUNDRILL-DIAMAX") { found = true; break; }
        EXPECT_TRUE(found);
    }
}

// ─── 3. Recognize finds the gun-drill candidate ───────────────────────────
TEST(SkillGunDrill, RecognizeFindsDeepHole)
{
    auto stock = skill::createCuboidStock(30.0, 30.0, 70.0);

    skill::gun_drill::Input in;
    in.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 15.0;
    in.position_y_mm = 15.0;
    in.diameter_mm   = 3.0;
    in.depth_mm      = 45.0;          // ratio = 15
    in.straightness_class = "0.05_per_100mm";

    auto out = skill::gun_drill::apply(*stock, in);
    auto cands = skill::gun_drill::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);

    const auto& c = cands.front();
    EXPECT_EQ(c.skill_id, std::string("gun_drill"));
    EXPECT_NEAR(c.confidence, 0.6, 1e-6);     // ambiguous w/ drill_hole etc.
    EXPECT_NEAR(c.recovered_params["diameter_mm"].get<double>(), 3.0, 1e-3);
    EXPECT_NEAR(c.recovered_params["depth_mm"].get<double>(),    45.0, 1e-3);
    EXPECT_GE(c.matched_geometry["depth_dia_ratio"].get<double>(), 10.0);
}

// ─── 4. STEP round-trip preserves the deep-hole recognition ───────────────
TEST(SkillGunDrill, RoundTripViaStep)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 80.0);

    skill::gun_drill::Input in;
    in.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 20.0;
    in.position_y_mm = 20.0;
    in.diameter_mm   = 5.0;
    in.depth_mm      = 60.0;          // ratio = 12
    in.straightness_class = "0.05_per_100mm";

    auto synth = skill::gun_drill::apply(*stock, in);
    const fs::path stepPath = fs::temp_directory_path() / "gun_drill_roundtrip.step";
    std::string err;
    ASSERT_TRUE(io::StepIO::write(synth.workpiece->shape(), stepPath, err)) << err;

    auto reim = io::StepIO::read(stepPath, err);
    ASSERT_TRUE(reim.has_value()) << err;
    skill::Workpiece reimp(*reim);

    auto cands = skill::gun_drill::recognize(reimp);
    ASSERT_GE(cands.size(), 1u);
    const auto& c = cands.front();
    EXPECT_NEAR(c.recovered_params["diameter_mm"].get<double>(), 5.0, 1e-2);
    EXPECT_NEAR(c.recovered_params["depth_mm"].get<double>(),    60.0, 1e-2);

    // Topology hash via re-synthesis.
    auto stock2 = skill::createCuboidStock(40.0, 40.0, 80.0);
    auto resynth = skill::gun_drill::apply(*stock2, in);
    EXPECT_EQ(synth.workpiece->faceCount(), resynth.workpiece->faceCount());
    EXPECT_NEAR(volumeOf(synth.workpiece->shape()),
                volumeOf(resynth.workpiece->shape()), 1e-3);
}

// ─── 5. Sub-10 depth/dia → warning (synthesis still proceeds) ─────────────
TEST(SkillGunDrill, ShallowRatioWarns)
{
    auto stock = skill::createCuboidStock(30.0, 30.0, 30.0);

    skill::gun_drill::Input in;
    in.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 15.0;
    in.position_y_mm = 15.0;
    in.diameter_mm   = 5.0;
    in.depth_mm      = 15.0;          // ratio = 3 — gun drill not justified
    auto r = skill::gun_drill::validate(*stock, in);
    EXPECT_TRUE(r.passed);             // warning only
    bool warn = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-GUNDRILL-RATIO" && f.severity == "warning") { warn = true; break; }
    EXPECT_TRUE(warn);

    // recognize on this shallow hole should NOT return a gun_drill
    // candidate (ratio < 10 → filtered out).
    auto out = skill::gun_drill::apply(*stock, in);
    auto cands = skill::gun_drill::recognize(*out.workpiece);
    EXPECT_TRUE(cands.empty())
        << "shallow holes should not be recognised as gun_drill";
}
