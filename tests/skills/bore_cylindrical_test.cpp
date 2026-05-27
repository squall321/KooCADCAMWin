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
