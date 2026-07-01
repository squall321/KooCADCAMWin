// @lat: [[process/test-strategy#skill round-trip]]
//
// micro_drill — sub-0.8 mm holes with strict depth/dia ≤ 5.
//
// Cases (5):
//   1. Synthesis on a sub-0.8 mm hole removes the expected volume.
//   2. DFM rejects diameters < 0.1 mm and depth/dia > 5; warns at dia ≥ 0.8.
//   3. Recognize finds the sub-0.8 mm cylindrical face with confidence 0.85.
//   4. STEP round-trip preserves recognition.
//   5. Regular-size hole (1 mm) does NOT trigger micro_drill recognition.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/micro_drill.hpp"

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

// ─── 1. Synthesis: 0.5 mm dia × 1.5 mm depth → π r² depth volume removed ──
TEST(SkillMicroDrill, ApplyRemovesTinyVolume)
{
    auto stock = skill::createCuboidStock(20.0, 20.0, 5.0);

    skill::micro_drill::Input in;
    in.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1), 5.0, "largest" };
    in.position_x_mm = 10.0;
    in.position_y_mm = 10.0;
    in.axis_dir      = gp_Dir(0, 0, -1);
    in.diameter_mm   = 0.5;
    in.depth_mm      = 1.5;       // ratio = 3 → OK (≤ 5)

    auto out = skill::micro_drill::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    const double approx = M_PI * 0.25 * 0.25 * 1.5;
    const double diff = volumeOf(stock->shape()) - volumeOf(out.workpiece->shape());
    EXPECT_NEAR(diff, approx, approx * 0.10);   // looser tol for tiny features

    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_EQ(out.signature.skill_id, std::string("micro_drill"));
    EXPECT_EQ(out.signature.tooling.tool_type, std::string("micro_drill"));
}

// ─── 2. DFM: < 0.1 mm error, ≥ 0.8 mm warning, depth/dia > 5 error ────────
TEST(SkillMicroDrill, ValidateDFM)
{
    auto stock = skill::createCuboidStock(20.0, 20.0, 5.0);

    // dia 0.05 mm → DFM-MICRO-DIAMIN error
    {
        skill::micro_drill::Input in;
        in.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
        in.position_x_mm = 10.0;
        in.position_y_mm = 10.0;
        in.diameter_mm   = 0.05;
        in.depth_mm      = 0.1;
        auto r = skill::micro_drill::validate(*stock, in);
        EXPECT_FALSE(r.passed);
        bool found = false;
        for (const auto& f : r.findings)
            if (f.code == "DFM-MICRO-DIAMIN") { found = true; break; }
        EXPECT_TRUE(found);
        EXPECT_THROW(skill::micro_drill::apply(*stock, in), skill::SkillError);
    }

    // dia 1.0 → DFM-MICRO-DIAMAX warning (still passes)
    {
        skill::micro_drill::Input in;
        in.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
        in.position_x_mm = 10.0;
        in.position_y_mm = 10.0;
        in.diameter_mm   = 1.0;
        in.depth_mm      = 2.0;
        auto r = skill::micro_drill::validate(*stock, in);
        EXPECT_TRUE(r.passed);
        bool warn = false;
        for (const auto& f : r.findings)
            if (f.code == "DFM-MICRO-DIAMAX" && f.severity == "warning") { warn = true; break; }
        EXPECT_TRUE(warn);
    }

    // depth/dia > 5 → DFM-MICRO-RATIO error
    {
        skill::micro_drill::Input in;
        in.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
        in.position_x_mm = 10.0;
        in.position_y_mm = 10.0;
        in.diameter_mm   = 0.4;
        in.depth_mm      = 3.0;       // ratio = 7.5 > 5
        auto r = skill::micro_drill::validate(*stock, in);
        EXPECT_FALSE(r.passed);
        bool found = false;
        for (const auto& f : r.findings)
            if (f.code == "DFM-MICRO-RATIO") { found = true; break; }
        EXPECT_TRUE(found);
    }
}

// ─── 3. Recognize finds the sub-0.8 mm hole with high confidence ──────────
TEST(SkillMicroDrill, RecognizeFindsSubMmHole)
{
    auto stock = skill::createCuboidStock(20.0, 20.0, 5.0);

    skill::micro_drill::Input in;
    in.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 10.0;
    in.position_y_mm = 10.0;
    in.diameter_mm   = 0.6;
    in.depth_mm      = 2.0;

    auto out = skill::micro_drill::apply(*stock, in);
    auto cands = skill::micro_drill::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);

    const auto& c = cands.front();
    EXPECT_EQ(c.skill_id, std::string("micro_drill"));
    EXPECT_NEAR(c.confidence, 0.85, 1e-6);
    EXPECT_NEAR(c.recovered_params["diameter_mm"].get<double>(), 0.6, 1e-3);
    EXPECT_NEAR(c.recovered_params["depth_mm"].get<double>(),    2.0, 1e-3);
    // Entry Z must be the MOUTH (top face Z=5), NOT the blind bottom (Z=3).
    // OCCT stores the drilled cylinder axis pointing -Z, so a max-projection pick
    // would wrongly stamp the bottom; entry must be resolved by adjacency.
    ASSERT_TRUE(c.recovered_params.contains("position_z_mm"));
    EXPECT_NEAR(c.recovered_params["position_z_mm"].get<double>(), 5.0, 1e-3)
        << "recovered entry Z must be the mouth (5), not the blind bottom (3)";
}

// ─── 4. STEP round-trip ──────────────────────────────────────────────────
TEST(SkillMicroDrill, RoundTripViaStep)
{
    auto stock = skill::createCuboidStock(20.0, 20.0, 5.0);

    skill::micro_drill::Input in;
    in.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 12.0;
    in.position_y_mm = 8.0;
    in.diameter_mm   = 0.5;
    in.depth_mm      = 2.0;

    auto synth = skill::micro_drill::apply(*stock, in);
    const fs::path stepPath = fs::temp_directory_path() / "micro_drill_roundtrip.step";
    std::string err;
    ASSERT_TRUE(io::StepIO::write(synth.workpiece->shape(), stepPath, err)) << err;

    auto reimp = io::StepIO::read(stepPath, err);
    ASSERT_TRUE(reimp.has_value()) << err;
    skill::Workpiece reim(*reimp);

    auto cands = skill::micro_drill::recognize(reim);
    ASSERT_GE(cands.size(), 1u);
    const auto& c = cands.front();
    EXPECT_NEAR(c.recovered_params["diameter_mm"].get<double>(), in.diameter_mm, 1e-2);
    EXPECT_NEAR(c.recovered_params["depth_mm"].get<double>(),    in.depth_mm,    1e-2);

    auto stock2 = skill::createCuboidStock(20.0, 20.0, 5.0);
    auto resynth = skill::micro_drill::apply(*stock2, in);
    EXPECT_EQ(synth.workpiece->faceCount(), resynth.workpiece->faceCount());
}

// ─── 5. Regular-size hole (1 mm) is NOT recognised as micro_drill ─────────
TEST(SkillMicroDrill, RegularSizeNotRecognised)
{
    auto stock = skill::createCuboidStock(20.0, 20.0, 5.0);

    // dia = 1.0 mm (≥ 0.8) — outside the micro envelope.  DFM warns but
    // synthesis still works; recognize() must NOT emit a candidate for it.
    skill::micro_drill::Input in;
    in.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 10.0;
    in.position_y_mm = 10.0;
    in.diameter_mm   = 1.0;
    in.depth_mm      = 2.0;
    auto out = skill::micro_drill::apply(*stock, in);   // warning only
    auto cands = skill::micro_drill::recognize(*out.workpiece);
    EXPECT_TRUE(cands.empty())
        << "1 mm holes should not be recognised as micro_drill (envelope < 0.8 mm)";
}
