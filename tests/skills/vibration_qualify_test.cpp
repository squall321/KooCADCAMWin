// @lat: [[process/test-strategy#skill round-trip]]
//
// vibration_qualify skill — 5-case sweep covering identity-geometry
// synthesis, DFM rejection (freq min ≥ max), key-param signature,
// metadata-only recognition.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/vibration_qualify.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

using namespace koocadcam;

namespace {

double volumeOf(const TopoDS_Shape& s)
{
    GProp_GProps p;
    BRepGProp::VolumeProperties(s, p);
    return p.Mass();
}

bool hasFinding(const skill::DFMReport& r, const std::string& code)
{
    for (const auto& f : r.findings) if (f.code == code) return true;
    return false;
}

skill::vibration_qualify::Input goodInput()
{
    skill::vibration_qualify::Input in;
    in.freq_min_hz  = 20.0;
    in.freq_max_hz  = 2000.0;
    in.accel_g_rms  = 7.7;
    in.duration_min = 60.0;
    in.axes_count   = 3;
    in.result       = "pass";
    return in;
}

}  // namespace

// ─── 1. Apply: geometry CLONED unchanged ──────────────────────────────────
TEST(SkillVibrationQualify, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(80.0, 60.0, 20.0);
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    auto out = skill::vibration_qualify::apply(*stock, goodInput());
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("vibration_qualify"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. Validate REJECTS bad input (freq_max ≤ freq_min, accel = 0) ──────
TEST(SkillVibrationQualify, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(80.0, 60.0, 20.0);

    auto in = goodInput();
    in.freq_min_hz = 2000.0;
    in.freq_max_hz = 20.0;            // max ≤ min
    auto r = skill::vibration_qualify::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-INPUT"));
    EXPECT_THROW(skill::vibration_qualify::apply(*stock, in),
                 skill::SkillError);

    auto in2 = goodInput();
    in2.accel_g_rms = 0.0;            // not > 0
    auto r2 = skill::vibration_qualify::validate(*stock, in2);
    EXPECT_FALSE(r2.passed);
    EXPECT_TRUE(hasFinding(r2, "DFM-VIB-ACCEL"));

    auto in3 = goodInput();
    in3.duration_min = 0.5;           // < 1 min
    auto r3 = skill::vibration_qualify::validate(*stock, in3);
    EXPECT_FALSE(r3.passed);
    EXPECT_TRUE(hasFinding(r3, "DFM-INPUT"));
}

// ─── 3. Signature records kind + freq / accel / duration / axes ──────────
TEST(SkillVibrationQualify, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCuboidStock(80.0, 60.0, 20.0);

    auto in = goodInput();
    in.freq_min_hz  = 50.0;
    in.freq_max_hz  = 1500.0;
    in.accel_g_rms  = 11.0;
    in.duration_min = 120.0;
    in.axes_count   = 1;

    auto out = skill::vibration_qualify::apply(*stock, in);

    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("vibration_qualify"));
    EXPECT_NEAR(out.signature.pattern["freq_min_hz"].get<double>(),  50.0, 1e-9);
    EXPECT_NEAR(out.signature.pattern["freq_max_hz"].get<double>(), 1500.0, 1e-9);
    EXPECT_NEAR(out.signature.pattern["accel_g_rms"].get<double>(),  11.0, 1e-9);
    EXPECT_NEAR(out.signature.pattern["duration_min"].get<double>(), 120.0, 1e-9);
    EXPECT_EQ(out.signature.pattern["axes_count"].get<int>(), 1);
    EXPECT_EQ(out.signature.pattern["test_kind"].get<std::string>(),
              std::string("single_axis_sine"));
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());

    // freq_range_hz is a 2-tuple for downstream consumers
    ASSERT_TRUE(out.signature.pattern["freq_range_hz"].is_array());
    EXPECT_EQ(out.signature.pattern["freq_range_hz"].size(), 2u);
}

// ─── 4. Recognize via metadata replay ─────────────────────────────────────
TEST(SkillVibrationQualify, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(80.0, 60.0, 20.0);
    auto out = skill::vibration_qualify::apply(*stock, goodInput());

    auto cands = skill::vibration_qualify::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_NEAR(cands[0].recovered_params["accel_g_rms"].get<double>(),
                7.7, 1e-9);

    skill::Workpiece raw(out.workpiece->shape());
    auto cands2 = skill::vibration_qualify::recognize(raw);
    EXPECT_TRUE(cands2.empty())
        << "vibration_qualify cannot be recognized geometrically";
}

// ─── 5. SPECIFIC: 3-axis triples the per-axis dwell in cycle-time est ─────
TEST(SkillVibrationQualify, ThreeAxisCycleExceedsSingleAxis)
{
    auto stock = skill::createCuboidStock(80.0, 60.0, 20.0);

    auto oneIn = goodInput();
    oneIn.axes_count = 1;
    oneIn.duration_min = 30.0;
    auto oneOut = skill::vibration_qualify::apply(*stock, oneIn);

    auto threeIn = goodInput();
    threeIn.axes_count = 3;
    threeIn.duration_min = 30.0;
    auto threeOut = skill::vibration_qualify::apply(*stock, threeIn);

    // 3-axis sums duration across X+Y+Z; fixed setup/teardown is the same.
    // Δ should be ≈ 2 × duration_min × 60 = 3600 s
    const double delta = threeOut.signature.tooling.est_cycle_time_s
                       - oneOut.signature.tooling.est_cycle_time_s;
    EXPECT_NEAR(delta, 2.0 * 30.0 * 60.0, 1.0);

    EXPECT_EQ(threeOut.signature.pattern["test_kind"].get<std::string>(),
              std::string("tri_axis_random"));
    EXPECT_EQ(oneOut.signature.pattern["test_kind"].get<std::string>(),
              std::string("single_axis_sine"));
}
