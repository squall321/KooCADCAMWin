// @lat: [[process/test-strategy#skill round-trip]]
//
// deep_emboss skill — deep raised feature with depth/thickness DFM check.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/deep_emboss.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

#include <cmath>

using namespace koocadcam;

namespace {
double volumeOf(const TopoDS_Shape& s) {
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}
}  // namespace

// ── 1. Apply: adds raised cylinder of expected volume ──────────────────
TEST(SkillDeepEmboss, ApplyChangesGeometryOrPreserves)
{
    auto stock = skill::createCuboidStock(100.0, 80.0, 1.5);

    skill::deep_emboss::Input in;
    in.position_x_mm   = 50.0;
    in.position_y_mm   = 40.0;
    in.diameter_mm     = 10.0;
    in.emboss_depth_mm = 3.0;        // 1 ≤ 3 ≤ 5 × 1.5 = 7.5

    auto out = skill::deep_emboss::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    const double expected = M_PI * 5.0 * 5.0 * 3.0;
    const double added = volumeOf(out.workpiece->shape()) - volumeOf(stock->shape());
    EXPECT_NEAR(added, expected, expected * 0.05);
}

// ── 2. Validate rejects bad input: depth < 1 mm + depth > 5 × t ───────
TEST(SkillDeepEmboss, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(100.0, 80.0, 1.5);

    // Too shallow (should use regular emboss).
    skill::deep_emboss::Input in;
    in.position_x_mm   = 50.0;
    in.position_y_mm   = 40.0;
    in.diameter_mm     = 10.0;
    in.emboss_depth_mm = 0.3;       // < 1.0 mm

    auto r = skill::deep_emboss::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool foundMin = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-DEMB-DEPTH-MIN") { foundMin = true; break; }
    EXPECT_TRUE(foundMin);
    EXPECT_THROW(skill::deep_emboss::apply(*stock, in), skill::SkillError);

    // Too deep.
    skill::deep_emboss::Input in2 = in;
    in2.emboss_depth_mm = 10.0;     // > 5 × 1.5 = 7.5
    auto r2 = skill::deep_emboss::validate(*stock, in2);
    EXPECT_FALSE(r2.passed);
    bool foundMax = false;
    for (const auto& f : r2.findings)
        if (f.code == "DFM-DEMB-DEPTH-MAX") { foundMax = true; break; }
    EXPECT_TRUE(foundMax);
}

// ── 3. Signature records kind, depth, and ratio ───────────────────────
TEST(SkillDeepEmboss, SignatureRecordsKind)
{
    auto stock = skill::createCuboidStock(100.0, 80.0, 1.5);

    skill::deep_emboss::Input in;
    in.position_x_mm   = 50.0;
    in.position_y_mm   = 40.0;
    in.diameter_mm     = 8.0;
    in.emboss_depth_mm = 3.0;

    auto out = skill::deep_emboss::apply(*stock, in);
    EXPECT_EQ(out.signature.skill_id, std::string("deep_emboss"));
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("deep_emboss"));
    EXPECT_NEAR(out.signature.pattern["emboss_depth_mm"].get<double>(), 3.0, 1e-6);
    EXPECT_NEAR(out.signature.pattern["depth_to_thickness_ratio"].get<double>(),
                3.0 / 1.5, 1e-6);
    EXPECT_TRUE(out.signature.pattern["raised_face_present"].get<bool>());
    EXPECT_EQ(out.signature.tooling.tool_type, std::string("deep_emboss_die"));
}

// ── 4. Recognize finds the deep protruding feature ────────────────────
TEST(SkillDeepEmboss, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(100.0, 80.0, 1.5);

    skill::deep_emboss::Input in;
    in.position_x_mm   = 60.0;
    in.position_y_mm   = 30.0;
    in.diameter_mm     = 8.0;
    in.emboss_depth_mm = 2.5;

    auto out = skill::deep_emboss::apply(*stock, in);
    auto cands = skill::deep_emboss::recognize(*out.workpiece);

    ASSERT_FALSE(cands.empty());
    const auto& c = cands.front();
    EXPECT_EQ(c.skill_id, std::string("deep_emboss"));
    EXPECT_NEAR(c.recovered_params["position_x_mm"].get<double>(), 60.0, 1e-3);
    EXPECT_NEAR(c.recovered_params["position_y_mm"].get<double>(), 30.0, 1e-3);
    EXPECT_NEAR(c.recovered_params["emboss_depth_mm"].get<double>(), 2.5, 0.05);
    EXPECT_NEAR(c.recovered_params["diameter_mm"].get<double>(), 8.0, 0.05);
}

// ── 5. Specific: depth/thickness ratio info finding ──────────────────
TEST(SkillDeepEmboss, RatioWarningInfoOnHighRatio)
{
    auto stock = skill::createCuboidStock(100.0, 80.0, 1.5);

    // Ratio between 4 and 5 → info finding but still passes.
    skill::deep_emboss::Input in;
    in.position_x_mm   = 50.0;
    in.position_y_mm   = 40.0;
    in.diameter_mm     = 10.0;
    in.emboss_depth_mm = 7.0;        // 7 / 1.5 = 4.67 ratio

    auto r = skill::deep_emboss::validate(*stock, in);
    EXPECT_TRUE(r.passed) << "ratio 4.67 should be info-only";
    bool foundRatio = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-DEMB-RATIO") { foundRatio = true; break; }
    EXPECT_TRUE(foundRatio);
    EXPECT_NO_THROW(skill::deep_emboss::apply(*stock, in));
}
