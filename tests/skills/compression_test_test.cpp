// @lat: [[process/test-strategy#skill round-trip]]
//
// compression_test skill — universal-testing-machine no-op sweep.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/compression_test.hpp"

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

skill::compression_test::Input goodInput()
{
    skill::compression_test::Input in;
    in.peak_load_kn    = 10.0;
    in.displacement_mm = 2.0;
    in.rate_mm_min     = 5.0;
    return in;
}

}  // namespace

// ─── 1. Apply: geometry CLONED unchanged ─────────────────────────────────
TEST(SkillCompressionTest, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    auto out = skill::compression_test::apply(*stock, goodInput());
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("compression_test"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. Validate REJECTS bad input (zero rate) ───────────────────────────
TEST(SkillCompressionTest, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto in = goodInput();
    in.rate_mm_min = 0.0;                  // invalid

    auto r = skill::compression_test::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-INPUT"));
    EXPECT_THROW(skill::compression_test::apply(*stock, in), skill::SkillError);

    auto in2 = goodInput();
    in2.peak_load_kn = -5.0;
    auto r2 = skill::compression_test::validate(*stock, in2);
    EXPECT_FALSE(r2.passed);
}

// ─── 3. Signature records kind + load/disp/rate ──────────────────────────
TEST(SkillCompressionTest, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto in = goodInput();
    in.peak_load_kn    = 50.0;
    in.displacement_mm = 3.5;
    in.rate_mm_min     = 12.0;

    auto out = skill::compression_test::apply(*stock, in);

    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("compression_test"));
    EXPECT_NEAR(out.signature.pattern["peak_load_kn"].get<double>(),   50.0, 1e-9);
    EXPECT_NEAR(out.signature.pattern["displacement_mm"].get<double>(), 3.5, 1e-9);
    EXPECT_NEAR(out.signature.pattern["rate_mm_min"].get<double>(),    12.0, 1e-9);
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
    EXPECT_GT(out.signature.pattern["compressive_stress_proxy_mpa"].get<double>(),
              0.0);
}

// ─── 4. Recognize via metadata replay ─────────────────────────────────────
TEST(SkillCompressionTest, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    auto out = skill::compression_test::apply(*stock, goodInput());

    auto cands = skill::compression_test::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_NEAR(cands[0].recovered_params["peak_load_kn"].get<double>(),
                10.0, 1e-9);

    skill::Workpiece raw(out.workpiece->shape());
    auto cands2 = skill::compression_test::recognize(raw);
    EXPECT_TRUE(cands2.empty())
        << "compression_test cannot be recognized geometrically";
}

// ─── 5. SPECIFIC: cycle time ≈ displacement / rate × 60 ──────────────────
TEST(SkillCompressionTest, CycleTimeMatchesDisplacementOverRate)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto in = goodInput();
    in.displacement_mm = 6.0;
    in.rate_mm_min     = 3.0;     // expect 2 min × 60 = 120 s

    auto out = skill::compression_test::apply(*stock, in);
    EXPECT_NEAR(out.signature.tooling.est_cycle_time_s, 120.0, 1e-6);
    EXPECT_EQ(out.signature.tooling.tool_type,
              std::string("universal_testing_machine"));
}
