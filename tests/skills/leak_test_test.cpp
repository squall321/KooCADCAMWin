// @lat: [[process/test-strategy#skill round-trip]]
//
// leak_test skill — 5-case sweep covering identity-geometry synthesis,
// DFM error on non-positive pressure / leak-rate, helium-specific info
// finding, metadata-replay recognition.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/leak_test.hpp"

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

skill::leak_test::Input goodInput()
{
    skill::leak_test::Input in;
    in.test_type             = "pressure";
    in.test_pressure_pa      = 1.0e5;
    in.max_leak_rate_pa_m3_s = 1.0e-6;
    in.duration_s            = 60.0;
    in.result                = "pass";
    return in;
}

}  // namespace

// ─── 1. Apply: geometry CLONED unchanged ──────────────────────────────────
TEST(SkillLeakTest, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(100.0, 60.0, 30.0);
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    auto out = skill::leak_test::apply(*stock, goodInput());
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("leak_test"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. Validate REJECTS bad input (zero pressure, zero rate) ─────────────
TEST(SkillLeakTest, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(100.0, 60.0, 30.0);

    auto in = goodInput();
    in.test_pressure_pa = 0.0;       // not > 0
    auto r = skill::leak_test::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-INPUT"));
    EXPECT_THROW(skill::leak_test::apply(*stock, in), skill::SkillError);

    auto in2 = goodInput();
    in2.max_leak_rate_pa_m3_s = -1.0e-3;     // not > 0
    auto r2 = skill::leak_test::validate(*stock, in2);
    EXPECT_FALSE(r2.passed);
    EXPECT_TRUE(hasFinding(r2, "DFM-INPUT"));
}

// ─── 3. Signature records kind + test_type / max_leak_rate ───────────────
TEST(SkillLeakTest, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCuboidStock(100.0, 60.0, 30.0);

    auto in = goodInput();
    in.test_type             = "helium";
    in.test_pressure_pa      = 5.0e5;
    in.max_leak_rate_pa_m3_s = 1.0e-9;
    in.duration_s            = 300.0;
    in.result                = "pass";

    auto out = skill::leak_test::apply(*stock, in);

    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("leak_test"));
    EXPECT_EQ(out.signature.pattern["test_type"].get<std::string>(),
              std::string("helium"));
    EXPECT_NEAR(out.signature.pattern["test_pressure_pa"].get<double>(),
                5.0e5, 1e-3);
    EXPECT_NEAR(out.signature.pattern["max_leak_rate_pa_m3_s"].get<double>(),
                1.0e-9, 1e-15);
    EXPECT_EQ(out.signature.pattern["qa_category"].get<std::string>(),
              std::string("mass_spec_helium"));
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
}

// ─── 4. Recognize via metadata replay ─────────────────────────────────────
TEST(SkillLeakTest, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(100.0, 60.0, 30.0);
    auto out = skill::leak_test::apply(*stock, goodInput());

    auto cands = skill::leak_test::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["test_type"].get<std::string>(),
              std::string("pressure"));

    skill::Workpiece raw(out.workpiece->shape());
    auto cands2 = skill::leak_test::recognize(raw);
    EXPECT_TRUE(cands2.empty())
        << "leak_test cannot be recognized geometrically";
}

// ─── 5. SPECIFIC: helium + loose leak rate emits DFM-LEAK-HE info ─────────
TEST(SkillLeakTest, HeliumLooseRateEmitsInfoFinding)
{
    auto stock = skill::createCuboidStock(100.0, 60.0, 30.0);

    auto in = goodInput();
    in.test_type             = "helium";
    in.max_leak_rate_pa_m3_s = 1.0e-3;        // far above typical He floor 1e-9

    auto r = skill::leak_test::validate(*stock, in);
    EXPECT_TRUE(r.passed)
        << "loose helium acceptance is INFO, not blocking";
    EXPECT_TRUE(hasFinding(r, "DFM-LEAK-HE"));
    EXPECT_NO_THROW(skill::leak_test::apply(*stock, in));

    // Tooling type differentiation
    auto pressIn = goodInput();
    pressIn.test_type = "pressure";
    auto pressOut = skill::leak_test::apply(*stock, pressIn);
    EXPECT_EQ(pressOut.signature.tooling.tool_type,
              std::string("leak_decay_rig"));
    auto heOut = skill::leak_test::apply(*stock, in);
    EXPECT_EQ(heOut.signature.tooling.tool_type,
              std::string("helium_mass_spec"));
}
