// @lat: [[process/test-strategy#skill round-trip]]
//
// inverter_test skill — 5-case sweep.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/inverter_test.hpp"

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

skill::inverter_test::Input goodInput()
{
    skill::inverter_test::Input in;
    in.inverter_id    = "INV-EV-001";
    in.peak_power_kw  = 200.0;
    in.efficiency_pct = 96.5;
    return in;
}

}  // namespace

// ─── 1. Apply: geometry CLONED unchanged ──────────────────────────────────
TEST(SkillInverterTest, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(300.0, 200.0, 100.0);
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    auto out = skill::inverter_test::apply(*stock, goodInput());
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("inverter_test"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. Validate REJECTS bad input ────────────────────────────────────────
TEST(SkillInverterTest, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(300.0, 200.0, 100.0);

    auto in = goodInput();
    in.efficiency_pct = 110.0;            // > 100%
    auto r = skill::inverter_test::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-INV-EFF"));
    EXPECT_THROW(skill::inverter_test::apply(*stock, in), skill::SkillError);

    auto in2 = goodInput();
    in2.peak_power_kw = 0.0;
    auto r2 = skill::inverter_test::validate(*stock, in2);
    EXPECT_FALSE(r2.passed);
    EXPECT_TRUE(hasFinding(r2, "DFM-INPUT"));

    auto in3 = goodInput();
    in3.inverter_id = "";
    auto r3 = skill::inverter_test::validate(*stock, in3);
    EXPECT_FALSE(r3.passed);
    EXPECT_TRUE(hasFinding(r3, "DFM-INPUT"));
}

// ─── 3. Signature records kind + key params ───────────────────────────────
TEST(SkillInverterTest, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCuboidStock(300.0, 200.0, 100.0);

    auto in = goodInput();
    in.inverter_id    = "INV-EV-093";
    in.peak_power_kw  = 350.0;
    in.efficiency_pct = 97.5;

    auto out = skill::inverter_test::apply(*stock, in);
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("inverter_test"));
    EXPECT_EQ(out.signature.pattern["inverter_id"].get<std::string>(),
              std::string("INV-EV-093"));
    EXPECT_NEAR(out.signature.pattern["peak_power_kw"].get<double>(),
                350.0, 1e-9);
    EXPECT_NEAR(out.signature.pattern["efficiency_pct"].get<double>(),
                97.5, 1e-9);
    EXPECT_EQ(out.signature.pattern["test_result"].get<std::string>(),
              std::string("PASS"));
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
}

// ─── 4. Recognize via metadata replay ─────────────────────────────────────
TEST(SkillInverterTest, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(300.0, 200.0, 100.0);
    auto out = skill::inverter_test::apply(*stock, goodInput());

    auto cands = skill::inverter_test::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_NEAR(cands[0].recovered_params["efficiency_pct"].get<double>(),
                96.5, 1e-9);

    skill::Workpiece raw(out.workpiece->shape());
    auto cands2 = skill::inverter_test::recognize(raw);
    EXPECT_TRUE(cands2.empty());
}

// ─── 5. SPECIFIC: efficiency < 90% flagged as FAIL via info finding ──────
TEST(SkillInverterTest, LowEfficiencyMarkedFailButProceeds)
{
    auto stock = skill::createCuboidStock(300.0, 200.0, 100.0);

    auto in = goodInput();
    in.efficiency_pct = 85.0;             // below FAIL threshold

    // Info-level finding — validate still passes, apply must succeed.
    auto r = skill::inverter_test::validate(*stock, in);
    EXPECT_TRUE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-INV-EFF"));

    auto out = skill::inverter_test::apply(*stock, in);
    EXPECT_EQ(out.signature.pattern["test_result"].get<std::string>(),
              std::string("FAIL"));

    // Loss = 200 * (1 - 0.85) = 30 kW
    EXPECT_NEAR(out.signature.pattern["loss_kw"].get<double>(), 30.0, 1e-9);
}
