// @lat: [[process/test-strategy#skill round-trip]]
//
// plastic_pyrolysis skill — 5-case sweep covering identity-geometry synthesis,
// feedstock+temp+yield DFM gates, metadata signature replay, and the
// recovered_oil_kg = feedstock_kg * oil_yield_pct / 100 specific assertion.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/plastic_pyrolysis.hpp"

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

}  // namespace

// ─── 1. Apply: geometry is COMPLETELY unchanged ──────────────────────────
TEST(SkillPlasticPyrolysis, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    skill::plastic_pyrolysis::Input in;
    in.feedstock_kg  = 100.0;
    in.temp_c        = 500.0;
    in.oil_yield_pct = 65.0;

    auto out = skill::plastic_pyrolysis::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("plastic_pyrolysis"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. DFM: zero feedstock / bad yield rejected ─────────────────────────
TEST(SkillPlasticPyrolysis, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::plastic_pyrolysis::Input bad;
    bad.feedstock_kg  = 0.0;             // invalid
    bad.temp_c        = 500.0;
    bad.oil_yield_pct = 60.0;
    auto r = skill::plastic_pyrolysis::validate(*stock, bad);
    EXPECT_FALSE(r.passed);
    EXPECT_THROW(skill::plastic_pyrolysis::apply(*stock, bad), skill::SkillError);

    bad.feedstock_kg  = 100.0;
    bad.oil_yield_pct = 0.0;             // invalid (must be > 0)
    EXPECT_FALSE(skill::plastic_pyrolysis::validate(*stock, bad).passed);

    bad.oil_yield_pct = 120.0;           // invalid (> 100)
    EXPECT_FALSE(skill::plastic_pyrolysis::validate(*stock, bad).passed);
}

// ─── 3. Signature records kind + feedstock + temp + yield ────────────────
TEST(SkillPlasticPyrolysis, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::plastic_pyrolysis::Input in;
    in.feedstock_kg  = 250.0;
    in.temp_c        = 550.0;
    in.oil_yield_pct = 70.0;

    auto out = skill::plastic_pyrolysis::apply(*stock, in);
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("plastic_pyrolysis"));
    EXPECT_NEAR(out.signature.pattern["feedstock_kg"].get<double>(),  250.0, 1e-9);
    EXPECT_NEAR(out.signature.pattern["temp_c"].get<double>(),        550.0, 1e-9);
    EXPECT_NEAR(out.signature.pattern["oil_yield_pct"].get<double>(),  70.0, 1e-9);
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
    EXPECT_EQ(out.signature.tooling.tool_type, std::string("pyrolysis_reactor"));
}

// ─── 4. Recognize via metadata replay ─────────────────────────────────────
TEST(SkillPlasticPyrolysis, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::plastic_pyrolysis::Input in;
    in.feedstock_kg  = 150.0;
    in.temp_c        = 480.0;
    in.oil_yield_pct = 60.0;

    auto out = skill::plastic_pyrolysis::apply(*stock, in);
    auto cands = skill::plastic_pyrolysis::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_NEAR(cands[0].recovered_params["feedstock_kg"].get<double>(),
                150.0, 1e-9);
    EXPECT_NEAR(cands[0].recovered_params["temp_c"].get<double>(), 480.0, 1e-9);

    skill::Workpiece raw(out.workpiece->shape());
    EXPECT_TRUE(skill::plastic_pyrolysis::recognize(raw).empty())
        << "plastic_pyrolysis cannot be recognized geometrically";
}

// ─── 5. SPECIFIC: recovered_oil_kg = feedstock * yield / 100 + temp gates ─
TEST(SkillPlasticPyrolysis, RecoveredOilMatchesFormulaAndTempGates)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::plastic_pyrolysis::Input in;
    in.feedstock_kg  = 200.0;
    in.temp_c        = 500.0;
    in.oil_yield_pct = 75.0;

    auto out = skill::plastic_pyrolysis::apply(*stock, in);
    // 200 kg × 75 % = 150 kg recovered oil
    EXPECT_NEAR(out.signature.pattern["recovered_oil_kg"].get<double>(),
                150.0, 1e-9);

    // Sub-300 °C produces info finding, not error.
    skill::plastic_pyrolysis::Input cool;
    cool.feedstock_kg  = 50.0;
    cool.temp_c        = 250.0;       // below 300 °C
    cool.oil_yield_pct = 40.0;
    auto r = skill::plastic_pyrolysis::validate(*stock, cool);
    EXPECT_TRUE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-PYRO-TEMP"));

    // Above 800 °C also info, not error.
    skill::plastic_pyrolysis::Input hot;
    hot.feedstock_kg  = 50.0;
    hot.temp_c        = 900.0;
    hot.oil_yield_pct = 50.0;
    auto r2 = skill::plastic_pyrolysis::validate(*stock, hot);
    EXPECT_TRUE(r2.passed);
    EXPECT_TRUE(hasFinding(r2, "DFM-PYRO-TEMP"));
}
