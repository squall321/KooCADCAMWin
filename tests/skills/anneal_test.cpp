// @lat: [[process/test-strategy#skill round-trip]]
//
// anneal skill — heat-treatment no-op sweep.  Covers:
//   1. identity geometry on apply
//   2. signature carries material + temperature + cooling + expected_hardness_hb
//   3. DFM info-level on out-of-range temperature (still passes)
//   4. DFM error on non-positive hold_time_min
//   5. recognize via metadata replay
//   6. unknown material triggers info-level finding but does NOT block

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/anneal.hpp"

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
TEST(SkillAnneal, ApplyDoesNotChangeGeometry)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0, "carbon_steel_1045");
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    skill::anneal::Input in;
    in.material      = "carbon_steel_1045";
    in.temperature_c = 800.0;       // mid of 760–840 range
    in.hold_time_min = 60.0;
    in.cooling_rate  = "slow_furnace";

    auto out = skill::anneal::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("anneal"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. Signature records material + temperature + expected hardness ─────
TEST(SkillAnneal, SignatureRecordsMetadata)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0, "aluminum_6061");

    skill::anneal::Input in;
    in.material      = "aluminum_6061";
    in.temperature_c = 413.0;       // mid of 410–415 range
    in.hold_time_min = 120.0;
    in.cooling_rate  = "air";

    auto out = skill::anneal::apply(*stock, in);
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(), std::string("anneal"));
    EXPECT_EQ(out.signature.pattern["material"].get<std::string>(),
              std::string("aluminum_6061"));
    EXPECT_EQ(out.signature.pattern["cooling_rate"].get<std::string>(),
              std::string("air"));
    EXPECT_NEAR(out.signature.pattern["expected_hardness_hb"].get<double>(),
                30.0, 1e-6);
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
    EXPECT_EQ(out.signature.tooling.tool_type, std::string("furnace"));
}

// ─── 3. DFM info-level on out-of-range temperature — operation proceeds ──
TEST(SkillAnneal, OutOfRangeTemperatureIsInfoNotError)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0, "carbon_steel_1045");

    skill::anneal::Input in;
    in.material      = "carbon_steel_1045";
    in.temperature_c = 950.0;       // well above 760–840 range
    in.hold_time_min = 60.0;

    auto r = skill::anneal::validate(*stock, in);
    EXPECT_TRUE(r.passed)
        << "out-of-range temperature should NOT block (info-only)";
    EXPECT_TRUE(hasFinding(r, "DFM-HT-TEMP-HIGH"));

    // apply() should succeed.
    EXPECT_NO_THROW(skill::anneal::apply(*stock, in));
}

// ─── 4. DFM error on non-positive hold_time_min ──────────────────────────
TEST(SkillAnneal, DfmErrorOnZeroHoldTime)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0, "carbon_steel_1045");

    skill::anneal::Input in;
    in.material      = "carbon_steel_1045";
    in.temperature_c = 800.0;
    in.hold_time_min = 0.0;         // invalid

    auto r = skill::anneal::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_THROW(skill::anneal::apply(*stock, in), skill::SkillError);
}

// ─── 5. Recognize via metadata replay ─────────────────────────────────────
TEST(SkillAnneal, RecognizeFromMetadata)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0, "carbon_steel_1045");

    skill::anneal::Input in;
    in.material      = "carbon_steel_1045";
    in.temperature_c = 800.0;
    in.hold_time_min = 60.0;
    in.cooling_rate  = "slow_furnace";
    auto out = skill::anneal::apply(*stock, in);

    auto cands = skill::anneal::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["material"].get<std::string>(),
              std::string("carbon_steel_1045"));

    // Same workpiece stripped of metadata → recognize is empty.
    skill::Workpiece raw(out.workpiece->shape());
    auto cands2 = skill::anneal::recognize(raw);
    EXPECT_TRUE(cands2.empty())
        << "anneal cannot be recognized geometrically";
}

// ─── 6. Unknown material → info finding, not block ───────────────────────
TEST(SkillAnneal, UnknownMaterialEmitsInfoAndProceeds)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0, "exotic_alloy_x");

    skill::anneal::Input in;
    in.material      = "exotic_alloy_x";
    in.temperature_c = 600.0;
    in.hold_time_min = 60.0;

    auto r = skill::anneal::validate(*stock, in);
    EXPECT_TRUE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-HT-MATERIAL"));
    EXPECT_NO_THROW(skill::anneal::apply(*stock, in));
}
