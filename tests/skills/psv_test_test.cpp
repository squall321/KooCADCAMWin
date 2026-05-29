// @lat: [[process/test-strategy#skill round-trip]]
//
// psv_test — 5-case sweep for pressure-safety-valve bench qualification.
// Metadata-only.  Verifies blowdown_pct envelope (2 – 15 %) and that the
// reseat pressure is derived correctly from set × (1 - blowdown/100).

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/psv_test.hpp"

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

}  // namespace

// ─── 1. Apply clones geometry unchanged ───────────────────────────────────
TEST(SkillPsvTest, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCylindricalStock(100.0, 200.0, "carbon_steel_1045");
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    skill::psv_test::Input in;
    in.set_pressure_kpa = 1000.0;
    in.blowdown_pct     = 7.0;

    auto out = skill::psv_test::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. Validate rejects bad input ────────────────────────────────────────
TEST(SkillPsvTest, ValidateRejectsBadInput)
{
    auto stock = skill::createCylindricalStock(100.0, 200.0);

    skill::psv_test::Input nonPos;
    nonPos.set_pressure_kpa = 0.0;     // invalid
    nonPos.blowdown_pct     = 5.0;
    auto rNP = skill::psv_test::validate(*stock, nonPos);
    EXPECT_FALSE(rNP.passed);
    EXPECT_THROW(skill::psv_test::apply(*stock, nonPos), skill::SkillError);

    skill::psv_test::Input tooLow;
    tooLow.set_pressure_kpa = 1000.0;
    tooLow.blowdown_pct     = 1.0;     // < 2 % chatter envelope
    auto rL = skill::psv_test::validate(*stock, tooLow);
    EXPECT_FALSE(rL.passed);
    EXPECT_THROW(skill::psv_test::apply(*stock, tooLow), skill::SkillError);

    skill::psv_test::Input tooHigh;
    tooHigh.set_pressure_kpa = 1000.0;
    tooHigh.blowdown_pct     = 20.0;   // > 15 % poor reseat
    auto rH = skill::psv_test::validate(*stock, tooHigh);
    EXPECT_FALSE(rH.passed);
}

// ─── 3. Signature records kind + params ───────────────────────────────────
TEST(SkillPsvTest, SignatureRecordsKindAndParams)
{
    auto stock = skill::createCylindricalStock(100.0, 200.0);

    skill::psv_test::Input in;
    in.set_pressure_kpa = 2500.0;
    in.blowdown_pct     = 10.0;

    auto out = skill::psv_test::apply(*stock, in);

    EXPECT_EQ(out.signature.skill_id, std::string("psv_test"));
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("psv_test"));
    EXPECT_NEAR(out.signature.pattern["set_pressure_kpa"].get<double>(),
                2500.0, 1e-6);
    EXPECT_NEAR(out.signature.pattern["blowdown_pct"].get<double>(),
                10.0, 1e-6);
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
    EXPECT_EQ(out.signature.tooling.tool_type,
              std::string("psv_test_stand"));
}

// ─── 4. Recognize via metadata replay ─────────────────────────────────────
TEST(SkillPsvTest, RecognizeMetadataReplay)
{
    auto stock = skill::createCylindricalStock(100.0, 200.0);

    skill::psv_test::Input in;
    in.set_pressure_kpa = 1500.0;
    in.blowdown_pct     = 6.0;

    auto out = skill::psv_test::apply(*stock, in);
    auto cands = skill::psv_test::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_NEAR(cands[0].recovered_params["set_pressure_kpa"].get<double>(),
                1500.0, 1e-6);

    skill::Workpiece raw(out.workpiece->shape());
    auto candsEmpty = skill::psv_test::recognize(raw);
    EXPECT_TRUE(candsEmpty.empty());
}

// ─── 5. Specific: reseat_pressure_kpa = set × (1 − blowdown/100) ──────────
// The reseat pressure is the safety-critical derived quantity and the
// certification record's primary downstream consumer.  Verify the maths.
TEST(SkillPsvTest, ReseatPressureFollowsBlowdownFormula)
{
    auto stock = skill::createCylindricalStock(100.0, 200.0);

    skill::psv_test::Input in;
    in.set_pressure_kpa = 2000.0;
    in.blowdown_pct     = 5.0;

    auto out = skill::psv_test::apply(*stock, in);
    const double expectedReseat = 2000.0 * (1.0 - 5.0 / 100.0);   // = 1900 kPa
    EXPECT_NEAR(out.signature.pattern["reseat_pressure_kpa"].get<double>(),
                expectedReseat, 1e-6);
}
