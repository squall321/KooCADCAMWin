// @lat: [[process/test-strategy#skill round-trip]]
//
// solar_array_deploy_test skill — non-destructive deployment campaign sweep.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/solar_array_deploy_test.hpp"

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

// ─── 1. Apply: geometry CLONED unchanged ──────────────────────────────────
TEST(SkillSolarArrayDeployTest, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(500.0, 2000.0, 50.0,
                                          "aluminum_6061");
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    skill::solar_array_deploy_test::Input in;
    in.array_id        = "SA-LEO-001";
    in.cycles_count    = 6;
    in.latch_torque_nm = 18.5;

    auto out = skill::solar_array_deploy_test::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id,
              std::string("solar_array_deploy_test"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. ValidateRejectsBadInput: zero cycles / negative torque → errors ──
TEST(SkillSolarArrayDeployTest, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(500.0, 2000.0, 50.0);

    skill::solar_array_deploy_test::Input in;
    in.array_id        = "SA-X";
    in.cycles_count    = 0;            // < 1
    in.latch_torque_nm = 10.0;

    auto r = skill::solar_array_deploy_test::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-SPACE-DEPLOY"));
    EXPECT_THROW(skill::solar_array_deploy_test::apply(*stock, in),
                 skill::SkillError);

    skill::solar_array_deploy_test::Input in2;
    in2.array_id        = "SA-Y";
    in2.cycles_count    = 3;
    in2.latch_torque_nm = -1.0;        // negative
    auto r2 = skill::solar_array_deploy_test::validate(*stock, in2);
    EXPECT_FALSE(r2.passed);
    EXPECT_TRUE(hasFinding(r2, "DFM-SPACE-LATCH"));
}

// ─── 3. Signature records kind + array_id + cycles + torque ──────────────
TEST(SkillSolarArrayDeployTest, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCuboidStock(500.0, 2000.0, 50.0);

    skill::solar_array_deploy_test::Input in;
    in.array_id        = "SA-GEO-7";
    in.cycles_count    = 10;
    in.latch_torque_nm = 22.0;

    auto out = skill::solar_array_deploy_test::apply(*stock, in);
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("solar_array_deploy_test"));
    EXPECT_EQ(out.signature.pattern["array_id"].get<std::string>(),
              std::string("SA-GEO-7"));
    EXPECT_EQ(out.signature.pattern["cycles_count"].get<int>(), 10);
    EXPECT_NEAR(out.signature.pattern["latch_torque_nm"].get<double>(),
                22.0, 1e-9);
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
}

// ─── 4. Recognize via metadata replay ─────────────────────────────────────
TEST(SkillSolarArrayDeployTest, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(500.0, 2000.0, 50.0);

    skill::solar_array_deploy_test::Input in;
    in.array_id        = "SA-CUBESAT-1";
    in.cycles_count    = 3;
    in.latch_torque_nm = 5.5;

    auto out  = skill::solar_array_deploy_test::apply(*stock, in);
    auto cands = skill::solar_array_deploy_test::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["array_id"].get<std::string>(),
              std::string("SA-CUBESAT-1"));

    skill::Workpiece raw(out.workpiece->shape());
    EXPECT_TRUE(skill::solar_array_deploy_test::recognize(raw).empty())
        << "solar_array_deploy_test cannot be recognized geometrically";
}

// ─── 5. SPECIFIC: cycle-time scales with cycles_count ────────────────────
TEST(SkillSolarArrayDeployTest, CycleTimeScalesWithCycles)
{
    auto stock = skill::createCuboidStock(500.0, 2000.0, 50.0);

    skill::solar_array_deploy_test::Input small;
    small.array_id        = "SA-A";
    small.cycles_count    = 2;
    small.latch_torque_nm = 10.0;

    skill::solar_array_deploy_test::Input big;
    big.array_id        = "SA-B";
    big.cycles_count    = 20;
    big.latch_torque_nm = 10.0;

    auto outSmall = skill::solar_array_deploy_test::apply(*stock, small);
    auto outBig   = skill::solar_array_deploy_test::apply(*stock, big);

    EXPECT_LT(outSmall.signature.tooling.est_cycle_time_s,
              outBig.signature.tooling.est_cycle_time_s);
    EXPECT_EQ(outSmall.signature.tooling.tool_type,
              std::string("gravity_offload_rig"));
}
