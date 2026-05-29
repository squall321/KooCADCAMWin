// @lat: [[process/test-strategy#skill round-trip]]
//
// cell_seal skill — battery-manufacturing metadata-only sweep.
// Covers:
//   1. apply clones the workpiece without touching geometry
//   2. validate rejects unknown method / failing leak rate
//   3. signature records kind + seal_method + leak rate
//   4. recognize replays metadata only
//   5. marginal leak rate (1e-9 < x ≤ 1e-8) emits info but proceeds

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/cell_seal.hpp"

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
TEST(SkillCellSeal, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCylindricalStock(21.0, 70.0, "nickel_plated_steel_can");
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    skill::cell_seal::Input in;
    in.seal_method   = "weld";
    in.leak_pa_m3_s  = 1.0e-10;
    in.weld_energy_j = 25.0;
    in.seal_temp_c   = 165.0;

    auto out = skill::cell_seal::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("cell_seal"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. DFM: rejects bad inputs ──────────────────────────────────────────
TEST(SkillCellSeal, ValidateRejectsBadInput)
{
    auto stock = skill::createCylindricalStock(21.0, 70.0);

    // 2a. unknown seal method
    skill::cell_seal::Input badMethod;
    badMethod.seal_method  = "glue";
    badMethod.leak_pa_m3_s = 1.0e-10;
    auto r1 = skill::cell_seal::validate(*stock, badMethod);
    EXPECT_FALSE(r1.passed);
    EXPECT_TRUE(hasFinding(r1, "DFM-SEAL-METHOD"));
    EXPECT_THROW(skill::cell_seal::apply(*stock, badMethod),
                 skill::SkillError);

    // 2b. leak rate fails hermetic spec
    skill::cell_seal::Input leaky;
    leaky.seal_method  = "weld";
    leaky.leak_pa_m3_s = 1.0e-6;        // far above 1e-8 threshold
    auto r2 = skill::cell_seal::validate(*stock, leaky);
    EXPECT_FALSE(r2.passed);
    EXPECT_TRUE(hasFinding(r2, "DFM-SEAL-LEAK"));
    EXPECT_THROW(skill::cell_seal::apply(*stock, leaky),
                 skill::SkillError);

    // 2c. negative leak rate is nonsense
    skill::cell_seal::Input negLeak;
    negLeak.seal_method  = "weld";
    negLeak.leak_pa_m3_s = -1.0e-10;
    auto r3 = skill::cell_seal::validate(*stock, negLeak);
    EXPECT_FALSE(r3.passed);
    EXPECT_TRUE(hasFinding(r3, "DFM-INPUT"));
}

// ─── 3. Signature records kind + key parameters ──────────────────────────
TEST(SkillCellSeal, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCuboidStock(70.0, 100.0, 6.0, "Al_pouch_laminate");

    skill::cell_seal::Input in;
    in.seal_method   = "heat";
    in.leak_pa_m3_s  = 5.0e-10;
    in.weld_energy_j = 0.0;
    in.seal_temp_c   = 175.0;

    auto out = skill::cell_seal::apply(*stock, in);

    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("cell_seal"));
    EXPECT_EQ(out.signature.pattern["seal_method"].get<std::string>(),
              std::string("heat"));
    EXPECT_NEAR(out.signature.pattern["leak_pa_m3_s"].get<double>(),
                5.0e-10, 1e-20);
    EXPECT_NEAR(out.signature.pattern["seal_temp_c"].get<double>(),
                175.0, 1e-6);
    // weld_energy_j is 0 for heat seal
    EXPECT_NEAR(out.signature.pattern["weld_energy_j"].get<double>(),
                0.0, 1e-12);
    EXPECT_TRUE(out.signature.pattern["hermetic_pass"].get<bool>());
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
    EXPECT_EQ(out.signature.tooling.tool_type,
              std::string("heat_seal_jaw"));
}

// ─── 4. Recognize replays metadata only ──────────────────────────────────
TEST(SkillCellSeal, RecognizeMetadataReplay)
{
    auto stock = skill::createCylindricalStock(21.0, 70.0);

    skill::cell_seal::Input in;
    in.seal_method   = "crimp";
    in.leak_pa_m3_s  = 8.0e-10;
    in.weld_energy_j = 0.0;
    in.seal_temp_c   = 0.0;

    auto out = skill::cell_seal::apply(*stock, in);

    auto cands = skill::cell_seal::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["seal_method"].get<std::string>(),
              std::string("crimp"));
    EXPECT_NEAR(cands[0].recovered_params["leak_pa_m3_s"].get<double>(),
                8.0e-10, 1e-20);

    skill::Workpiece raw(out.workpiece->shape());
    auto empty = skill::cell_seal::recognize(raw);
    EXPECT_TRUE(empty.empty())
        << "cell_seal cannot be recognized geometrically";
}

// ─── 5. Specific: marginal leak (between 1e-9 and 1e-8) info-only ───────
TEST(SkillCellSeal, MarginalLeakEmitsWarningAndProceeds)
{
    auto stock = skill::createCylindricalStock(21.0, 70.0);

    skill::cell_seal::Input marginal;
    marginal.seal_method   = "weld";
    marginal.leak_pa_m3_s  = 5.0e-9;        // between 1e-9 and 1e-8
    marginal.weld_energy_j = 25.0;

    auto r = skill::cell_seal::validate(*stock, marginal);
    EXPECT_TRUE(r.passed)
        << "marginal leak is info-only, must not block";
    EXPECT_TRUE(hasFinding(r, "DFM-SEAL-LEAK-WARN"));
    EXPECT_NO_THROW(skill::cell_seal::apply(*stock, marginal));
}
