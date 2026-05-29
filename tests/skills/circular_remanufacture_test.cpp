// @lat: [[process/test-strategy#skill round-trip]]
//
// circular_remanufacture skill — 5-case sweep covering identity-geometry
// synthesis, core_id + steps + warranty DFM gates, metadata signature
// replay, and the step_count derived field + cycle-time specific assertion.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/circular_remanufacture.hpp"

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
TEST(SkillCircularRemanufacture, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    skill::circular_remanufacture::Input in;
    in.core_id         = "MOTOR-A37";
    in.refurbish_steps = { "clean", "inspect", "replace_seals", "recertify" };
    in.warranty_months = 24;

    auto out = skill::circular_remanufacture::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("circular_remanufacture"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. DFM: empty core_id / empty steps / zero warranty rejected ────────
TEST(SkillCircularRemanufacture, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::circular_remanufacture::Input bad;
    bad.core_id         = "";                                    // invalid
    bad.refurbish_steps = { "clean", "inspect", "recertify" };
    bad.warranty_months = 12;
    auto r = skill::circular_remanufacture::validate(*stock, bad);
    EXPECT_FALSE(r.passed);
    EXPECT_THROW(skill::circular_remanufacture::apply(*stock, bad),
                 skill::SkillError);

    bad.core_id         = "CORE-X";
    bad.refurbish_steps = {};                                    // invalid
    EXPECT_FALSE(skill::circular_remanufacture::validate(*stock, bad).passed);

    bad.refurbish_steps = { "clean", "inspect", "recertify" };
    bad.warranty_months = 0;                                     // invalid
    EXPECT_FALSE(skill::circular_remanufacture::validate(*stock, bad).passed);
}

// ─── 3. Signature records kind + core_id + warranty + step_count ─────────
TEST(SkillCircularRemanufacture, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::circular_remanufacture::Input in;
    in.core_id         = "GBOX-7-2024";
    in.refurbish_steps = { "clean", "inspect", "replace_bearings",
                           "test", "recertify" };
    in.warranty_months = 36;

    auto out = skill::circular_remanufacture::apply(*stock, in);
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("circular_remanufacture"));
    EXPECT_EQ(out.signature.pattern["core_id"].get<std::string>(),
              std::string("GBOX-7-2024"));
    EXPECT_EQ(out.signature.pattern["warranty_months"].get<int>(), 36);
    EXPECT_EQ(out.signature.pattern["step_count"].get<int>(), 5);
    EXPECT_EQ(out.signature.pattern["refurbish_steps"].size(), 5u);
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
    EXPECT_EQ(out.signature.tooling.tool_type,
              std::string("remanufacture_cell"));
}

// ─── 4. Recognize via metadata replay ─────────────────────────────────────
TEST(SkillCircularRemanufacture, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::circular_remanufacture::Input in;
    in.core_id         = "PUMP-99";
    in.refurbish_steps = { "clean", "inspect", "replace_impeller", "recertify" };
    in.warranty_months = 18;

    auto out = skill::circular_remanufacture::apply(*stock, in);
    auto cands = skill::circular_remanufacture::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["core_id"].get<std::string>(),
              std::string("PUMP-99"));
    EXPECT_EQ(cands[0].recovered_params["warranty_months"].get<int>(), 18);

    skill::Workpiece raw(out.workpiece->shape());
    EXPECT_TRUE(skill::circular_remanufacture::recognize(raw).empty())
        << "circular_remanufacture cannot be recognized geometrically";
}

// ─── 5. SPECIFIC: cycle_time = 300 + 600 * step_count + warranty gate ────
TEST(SkillCircularRemanufacture, CycleTimeScalesWithSteps)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::circular_remanufacture::Input in;
    in.core_id         = "CORE-T1";
    in.refurbish_steps = { "clean", "inspect", "replace_seals",
                           "test", "recertify" };       // 5 steps
    in.warranty_months = 24;

    auto out = skill::circular_remanufacture::apply(*stock, in);
    // 300 + 600 * 5 = 3300 s
    EXPECT_NEAR(out.signature.tooling.est_cycle_time_s, 3300.0, 1e-9);

    // Too few steps → info finding but passes.
    skill::circular_remanufacture::Input thin;
    thin.core_id         = "CORE-T2";
    thin.refurbish_steps = { "clean", "recertify" };    // only 2 steps
    thin.warranty_months = 12;
    auto r = skill::circular_remanufacture::validate(*stock, thin);
    EXPECT_TRUE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-REMAN-STEPS"));

    // Over-long warranty → info finding but passes.
    skill::circular_remanufacture::Input gen;
    gen.core_id         = "CORE-T3";
    gen.refurbish_steps = { "clean", "inspect", "replace", "recertify" };
    gen.warranty_months = 96;                            // > 60 info threshold
    auto r2 = skill::circular_remanufacture::validate(*stock, gen);
    EXPECT_TRUE(r2.passed);
    EXPECT_TRUE(hasFinding(r2, "DFM-REMAN-WARRANTY"));
}
