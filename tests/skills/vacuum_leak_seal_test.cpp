// @lat: [[process/test-strategy#skill round-trip]]
//
// vacuum_leak_seal — 5-case sweep covering identity-geometry synthesis,
// DFM rejection of gross leak, metadata-only recognition.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/vacuum_leak_seal.hpp"

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

skill::vacuum_leak_seal::Input goodInput()
{
    skill::vacuum_leak_seal::Input in;
    in.leak_rate_pa_m3_s      = 1.0e-9;
    in.seal_method            = "torr_seal_epoxy";
    in.ultimate_pressure_torr = 1.0e-9;
    return in;
}

}  // namespace

// ─── 1. Apply: geometry CLONED unchanged ──────────────────────────────────
TEST(SkillVacuumLeakSeal, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    auto out = skill::vacuum_leak_seal::apply(*stock, goodInput());
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("vacuum_leak_seal"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. Validate REJECTS bad input (gross leak 1e-5 > 1e-7) ───────────────
TEST(SkillVacuumLeakSeal, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto in = goodInput();
    in.leak_rate_pa_m3_s = 1.0e-5;   // gross leak, > 1e-7
    auto r = skill::vacuum_leak_seal::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-LEAK-GROSS"));
    EXPECT_THROW(skill::vacuum_leak_seal::apply(*stock, in),
                 skill::SkillError);

    // ultimate pressure too high
    auto in2 = goodInput();
    in2.ultimate_pressure_torr = 1.0e-4;   // > 1e-6
    auto r2 = skill::vacuum_leak_seal::validate(*stock, in2);
    EXPECT_FALSE(r2.passed);
    EXPECT_TRUE(hasFinding(r2, "DFM-INPUT"));
}

// ─── 3. Signature records kind + key params ──────────────────────────────
TEST(SkillVacuumLeakSeal, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto in = goodInput();
    in.leak_rate_pa_m3_s      = 5.0e-10;
    in.seal_method            = "braze_patch";
    in.ultimate_pressure_torr = 1.0e-10;

    auto out = skill::vacuum_leak_seal::apply(*stock, in);

    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("vacuum_leak_seal"));
    EXPECT_NEAR(out.signature.pattern["leak_rate_pa_m3_s"].get<double>(),
                5.0e-10, 1e-20);
    EXPECT_EQ(out.signature.pattern["seal_method"].get<std::string>(),
              std::string("braze_patch"));
    EXPECT_NEAR(out.signature.pattern["ultimate_pressure_torr"].get<double>(),
                1.0e-10, 1e-20);
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
}

// ─── 4. Recognize via metadata replay ─────────────────────────────────────
TEST(SkillVacuumLeakSeal, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    auto out = skill::vacuum_leak_seal::apply(*stock, goodInput());

    auto cands = skill::vacuum_leak_seal::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["seal_method"].get<std::string>(),
              std::string("torr_seal_epoxy"));

    skill::Workpiece raw(out.workpiece->shape());
    auto cands2 = skill::vacuum_leak_seal::recognize(raw);
    EXPECT_TRUE(cands2.empty())
        << "vacuum_leak_seal cannot be recognized geometrically";
}

// ─── 5. SPECIFIC: vacuum regime classification (UHV vs HV vs XHV) ─────────
TEST(SkillVacuumLeakSeal, VacuumRegimeClassification)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto xhvIn = goodInput(); xhvIn.ultimate_pressure_torr = 1.0e-12;
    auto uhvIn = goodInput(); uhvIn.ultimate_pressure_torr = 1.0e-10;
    auto hvIn  = goodInput(); hvIn.ultimate_pressure_torr  = 1.0e-7;

    EXPECT_EQ(skill::vacuum_leak_seal::apply(*stock, xhvIn)
                  .signature.pattern["vacuum_regime"].get<std::string>(),
              std::string("XHV"));
    EXPECT_EQ(skill::vacuum_leak_seal::apply(*stock, uhvIn)
                  .signature.pattern["vacuum_regime"].get<std::string>(),
              std::string("UHV"));

    // HV ⇒ DFM-LEAK-UHV info finding (still passes).
    auto r = skill::vacuum_leak_seal::validate(*stock, hvIn);
    EXPECT_TRUE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-LEAK-UHV"));
    EXPECT_EQ(skill::vacuum_leak_seal::apply(*stock, hvIn)
                  .signature.pattern["vacuum_regime"].get<std::string>(),
              std::string("HV"));
}
