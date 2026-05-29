// @lat: [[process/test-strategy#skill round-trip]]
//
// encap_mold skill — 5-case sweep covering identity-geometry synthesis,
// DFM rejection of out-of-range pressure, metadata-only recognition.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/encap_mold.hpp"

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

skill::encap_mold::Input goodInput()
{
    skill::encap_mold::Input in;
    in.molding_compound      = "EMC_G700";
    in.cavity_count          = 64;
    in.transfer_pressure_kpa = 7000.0;
    return in;
}

}  // namespace

// ─── 1. Apply: geometry CLONED unchanged ──────────────────────────────────
TEST(SkillEncapMold, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    auto out = skill::encap_mold::apply(*stock, goodInput());
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("encap_mold"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. Validate REJECTS bad input ────────────────────────────────────────
TEST(SkillEncapMold, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto in = goodInput();
    in.transfer_pressure_kpa = 1000.0;     // < 3000 kPa
    auto r = skill::encap_mold::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-PRESSURE"));
    EXPECT_THROW(skill::encap_mold::apply(*stock, in), skill::SkillError);

    auto in2 = goodInput();
    in2.cavity_count = 500;                 // > 256
    auto r2 = skill::encap_mold::validate(*stock, in2);
    EXPECT_FALSE(r2.passed);
    EXPECT_TRUE(hasFinding(r2, "DFM-CAVITY"));
}

// ─── 3. Signature records kind + key params ──────────────────────────────
TEST(SkillEncapMold, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto in = goodInput();
    in.molding_compound      = "EMC_KMC184";
    in.cavity_count          = 128;
    in.transfer_pressure_kpa = 9500.0;

    auto out = skill::encap_mold::apply(*stock, in);

    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("encap_mold"));
    EXPECT_EQ(out.signature.pattern["molding_compound"].get<std::string>(),
              std::string("EMC_KMC184"));
    EXPECT_EQ(out.signature.pattern["cavity_count"].get<int>(), 128);
    EXPECT_NEAR(out.signature.pattern["transfer_pressure_kpa"].get<double>(),
                9500.0, 1e-9);
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
}

// ─── 4. Recognize via metadata replay ─────────────────────────────────────
TEST(SkillEncapMold, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    auto out = skill::encap_mold::apply(*stock, goodInput());

    auto cands = skill::encap_mold::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["molding_compound"].get<std::string>(),
              std::string("EMC_G700"));

    skill::Workpiece raw(out.workpiece->shape());
    auto cands2 = skill::encap_mold::recognize(raw);
    EXPECT_TRUE(cands2.empty())
        << "encap_mold cannot be recognized geometrically";
}

// ─── 5. SPECIFIC: per-unit cycle time scales inversely with cavities ──────
TEST(SkillEncapMold, MoreCavitiesShortensPerUnitCycle)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto fewIn = goodInput();
    fewIn.cavity_count = 16;
    auto fewOut = skill::encap_mold::apply(*stock, fewIn);

    auto manyIn = goodInput();
    manyIn.cavity_count = 256;
    auto manyOut = skill::encap_mold::apply(*stock, manyIn);

    EXPECT_LT(manyOut.signature.tooling.est_cycle_time_s,
              fewOut.signature.tooling.est_cycle_time_s);
    EXPECT_EQ(fewOut.signature.tooling.tool_type,
              std::string("transfer_mold_press"));
    EXPECT_NEAR(fewOut.signature.pattern["mold_temperature_c"].get<double>(),
                175.0, 1e-9);
}
