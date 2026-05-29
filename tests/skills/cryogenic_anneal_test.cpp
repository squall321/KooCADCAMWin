// @lat: [[process/test-strategy#skill round-trip]]
//
// cryogenic_anneal — 5-case sweep covering identity-geometry synthesis,
// DFM rejection of out-of-range cold_temp, metadata-only recognition.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/cryogenic_anneal.hpp"

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

skill::cryogenic_anneal::Input goodInput()
{
    skill::cryogenic_anneal::Input in;
    in.cold_temp_k = 77.0;
    in.hold_h      = 24.0;
    in.ramp_k_min  = 1.0;
    return in;
}

}  // namespace

// ─── 1. Apply: geometry CLONED unchanged ──────────────────────────────────
TEST(SkillCryogenicAnneal, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    auto out = skill::cryogenic_anneal::apply(*stock, goodInput());
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("cryogenic_anneal"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. Validate REJECTS bad input (cold_temp 250 K out of cryo range) ────
TEST(SkillCryogenicAnneal, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto in = goodInput();
    in.cold_temp_k = 250.0;   // > 200 K cryo regime limit
    auto r = skill::cryogenic_anneal::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-CRYO-TEMP"));
    EXPECT_THROW(skill::cryogenic_anneal::apply(*stock, in), skill::SkillError);

    // hold_h = 0 ⇒ DFM-INPUT error
    auto in2 = goodInput();
    in2.hold_h = 0.0;
    auto r2 = skill::cryogenic_anneal::validate(*stock, in2);
    EXPECT_FALSE(r2.passed);
    EXPECT_TRUE(hasFinding(r2, "DFM-INPUT"));
}

// ─── 3. Signature records kind + key params ──────────────────────────────
TEST(SkillCryogenicAnneal, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto in = goodInput();
    in.cold_temp_k = 4.2;       // LHe
    in.hold_h      = 48.0;
    in.ramp_k_min  = 0.5;

    auto out = skill::cryogenic_anneal::apply(*stock, in);

    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("cryogenic_anneal"));
    EXPECT_NEAR(out.signature.pattern["cold_temp_k"].get<double>(), 4.2,  1e-9);
    EXPECT_NEAR(out.signature.pattern["hold_h"].get<double>(),      48.0, 1e-9);
    EXPECT_NEAR(out.signature.pattern["ramp_k_min"].get<double>(),  0.5,  1e-9);
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
    EXPECT_EQ(out.signature.tooling.tool_type,
              std::string("cryogenic_chamber"));
}

// ─── 4. Recognize via metadata replay ─────────────────────────────────────
TEST(SkillCryogenicAnneal, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    auto out = skill::cryogenic_anneal::apply(*stock, goodInput());

    auto cands = skill::cryogenic_anneal::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_NEAR(cands[0].recovered_params["cold_temp_k"].get<double>(),
                77.0, 1e-9);

    skill::Workpiece raw(out.workpiece->shape());
    auto cands2 = skill::cryogenic_anneal::recognize(raw);
    EXPECT_TRUE(cands2.empty())
        << "cryogenic_anneal cannot be recognized geometrically";
}

// ─── 5. SPECIFIC: cycle time scales with ramp rate ────────────────────────
TEST(SkillCryogenicAnneal, FastRampShortensCycleTime)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto slowIn = goodInput();
    slowIn.ramp_k_min = 0.5;
    auto slowOut = skill::cryogenic_anneal::apply(*stock, slowIn);

    auto fastIn = goodInput();
    fastIn.ramp_k_min = 4.0;   // still ≤ 5 K/min, no warning
    auto fastOut = skill::cryogenic_anneal::apply(*stock, fastIn);

    EXPECT_LT(fastOut.signature.tooling.est_cycle_time_s,
              slowOut.signature.tooling.est_cycle_time_s);

    // Excessive ramp triggers thermal-shock warning (but still passes).
    auto shockIn = goodInput();
    shockIn.ramp_k_min = 10.0;
    auto shockR = skill::cryogenic_anneal::validate(*stock, shockIn);
    EXPECT_TRUE(shockR.passed);
    EXPECT_TRUE(hasFinding(shockR, "DFM-CRYO-RAMP"));
}
