// @lat: [[process/test-strategy#skill round-trip]]
//
// gas_purify — 5-case sweep covering identity-geometry synthesis,
// DFM rejection of bad purity, metadata-only recognition.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/gas_purify.hpp"

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

skill::gas_purify::Input goodInput()
{
    skill::gas_purify::Input in;
    in.gas_species         = "argon";
    in.purification_method = "getter";
    in.final_purity_ppb    = 10.0;
    return in;
}

}  // namespace

// ─── 1. Apply: geometry CLONED unchanged ──────────────────────────────────
TEST(SkillGasPurify, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    auto out = skill::gas_purify::apply(*stock, goodInput());
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("gas_purify"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. Validate REJECTS bad input (purity ≤ 0) ───────────────────────────
TEST(SkillGasPurify, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto in = goodInput();
    in.final_purity_ppb = 0.0;   // must be > 0
    auto r = skill::gas_purify::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-INPUT"));
    EXPECT_THROW(skill::gas_purify::apply(*stock, in), skill::SkillError);

    // empty gas species
    auto in2 = goodInput();
    in2.gas_species = "";
    auto r2 = skill::gas_purify::validate(*stock, in2);
    EXPECT_FALSE(r2.passed);
    EXPECT_TRUE(hasFinding(r2, "DFM-INPUT"));
}

// ─── 3. Signature records kind + key params ──────────────────────────────
TEST(SkillGasPurify, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto in = goodInput();
    in.gas_species         = "hydrogen";
    in.purification_method = "Pd_membrane";
    in.final_purity_ppb    = 1.0;

    auto out = skill::gas_purify::apply(*stock, in);

    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("gas_purify"));
    EXPECT_EQ(out.signature.pattern["gas_species"].get<std::string>(),
              std::string("hydrogen"));
    EXPECT_EQ(out.signature.pattern["purification_method"].get<std::string>(),
              std::string("Pd_membrane"));
    EXPECT_NEAR(out.signature.pattern["final_purity_ppb"].get<double>(),
                1.0, 1e-9);
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
}

// ─── 4. Recognize via metadata replay ─────────────────────────────────────
TEST(SkillGasPurify, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    auto out = skill::gas_purify::apply(*stock, goodInput());

    auto cands = skill::gas_purify::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["gas_species"].get<std::string>(),
              std::string("argon"));

    skill::Workpiece raw(out.workpiece->shape());
    auto cands2 = skill::gas_purify::recognize(raw);
    EXPECT_TRUE(cands2.empty())
        << "gas_purify cannot be recognized geometrically";
}

// ─── 5. SPECIFIC: purity grade ladder + Pd/non-H2 warning ─────────────────
TEST(SkillGasPurify, PurityGradeLadderAndPdWarning)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto in9N = goodInput(); in9N.final_purity_ppb = 0.5;
    auto in7N = goodInput(); in7N.final_purity_ppb = 50.0;
    EXPECT_EQ(skill::gas_purify::apply(*stock, in9N)
                  .signature.pattern["purity_grade"].get<std::string>(),
              std::string("UHP-9N"));
    EXPECT_EQ(skill::gas_purify::apply(*stock, in7N)
                  .signature.pattern["purity_grade"].get<std::string>(),
              std::string("UHP-7N"));

    // Pd_membrane on argon ⇒ warning but still passes.
    auto inBad = goodInput();
    inBad.gas_species         = "argon";
    inBad.purification_method = "Pd_membrane";
    auto r = skill::gas_purify::validate(*stock, inBad);
    EXPECT_TRUE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-GAS-METHOD"));
}
