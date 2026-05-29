// @lat: [[process/test-strategy#skill round-trip]]
//
// hydrogen_purify — 5-case sweep: identity-geometry synthesis, DFM gates
// on method / purity / flow, metadata replay, method → tool_type mapping.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/hydrogen_purify.hpp"

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

skill::hydrogen_purify::Input goodInput()
{
    skill::hydrogen_purify::Input in;
    in.method     = "PSA";
    in.purity_pct = 99.99;
    in.flow_nm3_h = 1000.0;
    return in;
}

}  // namespace

// ── 1. Apply: geometry CLONED unchanged ──────────────────────────────────
TEST(SkillHydrogenPurify, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 30.0);
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    auto out = skill::hydrogen_purify::apply(*stock, goodInput());
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("hydrogen_purify"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ── 2. Validate REJECTS bad input ────────────────────────────────────────
TEST(SkillHydrogenPurify, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 30.0);

    auto bad = goodInput();
    bad.purity_pct = 0.0;
    EXPECT_FALSE(skill::hydrogen_purify::validate(*stock, bad).passed);
    EXPECT_THROW(skill::hydrogen_purify::apply(*stock, bad), skill::SkillError);

    bad = goodInput();
    bad.purity_pct = 100.0;       // boundary excluded
    EXPECT_FALSE(skill::hydrogen_purify::validate(*stock, bad).passed);

    bad = goodInput();
    bad.flow_nm3_h = -1.0;
    EXPECT_FALSE(skill::hydrogen_purify::validate(*stock, bad).passed);
    EXPECT_THROW(skill::hydrogen_purify::apply(*stock, bad), skill::SkillError);
}

// ── 3. Signature records kind + key params ───────────────────────────────
TEST(SkillHydrogenPurify, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 30.0);

    skill::hydrogen_purify::Input in;
    in.method     = "cryogenic";
    in.purity_pct = 99.9999;
    in.flow_nm3_h = 12000.0;

    auto out = skill::hydrogen_purify::apply(*stock, in);

    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("hydrogen_purify"));
    EXPECT_EQ(out.signature.pattern["method"].get<std::string>(),
              std::string("cryogenic"));
    EXPECT_NEAR(out.signature.pattern["purity_pct"].get<double>(),
                99.9999, 1e-9);
    EXPECT_NEAR(out.signature.pattern["flow_nm3_h"].get<double>(),
                12000.0, 1e-9);
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
}

// ── 4. Recognize via metadata replay ─────────────────────────────────────
TEST(SkillHydrogenPurify, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 30.0);
    auto out = skill::hydrogen_purify::apply(*stock, goodInput());

    auto cands = skill::hydrogen_purify::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["method"].get<std::string>(),
              std::string("PSA"));
    EXPECT_NEAR(cands[0].recovered_params["purity_pct"].get<double>(),
                99.99, 1e-9);

    // Strip metadata → empty.
    skill::Workpiece raw(out.workpiece->shape());
    EXPECT_TRUE(skill::hydrogen_purify::recognize(raw).empty());
}

// ── 5. SPECIFIC: method selects tool_type + invalid method → INFO not block ─
TEST(SkillHydrogenPurify, MethodSelectsToolTypeAndInfoOnUnknown)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 30.0);

    skill::hydrogen_purify::Input psa = goodInput();
    psa.method = "PSA";
    auto outPsa = skill::hydrogen_purify::apply(*stock, psa);
    EXPECT_EQ(outPsa.signature.tooling.tool_type,
              std::string("psa_adsorber"));

    skill::hydrogen_purify::Input cryo = goodInput();
    cryo.method     = "cryogenic";
    cryo.flow_nm3_h = 10000.0;
    auto outCryo = skill::hydrogen_purify::apply(*stock, cryo);
    EXPECT_EQ(outCryo.signature.tooling.tool_type,
              std::string("cryo_distillation_unit"));

    skill::hydrogen_purify::Input ely = goodInput();
    ely.method     = "electrolysis";
    ely.flow_nm3_h = 100.0;
    auto outEly = skill::hydrogen_purify::apply(*stock, ely);
    EXPECT_EQ(outEly.signature.tooling.tool_type,
              std::string("pem_pd_membrane"));

    // Unknown method → INFO only.
    skill::hydrogen_purify::Input weird = goodInput();
    weird.method = "ionic_liquid";
    auto r = skill::hydrogen_purify::validate(*stock, weird);
    EXPECT_TRUE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-H2-METHOD"));
    EXPECT_NO_THROW(skill::hydrogen_purify::apply(*stock, weird));
}
