// @lat: [[process/test-strategy#skill round-trip]]
//
// concrete_pour skill — 5-case sweep covering identity-geometry synthesis,
// DFM rejection of negative slump, metadata-only recognition, and the
// specific assertion that the additive volume is recorded as a negative
// stock_removed_mm3 in mm3 (m3 → mm3 = ×1e9).

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/concrete_pour.hpp"

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

skill::concrete_pour::Input goodInput()
{
    skill::concrete_pour::Input in;
    in.volume_m3  = 2.5;
    in.mix_design = "C25/30";
    in.slump_mm   = 100.0;
    return in;
}

}  // namespace

// ─── 1. Apply: geometry CLONED unchanged ──────────────────────────────────
TEST(SkillConcretePour, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    auto out = skill::concrete_pour::apply(*stock, goodInput());
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("concrete_pour"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
}

// ─── 2. Validate REJECTS bad input (negative slump) ───────────────────────
TEST(SkillConcretePour, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto in = goodInput();
    in.slump_mm = -10.0;                       // outside [0, 250]
    auto r = skill::concrete_pour::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-CONC-SLUMP"));
    EXPECT_THROW(skill::concrete_pour::apply(*stock, in), skill::SkillError);

    auto in2 = goodInput();
    in2.volume_m3 = 0.0;                       // ≤ 0
    auto r2 = skill::concrete_pour::validate(*stock, in2);
    EXPECT_FALSE(r2.passed);
    EXPECT_TRUE(hasFinding(r2, "DFM-INPUT"));

    auto in3 = goodInput();
    in3.mix_design = "";                       // empty
    auto r3 = skill::concrete_pour::validate(*stock, in3);
    EXPECT_FALSE(r3.passed);
    EXPECT_TRUE(hasFinding(r3, "DFM-INPUT"));
}

// ─── 3. Signature records kind + volume + mix + slump ────────────────────
TEST(SkillConcretePour, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto in = goodInput();
    in.volume_m3  = 5.0;
    in.mix_design = "C40/50";
    in.slump_mm   = 150.0;
    auto out = skill::concrete_pour::apply(*stock, in);

    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("concrete_pour"));
    EXPECT_NEAR(out.signature.pattern["volume_m3"].get<double>(), 5.0, 1e-9);
    EXPECT_EQ(out.signature.pattern["mix_design"].get<std::string>(),
              std::string("C40/50"));
    EXPECT_NEAR(out.signature.pattern["slump_mm"].get<double>(), 150.0, 1e-9);
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
}

// ─── 4. Recognize via metadata replay ─────────────────────────────────────
TEST(SkillConcretePour, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    auto out = skill::concrete_pour::apply(*stock, goodInput());

    auto cands = skill::concrete_pour::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["mix_design"].get<std::string>(),
              std::string("C25/30"));

    skill::Workpiece raw(out.workpiece->shape());
    auto cands2 = skill::concrete_pour::recognize(raw);
    EXPECT_TRUE(cands2.empty())
        << "concrete_pour cannot be recognized geometrically";
}

// ─── 5. SPECIFIC: ADDED volume recorded as negative mm3 (m3 × 1e9) ────────
TEST(SkillConcretePour, AdditiveVolumeRecordedAsNegativeMm3)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto in = goodInput();
    in.volume_m3 = 3.0;                        // 3 m3 = 3e9 mm3
    auto out = skill::concrete_pour::apply(*stock, in);

    // ADDED volume is recorded as negative stock_removed_mm3 (mass-balance
    // convention, mirrors spot_weld).
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, -3.0e9, 1.0);
    EXPECT_EQ(out.signature.tooling.tool_type, std::string("concrete_pump"));
}
