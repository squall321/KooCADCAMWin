// @lat: [[process/test-strategy#skill round-trip]]
//
// fertilize_apply — 5-case sweep covering identity-geometry synthesis,
// DFM gates on dose, NPK ratio parsing, and metadata signature replay.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/fertilize_apply.hpp"

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

skill::fertilize_apply::Input goodInput()
{
    skill::fertilize_apply::Input in;
    in.fertilizer_type = "compound";
    in.npk_ratio       = "20-10-10";
    in.dose_kg_ha      = 200.0;
    return in;
}

}  // namespace

// ── 1. Apply: geometry CLONED unchanged ──────────────────────────────────
TEST(SkillFertilizeApply, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    auto out = skill::fertilize_apply::apply(*stock, goodInput());
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("fertilize_apply"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ── 2. DFM rejects empty type, zero/over-cap dose ────────────────────────
TEST(SkillFertilizeApply, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto bad = goodInput();
    bad.fertilizer_type = "";                // invalid
    EXPECT_FALSE(skill::fertilize_apply::validate(*stock, bad).passed);
    EXPECT_THROW(skill::fertilize_apply::apply(*stock, bad), skill::SkillError);

    bad = goodInput();
    bad.dose_kg_ha = 0.0;                    // invalid
    EXPECT_FALSE(skill::fertilize_apply::validate(*stock, bad).passed);

    bad = goodInput();
    bad.dose_kg_ha = 5000.0;                 // > 2000 cap
    auto r = skill::fertilize_apply::validate(*stock, bad);
    EXPECT_FALSE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-FERT-DOSE"));
}

// ── 3. Signature records kind + type/npk/dose ────────────────────────────
TEST(SkillFertilizeApply, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto in = goodInput();
    in.fertilizer_type = "urea";
    in.npk_ratio       = "46-0-0";
    in.dose_kg_ha      = 150.0;

    auto out = skill::fertilize_apply::apply(*stock, in);
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("fertilize_apply"));
    EXPECT_EQ(out.signature.pattern["fertilizer_type"].get<std::string>(),
              std::string("urea"));
    EXPECT_EQ(out.signature.pattern["npk_ratio"].get<std::string>(),
              std::string("46-0-0"));
    EXPECT_TRUE(out.signature.pattern["npk_parsed"].get<bool>());
    EXPECT_NEAR(out.signature.pattern["n_pct"].get<double>(), 46.0, 1e-9);
    EXPECT_NEAR(out.signature.pattern["p_pct"].get<double>(),  0.0, 1e-9);
    EXPECT_NEAR(out.signature.pattern["k_pct"].get<double>(),  0.0, 1e-9);
    EXPECT_NEAR(out.signature.pattern["dose_kg_ha"].get<double>(), 150.0, 1e-9);
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
    EXPECT_EQ(out.signature.tooling.tool_type,
              std::string("fertilizer_broadcaster"));
}

// ── 4. Recognize via metadata replay ─────────────────────────────────────
TEST(SkillFertilizeApply, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    auto out = skill::fertilize_apply::apply(*stock, goodInput());

    auto cands = skill::fertilize_apply::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["npk_ratio"].get<std::string>(),
              std::string("20-10-10"));
    EXPECT_NEAR(cands[0].recovered_params["dose_kg_ha"].get<double>(),
                200.0, 1e-9);

    // Strip metadata → empty.
    skill::Workpiece raw(out.workpiece->shape());
    EXPECT_TRUE(skill::fertilize_apply::recognize(raw).empty());
}

// ── 5. SPECIFIC: NPK sum > 100 emits warning; non-parseable emits info ───
TEST(SkillFertilizeApply, NpkSumTooHighWarnsAndUnparseableInfo)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto hi = goodInput();
    hi.npk_ratio = "60-30-30";               // sum 120 > 100
    auto rHi = skill::fertilize_apply::validate(*stock, hi);
    EXPECT_TRUE(rHi.passed);
    bool warned = false;
    for (const auto& f : rHi.findings) {
        if (f.code == "DFM-FERT-NPK" && f.severity == "warning") {
            warned = true; break;
        }
    }
    EXPECT_TRUE(warned);

    auto unp = goodInput();
    unp.npk_ratio = "garbage";               // not parseable
    auto rUnp = skill::fertilize_apply::validate(*stock, unp);
    EXPECT_TRUE(rUnp.passed);
    EXPECT_TRUE(hasFinding(rUnp, "DFM-FERT-NPK"));

    // Apply succeeds and records npk_parsed=false.
    auto unpOut = skill::fertilize_apply::apply(*stock, unp);
    EXPECT_FALSE(unpOut.signature.pattern["npk_parsed"].get<bool>());
}
