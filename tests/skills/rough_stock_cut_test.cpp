// @lat: [[process/test-strategy#skill round-trip]]
//
// rough_stock_cut skill — 5-case sweep.  Metadata-only.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/rough_stock_cut.hpp"

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

// ─── 1. Apply: geometry unchanged + signature added ───────────────────────
TEST(SkillRoughStockCut, ApplyHandlesInput)
{
    auto stock = skill::createCuboidStock(100.0, 100.0, 20.0);
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();

    skill::rough_stock_cut::Input in;
    in.original_stock_size = { 1000.0, 100.0, 20.0 };
    in.cut_dimension       = { 100.0,  100.0, 20.0 };

    auto out = skill::rough_stock_cut::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);

    EXPECT_EQ(out.signature.skill_id, std::string("rough_stock_cut"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
}

// ─── 2. Validate: cut bigger than original rejected ───────────────────────
TEST(SkillRoughStockCut, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(100.0, 100.0, 20.0);

    skill::rough_stock_cut::Input in;
    in.original_stock_size = { 50.0, 100.0, 20.0 };
    in.cut_dimension       = { 100.0, 100.0, 20.0 };   // cut > orig

    auto r = skill::rough_stock_cut::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_THROW(skill::rough_stock_cut::apply(*stock, in), skill::SkillError);
}

// ─── 3. Signature records sizes + waste_pct ──────────────────────────────
TEST(SkillRoughStockCut, SignatureRecordsKindAndKeys)
{
    auto stock = skill::createCuboidStock(100.0, 100.0, 20.0);

    skill::rough_stock_cut::Input in;
    in.original_stock_size = { 1000.0, 100.0, 20.0 };
    in.cut_dimension       = { 100.0,  100.0, 20.0 };

    auto out = skill::rough_stock_cut::apply(*stock, in);
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("rough_stock_cut"));
    ASSERT_TRUE(out.signature.pattern.contains("original_stock_size"));
    ASSERT_TRUE(out.signature.pattern.contains("cut_dimension"));
    ASSERT_TRUE(out.signature.pattern.contains("waste_pct"));
    // Cut is 1/10 of the original by volume → waste = 90 %.
    EXPECT_NEAR(out.signature.pattern["waste_pct"].get<double>(), 90.0, 0.01);
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
}

// ─── 4. Recognize via metadata replay ─────────────────────────────────────
TEST(SkillRoughStockCut, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(100.0, 100.0, 20.0);

    skill::rough_stock_cut::Input in;
    in.original_stock_size = { 200.0, 100.0, 20.0 };
    in.cut_dimension       = { 100.0, 100.0, 20.0 };
    auto out = skill::rough_stock_cut::apply(*stock, in);

    auto cands = skill::rough_stock_cut::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["original_stock_size"].size(), 3u);

    skill::Workpiece raw(out.workpiece->shape());
    auto cands2 = skill::rough_stock_cut::recognize(raw);
    EXPECT_TRUE(cands2.empty());
}

// ─── 5. Specific assertion: high waste_pct emits warning, still passes ───
TEST(SkillRoughStockCut, HighWasteIsWarningNotError)
{
    auto stock = skill::createCuboidStock(100.0, 100.0, 20.0);

    skill::rough_stock_cut::Input in;
    // Cut volume is 1 % of original → waste 99 % → > 50 % → warning.
    in.original_stock_size = { 10000.0, 100.0, 20.0 };
    in.cut_dimension       = { 100.0,    100.0, 20.0 };

    auto r = skill::rough_stock_cut::validate(*stock, in);
    EXPECT_TRUE(r.passed);
    EXPECT_TRUE(hasFinding(r, "RC-WASTE-HIGH"));
    EXPECT_NO_THROW(skill::rough_stock_cut::apply(*stock, in));
}
