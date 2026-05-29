// @lat: [[process/test-strategy#skill round-trip]]
//
// ore_crush — comminution.  Metadata-only.
// Cases (5):
//   1. ApplyClonesGeometryUnchanged
//   2. ValidateRejectsBadInput
//   3. SignatureRecordsKindAndParams
//   4. RecognizeMetadataReplay
//   5. ReductionRatioComputedAndCrusherTypeMapped

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/ore_crush.hpp"

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

// ─── 1. Apply clones geometry unchanged ───────────────────────────────────
TEST(SkillOreCrush, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(300.0, 300.0, 100.0);
    const double volBefore = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    skill::ore_crush::Input in;
    in.feed_size_mm  = 800.0;
    in.target_p80_mm = 100.0;
    in.crusher_type  = "jaw";

    auto out = skill::ore_crush::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("ore_crush"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. Validate rejects bad input ────────────────────────────────────────
TEST(SkillOreCrush, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(150.0, 150.0, 50.0);

    skill::ore_crush::Input badFeed;
    badFeed.feed_size_mm  = 0.0;
    badFeed.target_p80_mm = 50.0;
    badFeed.crusher_type  = "jaw";
    {
        auto r = skill::ore_crush::validate(*stock, badFeed);
        EXPECT_FALSE(r.passed);
        EXPECT_TRUE(hasFinding(r, "DFM-INPUT"));
    }
    EXPECT_THROW(skill::ore_crush::apply(*stock, badFeed), skill::SkillError);

    skill::ore_crush::Input noReduction;
    noReduction.feed_size_mm  = 100.0;
    noReduction.target_p80_mm = 100.0;   // target >= feed
    noReduction.crusher_type  = "jaw";
    {
        auto r = skill::ore_crush::validate(*stock, noReduction);
        EXPECT_FALSE(r.passed);
        EXPECT_TRUE(hasFinding(r, "DFM-CRUSH-RATIO"));
    }
    EXPECT_THROW(skill::ore_crush::apply(*stock, noReduction), skill::SkillError);

    skill::ore_crush::Input badCrusher;
    badCrusher.feed_size_mm  = 500.0;
    badCrusher.target_p80_mm = 50.0;
    badCrusher.crusher_type  = "blender";
    {
        auto r = skill::ore_crush::validate(*stock, badCrusher);
        EXPECT_FALSE(r.passed);
        EXPECT_TRUE(hasFinding(r, "DFM-CRUSHER-TYPE"));
    }
}

// ─── 3. Signature records kind + key params ───────────────────────────────
TEST(SkillOreCrush, SignatureRecordsKindAndParams)
{
    auto stock = skill::createCuboidStock(200.0, 200.0, 80.0);

    skill::ore_crush::Input in;
    in.feed_size_mm  = 400.0;
    in.target_p80_mm = 80.0;
    in.crusher_type  = "cone";

    auto out = skill::ore_crush::apply(*stock, in);

    EXPECT_EQ(out.signature.skill_id, std::string("ore_crush"));
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("ore_crush"));
    EXPECT_NEAR(out.signature.pattern["feed_size_mm"].get<double>(),
                400.0, 1e-9);
    EXPECT_NEAR(out.signature.pattern["target_p80_mm"].get<double>(),
                80.0, 1e-9);
    EXPECT_EQ(out.signature.pattern["crusher_type"].get<std::string>(),
              std::string("cone"));
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
}

// ─── 4. Recognize metadata replay ─────────────────────────────────────────
TEST(SkillOreCrush, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(200.0, 200.0, 80.0);

    skill::ore_crush::Input in;
    in.feed_size_mm  = 600.0;
    in.target_p80_mm = 75.0;
    in.crusher_type  = "gyratory";

    auto out   = skill::ore_crush::apply(*stock, in);
    auto cands = skill::ore_crush::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["crusher_type"].get<std::string>(),
              std::string("gyratory"));
    EXPECT_NEAR(cands[0].recovered_params["feed_size_mm"].get<double>(),
                600.0, 1e-9);

    skill::Workpiece raw(out.workpiece->shape());
    auto cands2 = skill::ore_crush::recognize(raw);
    EXPECT_TRUE(cands2.empty());
}

// ─── 5. Specific: reduction ratio + crusher tooling mapping ──────────────
TEST(SkillOreCrush, ReductionRatioComputedAndCrusherTypeMapped)
{
    auto stock = skill::createCuboidStock(200.0, 200.0, 80.0);

    skill::ore_crush::Input in;
    in.feed_size_mm  = 800.0;
    in.target_p80_mm = 100.0;     // ratio 8.0
    in.crusher_type  = "impact";

    auto out = skill::ore_crush::apply(*stock, in);
    EXPECT_NEAR(out.signature.pattern["reduction_ratio"].get<double>(),
                8.0, 1e-9);
    EXPECT_EQ(out.signature.tooling.tool_type,
              std::string("impact_crusher"));
}
