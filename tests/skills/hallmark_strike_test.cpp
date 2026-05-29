// @lat: [[process/test-strategy#skill round-trip]]
//
// hallmark_strike — statutory hallmark impression on jewelry.
// Metadata-only.  Cases (5):
//   1. ApplyClonesGeometryUnchanged
//   2. ValidateRejectsBadInput
//   3. SignatureRecordsKindAndParams
//   4. RecognizeMetadataReplay
//   5. SilverFamilyTaggedFor925Purity

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/hallmark_strike.hpp"

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
TEST(SkillHallmarkStrike, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(20.0, 5.0, 2.0);
    const double volBefore = volumeOf(stock->shape());
    const int faceBefore   = stock->faceCount();
    const int edgeBefore   = stock->edgeCount();

    skill::hallmark_strike::Input in;
    in.hallmark_id  = "KS-LON-925";
    in.alloy_purity = 925;
    in.force_kn     = 1.5;

    auto out = skill::hallmark_strike::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("hallmark_strike"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. Validate rejects bad input ────────────────────────────────────────
TEST(SkillHallmarkStrike, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(20.0, 5.0, 2.0);

    skill::hallmark_strike::Input emptyId;
    emptyId.hallmark_id  = "";
    emptyId.alloy_purity = 925;
    emptyId.force_kn     = 1.5;
    {
        auto r = skill::hallmark_strike::validate(*stock, emptyId);
        EXPECT_FALSE(r.passed);
        EXPECT_TRUE(hasFinding(r, "DFM-INPUT"));
    }
    EXPECT_THROW(skill::hallmark_strike::apply(*stock, emptyId),
                 skill::SkillError);

    skill::hallmark_strike::Input badPurity;
    badPurity.hallmark_id  = "X-1";
    badPurity.alloy_purity = 700;  // not a recognised millesimal
    badPurity.force_kn     = 1.5;
    {
        auto r = skill::hallmark_strike::validate(*stock, badPurity);
        EXPECT_FALSE(r.passed);
        EXPECT_TRUE(hasFinding(r, "DFM-PURITY"));
    }
    EXPECT_THROW(skill::hallmark_strike::apply(*stock, badPurity),
                 skill::SkillError);

    skill::hallmark_strike::Input zeroForce;
    zeroForce.hallmark_id  = "X-2";
    zeroForce.alloy_purity = 925;
    zeroForce.force_kn     = 0.0;
    {
        auto r = skill::hallmark_strike::validate(*stock, zeroForce);
        EXPECT_FALSE(r.passed);
        EXPECT_TRUE(hasFinding(r, "DFM-FORCE"));
    }
}

// ─── 3. Signature records kind + key params ───────────────────────────────
TEST(SkillHallmarkStrike, SignatureRecordsKindAndParams)
{
    auto stock = skill::createCuboidStock(20.0, 5.0, 2.0);

    skill::hallmark_strike::Input in;
    in.hallmark_id  = "UK-ASSAY-750";
    in.alloy_purity = 750;
    in.force_kn     = 2.4;

    auto out = skill::hallmark_strike::apply(*stock, in);

    EXPECT_EQ(out.signature.skill_id, std::string("hallmark_strike"));
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("hallmark_strike"));
    EXPECT_EQ(out.signature.pattern["hallmark_id"].get<std::string>(),
              std::string("UK-ASSAY-750"));
    EXPECT_EQ(out.signature.pattern["alloy_purity"].get<int>(), 750);
    EXPECT_NEAR(out.signature.pattern["force_kn"].get<double>(), 2.4, 1e-9);
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
}

// ─── 4. Recognize metadata replay ─────────────────────────────────────────
TEST(SkillHallmarkStrike, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(20.0, 5.0, 2.0);

    skill::hallmark_strike::Input in;
    in.hallmark_id  = "ZH-CN-999";
    in.alloy_purity = 999;
    in.force_kn     = 0.8;

    auto out   = skill::hallmark_strike::apply(*stock, in);
    auto cands = skill::hallmark_strike::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["hallmark_id"].get<std::string>(),
              std::string("ZH-CN-999"));
    EXPECT_EQ(cands[0].recovered_params["alloy_purity"].get<int>(), 999);

    skill::Workpiece raw(out.workpiece->shape());
    auto cands2 = skill::hallmark_strike::recognize(raw);
    EXPECT_TRUE(cands2.empty());
}

// ─── 5. Specific: 925 purity → "silver" purity family classification ─────
TEST(SkillHallmarkStrike, SilverFamilyTaggedFor925Purity)
{
    auto stock = skill::createCuboidStock(20.0, 5.0, 2.0);

    skill::hallmark_strike::Input ag;
    ag.hallmark_id  = "KS-LON-925";
    ag.alloy_purity = 925;
    ag.force_kn     = 1.5;
    auto outAg = skill::hallmark_strike::apply(*stock, ag);
    EXPECT_EQ(outAg.signature.pattern["purity_family"].get<std::string>(),
              std::string("silver"));

    skill::hallmark_strike::Input pt;
    pt.hallmark_id  = "KS-LON-950";
    pt.alloy_purity = 950;
    pt.force_kn     = 1.5;
    auto outPt = skill::hallmark_strike::apply(*stock, pt);
    EXPECT_EQ(outPt.signature.pattern["purity_family"].get<std::string>(),
              std::string("platinum_or_palladium"));

    EXPECT_EQ(outAg.signature.tooling.tool_type,
              std::string("hallmark_punch"));
    EXPECT_EQ(outAg.signature.tooling.tool_material,
              std::string("hardened_tool_steel"));
}
