// @lat: [[process/test-strategy#skill round-trip]]
//
// lost_wax_jewelry — investment casting for jewelry.  Metadata-only.
// Cases (5):
//   1. ApplyClonesGeometryUnchanged
//   2. ValidateRejectsBadInput
//   3. SignatureRecordsKindAndParams
//   4. RecognizeMetadataReplay
//   5. ExpectedMetalScalesWithAlloyDensity

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/lost_wax_jewelry.hpp"

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
TEST(SkillLostWaxJewelry, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(20.0, 20.0, 10.0);
    const double volBefore = volumeOf(stock->shape());
    const int faceBefore   = stock->faceCount();
    const int edgeBefore   = stock->edgeCount();

    skill::lost_wax_jewelry::Input in;
    in.alloy        = "Au18K";
    in.wax_weight_g = 4.0;
    in.sprue_count  = 4;

    auto out = skill::lost_wax_jewelry::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("lost_wax_jewelry"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. Validate rejects bad input ────────────────────────────────────────
TEST(SkillLostWaxJewelry, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(20.0, 20.0, 10.0);

    skill::lost_wax_jewelry::Input emptyAlloy;
    emptyAlloy.alloy        = "";
    emptyAlloy.wax_weight_g = 4.0;
    emptyAlloy.sprue_count  = 4;
    {
        auto r = skill::lost_wax_jewelry::validate(*stock, emptyAlloy);
        EXPECT_FALSE(r.passed);
        EXPECT_TRUE(hasFinding(r, "DFM-INPUT"));
    }
    EXPECT_THROW(skill::lost_wax_jewelry::apply(*stock, emptyAlloy),
                 skill::SkillError);

    skill::lost_wax_jewelry::Input zeroWax;
    zeroWax.alloy        = "Ag925";
    zeroWax.wax_weight_g = 0.0;
    zeroWax.sprue_count  = 4;
    {
        auto r = skill::lost_wax_jewelry::validate(*stock, zeroWax);
        EXPECT_FALSE(r.passed);
        EXPECT_TRUE(hasFinding(r, "DFM-WAX-WEIGHT"));
    }
    EXPECT_THROW(skill::lost_wax_jewelry::apply(*stock, zeroWax),
                 skill::SkillError);

    skill::lost_wax_jewelry::Input zeroSprue;
    zeroSprue.alloy        = "Ag925";
    zeroSprue.wax_weight_g = 4.0;
    zeroSprue.sprue_count  = 0;
    {
        auto r = skill::lost_wax_jewelry::validate(*stock, zeroSprue);
        EXPECT_FALSE(r.passed);
        EXPECT_TRUE(hasFinding(r, "DFM-SPRUE-COUNT"));
    }
}

// ─── 3. Signature records kind + key params ───────────────────────────────
TEST(SkillLostWaxJewelry, SignatureRecordsKindAndParams)
{
    auto stock = skill::createCuboidStock(20.0, 20.0, 10.0);

    skill::lost_wax_jewelry::Input in;
    in.alloy        = "Pt950";
    in.wax_weight_g = 8.5;
    in.sprue_count  = 6;

    auto out = skill::lost_wax_jewelry::apply(*stock, in);

    EXPECT_EQ(out.signature.skill_id, std::string("lost_wax_jewelry"));
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("lost_wax_jewelry"));
    EXPECT_EQ(out.signature.pattern["alloy"].get<std::string>(),
              std::string("Pt950"));
    EXPECT_NEAR(out.signature.pattern["wax_weight_g"].get<double>(),
                8.5, 1e-9);
    EXPECT_EQ(out.signature.pattern["sprue_count"].get<int>(), 6);
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
}

// ─── 4. Recognize metadata replay ─────────────────────────────────────────
TEST(SkillLostWaxJewelry, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(20.0, 20.0, 10.0);

    skill::lost_wax_jewelry::Input in;
    in.alloy        = "Ag925";
    in.wax_weight_g = 3.2;
    in.sprue_count  = 8;

    auto out   = skill::lost_wax_jewelry::apply(*stock, in);
    auto cands = skill::lost_wax_jewelry::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["alloy"].get<std::string>(),
              std::string("Ag925"));
    EXPECT_EQ(cands[0].recovered_params["sprue_count"].get<int>(), 8);

    skill::Workpiece raw(out.workpiece->shape());
    auto cands2 = skill::lost_wax_jewelry::recognize(raw);
    EXPECT_TRUE(cands2.empty());
}

// ─── 5. Specific: heavier alloy yields more metal for same wax ────────────
TEST(SkillLostWaxJewelry, ExpectedMetalScalesWithAlloyDensity)
{
    auto stock = skill::createCuboidStock(20.0, 20.0, 10.0);

    skill::lost_wax_jewelry::Input ag;
    ag.alloy        = "Ag925";   // density 10.36
    ag.wax_weight_g = 5.0;
    ag.sprue_count  = 4;
    auto outAg = skill::lost_wax_jewelry::apply(*stock, ag);

    skill::lost_wax_jewelry::Input pt;
    pt.alloy        = "Pt950";   // density 20.10
    pt.wax_weight_g = 5.0;
    pt.sprue_count  = 4;
    auto outPt = skill::lost_wax_jewelry::apply(*stock, pt);

    const double mAg = outAg.signature.pattern["expected_metal_g"].get<double>();
    const double mPt = outPt.signature.pattern["expected_metal_g"].get<double>();
    EXPECT_GT(mPt, mAg);                       // Pt > Ag for same wax
    EXPECT_NEAR(mAg, 5.0 * (10.36 / 0.95), 1e-6);
    EXPECT_NEAR(mPt, 5.0 * (20.10 / 0.95), 1e-6);
}
