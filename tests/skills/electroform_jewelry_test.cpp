// @lat: [[process/test-strategy#skill round-trip]]
//
// electroform_jewelry — electrolytic shell deposition on a mandrel.
// Metadata-only.  Cases (5):
//   1. ApplyClonesGeometryUnchanged
//   2. ValidateRejectsBadInput
//   3. SignatureRecordsKindAndParams
//   4. RecognizeMetadataReplay
//   5. GoldBathSelectedForGoldMetal

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/electroform_jewelry.hpp"

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
TEST(SkillElectroformJewelry, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(15.0, 15.0, 8.0);
    const double volBefore = volumeOf(stock->shape());
    const int faceBefore   = stock->faceCount();
    const int edgeBefore   = stock->edgeCount();

    skill::electroform_jewelry::Input in;
    in.deposit_metal = "gold";
    in.thickness_um  = 150.0;
    in.mandrel_id    = "MND-1001";

    auto out = skill::electroform_jewelry::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("electroform_jewelry"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. Validate rejects bad input ────────────────────────────────────────
TEST(SkillElectroformJewelry, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(15.0, 15.0, 8.0);

    skill::electroform_jewelry::Input badMetal;
    badMetal.deposit_metal = "tin";   // not in known set
    badMetal.thickness_um  = 150.0;
    badMetal.mandrel_id    = "MND-1";
    {
        auto r = skill::electroform_jewelry::validate(*stock, badMetal);
        EXPECT_FALSE(r.passed);
        EXPECT_TRUE(hasFinding(r, "DFM-DEPOSIT-METAL"));
    }
    EXPECT_THROW(skill::electroform_jewelry::apply(*stock, badMetal),
                 skill::SkillError);

    skill::electroform_jewelry::Input zeroT;
    zeroT.deposit_metal = "gold";
    zeroT.thickness_um  = 0.0;
    zeroT.mandrel_id    = "MND-1";
    {
        auto r = skill::electroform_jewelry::validate(*stock, zeroT);
        EXPECT_FALSE(r.passed);
        EXPECT_TRUE(hasFinding(r, "DFM-THICKNESS"));
    }
    EXPECT_THROW(skill::electroform_jewelry::apply(*stock, zeroT),
                 skill::SkillError);

    skill::electroform_jewelry::Input noMandrel;
    noMandrel.deposit_metal = "silver";
    noMandrel.thickness_um  = 150.0;
    noMandrel.mandrel_id    = "";
    {
        auto r = skill::electroform_jewelry::validate(*stock, noMandrel);
        EXPECT_FALSE(r.passed);
        EXPECT_TRUE(hasFinding(r, "DFM-INPUT"));
    }
}

// ─── 3. Signature records kind + key params ───────────────────────────────
TEST(SkillElectroformJewelry, SignatureRecordsKindAndParams)
{
    auto stock = skill::createCuboidStock(15.0, 15.0, 8.0);

    skill::electroform_jewelry::Input in;
    in.deposit_metal = "copper";
    in.thickness_um  = 300.0;
    in.mandrel_id    = "MND-7777";

    auto out = skill::electroform_jewelry::apply(*stock, in);

    EXPECT_EQ(out.signature.skill_id, std::string("electroform_jewelry"));
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("electroform_jewelry"));
    EXPECT_EQ(out.signature.pattern["deposit_metal"].get<std::string>(),
              std::string("copper"));
    EXPECT_NEAR(out.signature.pattern["thickness_um"].get<double>(),
                300.0, 1e-9);
    EXPECT_EQ(out.signature.pattern["mandrel_id"].get<std::string>(),
              std::string("MND-7777"));
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
}

// ─── 4. Recognize metadata replay ─────────────────────────────────────────
TEST(SkillElectroformJewelry, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(15.0, 15.0, 8.0);

    skill::electroform_jewelry::Input in;
    in.deposit_metal = "silver";
    in.thickness_um  = 100.0;
    in.mandrel_id    = "MND-A2";

    auto out   = skill::electroform_jewelry::apply(*stock, in);
    auto cands = skill::electroform_jewelry::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["deposit_metal"].get<std::string>(),
              std::string("silver"));
    EXPECT_EQ(cands[0].recovered_params["mandrel_id"].get<std::string>(),
              std::string("MND-A2"));

    skill::Workpiece raw(out.workpiece->shape());
    auto cands2 = skill::electroform_jewelry::recognize(raw);
    EXPECT_TRUE(cands2.empty());
}

// ─── 5. Specific: gold metal selects Au cyanide bath material ─────────────
TEST(SkillElectroformJewelry, GoldBathSelectedForGoldMetal)
{
    auto stock = skill::createCuboidStock(15.0, 15.0, 8.0);

    skill::electroform_jewelry::Input in;
    in.deposit_metal = "gold";
    in.thickness_um  = 80.0;
    in.mandrel_id    = "MND-Au-01";

    auto out = skill::electroform_jewelry::apply(*stock, in);
    EXPECT_EQ(out.signature.tooling.tool_type,
              std::string("electroforming_bath"));
    EXPECT_EQ(out.signature.tooling.tool_material,
              std::string("Au_cyanide_bath"));
    EXPECT_NEAR(out.signature.pattern["deposit_rate_um_h"].get<double>(),
                18.0, 1e-6);
}
