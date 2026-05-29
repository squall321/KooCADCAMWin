// @lat: [[process/test-strategy#skill round-trip]]
//
// blister_pack — thermoformed blister + lidding.  Metadata-only.
// Cases (5):
//   1. ApplyClonesGeometryUnchanged
//   2. ValidateRejectsBadInput
//   3. SignatureRecordsKindAndParams
//   4. RecognizeMetadataReplay
//   5. HighCavityCountEmitsInfo

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/blister_pack.hpp"

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
TEST(SkillBlisterPack, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(60.0, 90.0, 4.0);
    const double volBefore = volumeOf(stock->shape());
    const int faceBefore   = stock->faceCount();
    const int edgeBefore   = stock->edgeCount();

    skill::blister_pack::Input in;
    in.cavity_count     = 10;
    in.lidding_material = "aluminum_foil";
    in.sealing_temp_c   = 170.0;

    auto out = skill::blister_pack::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("blister_pack"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. Validate rejects bad input ────────────────────────────────────────
TEST(SkillBlisterPack, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 3.0);

    // cavity_count <= 0.
    skill::blister_pack::Input badCount;
    badCount.cavity_count     = 0;
    badCount.lidding_material = "aluminum_foil";
    badCount.sealing_temp_c   = 170.0;
    {
        auto r = skill::blister_pack::validate(*stock, badCount);
        EXPECT_FALSE(r.passed);
        EXPECT_TRUE(hasFinding(r, "DFM-INPUT"));
    }
    EXPECT_THROW(skill::blister_pack::apply(*stock, badCount),
                 skill::SkillError);

    // Empty lidding.
    skill::blister_pack::Input noLid;
    noLid.cavity_count     = 10;
    noLid.lidding_material = "";
    noLid.sealing_temp_c   = 170.0;
    {
        auto r = skill::blister_pack::validate(*stock, noLid);
        EXPECT_FALSE(r.passed);
        EXPECT_TRUE(hasFinding(r, "DFM-BLISTER-LID"));
    }
    EXPECT_THROW(skill::blister_pack::apply(*stock, noLid),
                 skill::SkillError);

    // sealing_temp_c <= 0.
    skill::blister_pack::Input badTemp;
    badTemp.cavity_count     = 10;
    badTemp.lidding_material = "aluminum_foil";
    badTemp.sealing_temp_c   = 0.0;
    {
        auto r = skill::blister_pack::validate(*stock, badTemp);
        EXPECT_FALSE(r.passed);
        EXPECT_TRUE(hasFinding(r, "DFM-INPUT"));
    }
    EXPECT_THROW(skill::blister_pack::apply(*stock, badTemp),
                 skill::SkillError);
}

// ─── 3. Signature records kind + key params ───────────────────────────────
TEST(SkillBlisterPack, SignatureRecordsKindAndParams)
{
    auto stock = skill::createCuboidStock(40.0, 60.0, 4.0);

    skill::blister_pack::Input in;
    in.cavity_count     = 14;
    in.lidding_material = "paper_foil";
    in.sealing_temp_c   = 180.0;

    auto out = skill::blister_pack::apply(*stock, in);

    EXPECT_EQ(out.signature.skill_id, std::string("blister_pack"));
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("blister_pack"));
    EXPECT_EQ(out.signature.pattern["cavity_count"].get<int>(), 14);
    EXPECT_EQ(out.signature.pattern["lidding_material"].get<std::string>(),
              std::string("paper_foil"));
    EXPECT_NEAR(out.signature.pattern["sealing_temp_c"].get<double>(),
                180.0, 1e-9);
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
}

// ─── 4. Recognize metadata replay ─────────────────────────────────────────
TEST(SkillBlisterPack, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 3.0);

    skill::blister_pack::Input in;
    in.cavity_count     = 7;
    in.lidding_material = "polymer_film";
    in.sealing_temp_c   = 160.0;

    auto out   = skill::blister_pack::apply(*stock, in);
    auto cands = skill::blister_pack::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["lidding_material"].get<std::string>(),
              std::string("polymer_film"));
    EXPECT_EQ(cands[0].recovered_params["cavity_count"].get<int>(), 7);

    skill::Workpiece raw(out.workpiece->shape());
    auto cands2 = skill::blister_pack::recognize(raw);
    EXPECT_TRUE(cands2.empty());
}

// ─── 5. Specific: cavity_count > 60 emits info, does NOT block ────────────
TEST(SkillBlisterPack, HighCavityCountEmitsInfoButProceeds)
{
    auto stock = skill::createCuboidStock(200.0, 200.0, 4.0);

    skill::blister_pack::Input big;
    big.cavity_count     = 100;
    big.lidding_material = "aluminum_foil";
    big.sealing_temp_c   = 170.0;

    auto r = skill::blister_pack::validate(*stock, big);
    EXPECT_TRUE(r.passed) << "high cavity count should NOT block";
    EXPECT_TRUE(hasFinding(r, "DFM-BLISTER-COUNT"));

    EXPECT_NO_THROW(skill::blister_pack::apply(*stock, big));
}
