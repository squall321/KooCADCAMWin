// @lat: [[process/test-strategy#skill round-trip]]
//
// titrate — volumetric titration record.  Metadata-only.
// Cases (5):
//   1. ApplyClonesGeometryUnchanged
//   2. ValidateRejectsBadInput
//   3. SignatureRecordsKindAndKeyParams
//   4. RecognizeMetadataReplay
//   5. ExtremePhSelectsConductivityProbe

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/titrate.hpp"

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

skill::titrate::Input goodInput()
{
    skill::titrate::Input in;
    in.analyte            = "HCl_0_1M";
    in.titrant            = "NaOH_0_1M";
    in.target_endpoint_ph = 7.0;
    in.volume_ml          = 25.0;
    return in;
}

}  // namespace

// ─── 1. Apply clones geometry unchanged ───────────────────────────────────
TEST(SkillTitrate, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 100.0);
    const double volBefore = volumeOf(stock->shape());
    const int faceBefore   = stock->faceCount();
    const int edgeBefore   = stock->edgeCount();

    auto out = skill::titrate::apply(*stock, goodInput());
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("titrate"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. Validate rejects bad input ────────────────────────────────────────
TEST(SkillTitrate, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 100.0);

    // Empty analyte.
    auto empty = goodInput();
    empty.analyte = "";
    {
        auto r = skill::titrate::validate(*stock, empty);
        EXPECT_FALSE(r.passed);
        EXPECT_TRUE(hasFinding(r, "DFM-INPUT"));
    }
    EXPECT_THROW(skill::titrate::apply(*stock, empty), skill::SkillError);

    // Out-of-range pH.
    auto badPh = goodInput();
    badPh.target_endpoint_ph = 15.0;
    {
        auto r = skill::titrate::validate(*stock, badPh);
        EXPECT_FALSE(r.passed);
        EXPECT_TRUE(hasFinding(r, "DFM-TITR-PH"));
    }
    EXPECT_THROW(skill::titrate::apply(*stock, badPh), skill::SkillError);

    // Non-positive volume.
    auto zeroVol = goodInput();
    zeroVol.volume_ml = 0.0;
    {
        auto r = skill::titrate::validate(*stock, zeroVol);
        EXPECT_FALSE(r.passed);
        EXPECT_TRUE(hasFinding(r, "DFM-TITR-VOL"));
    }
}

// ─── 3. Signature records kind + key params ───────────────────────────────
TEST(SkillTitrate, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 100.0);

    auto in = goodInput();
    in.analyte            = "acetic_acid";
    in.titrant            = "NaOH_0_05M";
    in.target_endpoint_ph = 8.4;
    in.volume_ml          = 18.5;

    auto out = skill::titrate::apply(*stock, in);

    EXPECT_EQ(out.signature.skill_id, std::string("titrate"));
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("titrate"));
    EXPECT_EQ(out.signature.pattern["analyte"].get<std::string>(),
              std::string("acetic_acid"));
    EXPECT_EQ(out.signature.pattern["titrant"].get<std::string>(),
              std::string("NaOH_0_05M"));
    EXPECT_NEAR(out.signature.pattern["target_endpoint_ph"].get<double>(),
                8.4, 1e-9);
    EXPECT_NEAR(out.signature.pattern["volume_ml"].get<double>(),
                18.5, 1e-9);
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
}

// ─── 4. Recognize via metadata replay ─────────────────────────────────────
TEST(SkillTitrate, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 100.0);

    auto out   = skill::titrate::apply(*stock, goodInput());
    auto cands = skill::titrate::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["analyte"].get<std::string>(),
              std::string("HCl_0_1M"));
    EXPECT_NEAR(cands[0].recovered_params["volume_ml"].get<double>(),
                25.0, 1e-9);

    skill::Workpiece raw(out.workpiece->shape());
    auto cands2 = skill::titrate::recognize(raw);
    EXPECT_TRUE(cands2.empty());
}

// ─── 5. Specific: extreme pH endpoint selects conductivity probe ──────────
TEST(SkillTitrate, ExtremePhSelectsConductivityProbe)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 100.0);

    auto neutral = goodInput();
    neutral.target_endpoint_ph = 7.0;
    auto neutralOut = skill::titrate::apply(*stock, neutral);
    EXPECT_EQ(neutralOut.signature.tooling.tool_type,
              std::string("ph_meter"));

    auto strongAcid = goodInput();
    strongAcid.target_endpoint_ph = 1.5;
    auto strongAcidOut = skill::titrate::apply(*stock, strongAcid);
    EXPECT_EQ(strongAcidOut.signature.tooling.tool_type,
              std::string("conductivity_probe"));

    auto strongBase = goodInput();
    strongBase.target_endpoint_ph = 12.5;
    auto strongBaseOut = skill::titrate::apply(*stock, strongBase);
    EXPECT_EQ(strongBaseOut.signature.tooling.tool_type,
              std::string("conductivity_probe"));
}
