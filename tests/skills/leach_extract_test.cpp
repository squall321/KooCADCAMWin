// @lat: [[process/test-strategy#skill round-trip]]
//
// leach_extract — hydrometallurgical leach.  Metadata-only.
// Cases (5):
//   1. ApplyClonesGeometryUnchanged
//   2. ValidateRejectsBadInput
//   3. SignatureRecordsKindAndParams
//   4. RecognizeMetadataReplay
//   5. AcidLeachSelectsRubberLinedSteelTank

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/leach_extract.hpp"

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
TEST(SkillLeachExtract, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(150.0, 150.0, 50.0);
    const double volBefore = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    skill::leach_extract::Input in;
    in.ph           = 2.0;
    in.leach_time_h = 24.0;
    in.recovery_pct = 80.0;

    auto out = skill::leach_extract::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("leach_extract"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. Validate rejects bad input ────────────────────────────────────────
TEST(SkillLeachExtract, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(100.0, 100.0, 50.0);

    skill::leach_extract::Input badPh;
    badPh.ph           = 15.0;        // out of [0,14]
    badPh.leach_time_h = 12.0;
    badPh.recovery_pct = 80.0;
    {
        auto r = skill::leach_extract::validate(*stock, badPh);
        EXPECT_FALSE(r.passed);
        EXPECT_TRUE(hasFinding(r, "DFM-INPUT"));
    }
    EXPECT_THROW(skill::leach_extract::apply(*stock, badPh), skill::SkillError);

    skill::leach_extract::Input zeroTime;
    zeroTime.ph           = 2.0;
    zeroTime.leach_time_h = 0.0;
    zeroTime.recovery_pct = 80.0;
    {
        auto r = skill::leach_extract::validate(*stock, zeroTime);
        EXPECT_FALSE(r.passed);
        EXPECT_TRUE(hasFinding(r, "DFM-INPUT"));
    }
    EXPECT_THROW(skill::leach_extract::apply(*stock, zeroTime), skill::SkillError);

    skill::leach_extract::Input badRecovery;
    badRecovery.ph           = 2.0;
    badRecovery.leach_time_h = 12.0;
    badRecovery.recovery_pct = 0.0;       // not in (0, 100]
    {
        auto r = skill::leach_extract::validate(*stock, badRecovery);
        EXPECT_FALSE(r.passed);
        EXPECT_TRUE(hasFinding(r, "DFM-INPUT"));
    }
}

// ─── 3. Signature records kind + key params ───────────────────────────────
TEST(SkillLeachExtract, SignatureRecordsKindAndParams)
{
    auto stock = skill::createCuboidStock(100.0, 100.0, 50.0);

    skill::leach_extract::Input in;
    in.ph           = 10.5;        // alkaline (cyanide-style)
    in.leach_time_h = 48.0;
    in.recovery_pct = 90.0;

    auto out = skill::leach_extract::apply(*stock, in);

    EXPECT_EQ(out.signature.skill_id, std::string("leach_extract"));
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("leach_extract"));
    EXPECT_NEAR(out.signature.pattern["ph"].get<double>(),
                10.5, 1e-9);
    EXPECT_NEAR(out.signature.pattern["leach_time_h"].get<double>(),
                48.0, 1e-9);
    EXPECT_NEAR(out.signature.pattern["recovery_pct"].get<double>(),
                90.0, 1e-9);
    EXPECT_EQ(out.signature.pattern["lixiviant_class"].get<std::string>(),
              std::string("alkaline"));
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
}

// ─── 4. Recognize metadata replay ─────────────────────────────────────────
TEST(SkillLeachExtract, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(100.0, 100.0, 50.0);

    skill::leach_extract::Input in;
    in.ph           = 6.0;        // neutral
    in.leach_time_h = 18.0;
    in.recovery_pct = 75.0;

    auto out   = skill::leach_extract::apply(*stock, in);
    auto cands = skill::leach_extract::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_NEAR(cands[0].recovered_params["ph"].get<double>(), 6.0, 1e-9);
    EXPECT_NEAR(cands[0].recovered_params["leach_time_h"].get<double>(),
                18.0, 1e-9);

    skill::Workpiece raw(out.workpiece->shape());
    auto cands2 = skill::leach_extract::recognize(raw);
    EXPECT_TRUE(cands2.empty());
}

// ─── 5. Specific: acid leach picks rubber-lined steel tank ───────────────
TEST(SkillLeachExtract, AcidLeachSelectsRubberLinedSteelTank)
{
    auto stock = skill::createCuboidStock(100.0, 100.0, 50.0);

    skill::leach_extract::Input in;
    in.ph           = 1.5;        // acid class
    in.leach_time_h = 24.0;
    in.recovery_pct = 85.0;

    auto out = skill::leach_extract::apply(*stock, in);
    EXPECT_EQ(out.signature.pattern["lixiviant_class"].get<std::string>(),
              std::string("acid"));
    EXPECT_EQ(out.signature.tooling.tool_type,
              std::string("leach_tank"));
    EXPECT_EQ(out.signature.tooling.tool_material,
              std::string("rubber_lined_steel"));
}
