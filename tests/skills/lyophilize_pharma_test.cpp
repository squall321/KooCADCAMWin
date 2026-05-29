// @lat: [[process/test-strategy#skill round-trip]]
//
// lyophilize_pharma — pharma freeze-dry metadata stamp.  Metadata-only.
// Cases (5):
//   1. ApplyClonesGeometryUnchanged
//   2. ValidateRejectsBadInput
//   3. SignatureRecordsKindAndParams
//   4. RecognizeMetadataReplay
//   5. HighSecondaryTempEmitsDenatInfo

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/lyophilize_pharma.hpp"

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
TEST(SkillLyophilizePharma, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(20.0, 20.0, 5.0);
    const double volBefore = volumeOf(stock->shape());
    const int faceBefore   = stock->faceCount();
    const int edgeBefore   = stock->edgeCount();

    skill::lyophilize_pharma::Input in;
    in.primary_dry_temp_c   = -30.0;
    in.secondary_dry_temp_c = 25.0;
    in.vacuum_mbar          = 0.1;

    auto out = skill::lyophilize_pharma::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. Validate rejects bad input ─────────────────────────────────────────
TEST(SkillLyophilizePharma, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(20.0, 20.0, 3.0);

    // Vacuum out of range.
    skill::lyophilize_pharma::Input zeroV;
    zeroV.primary_dry_temp_c   = -30.0;
    zeroV.secondary_dry_temp_c = 25.0;
    zeroV.vacuum_mbar          = 0.0;
    {
        auto r = skill::lyophilize_pharma::validate(*stock, zeroV);
        EXPECT_FALSE(r.passed);
        EXPECT_TRUE(hasFinding(r, "DFM-INPUT"));
    }
    EXPECT_THROW(skill::lyophilize_pharma::apply(*stock, zeroV),
                 skill::SkillError);

    // Secondary not greater than primary.
    skill::lyophilize_pharma::Input badOrder;
    badOrder.primary_dry_temp_c   = 10.0;
    badOrder.secondary_dry_temp_c = 5.0;       // ≤ primary → error
    badOrder.vacuum_mbar          = 0.1;
    {
        auto r = skill::lyophilize_pharma::validate(*stock, badOrder);
        EXPECT_FALSE(r.passed);
        EXPECT_TRUE(hasFinding(r, "DFM-INPUT"));
    }

    // Vacuum too high.
    skill::lyophilize_pharma::Input highV;
    highV.primary_dry_temp_c   = -30.0;
    highV.secondary_dry_temp_c = 25.0;
    highV.vacuum_mbar          = 250.0;     // > 100 → error
    {
        auto r = skill::lyophilize_pharma::validate(*stock, highV);
        EXPECT_FALSE(r.passed);
        EXPECT_TRUE(hasFinding(r, "DFM-INPUT"));
    }
}

// ─── 3. Signature records kind + key params ────────────────────────────────
TEST(SkillLyophilizePharma, SignatureRecordsKindAndParams)
{
    auto stock = skill::createCuboidStock(15.0, 15.0, 3.0);

    skill::lyophilize_pharma::Input in;
    in.primary_dry_temp_c   = -25.0;
    in.secondary_dry_temp_c = 30.0;
    in.vacuum_mbar          = 0.05;

    auto out = skill::lyophilize_pharma::apply(*stock, in);

    EXPECT_EQ(out.signature.skill_id, std::string("lyophilize_pharma"));
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("lyophilize_pharma"));
    EXPECT_NEAR(out.signature.pattern["primary_dry_temp_c"].get<double>(),
                -25.0, 1e-9);
    EXPECT_NEAR(out.signature.pattern["secondary_dry_temp_c"].get<double>(),
                30.0, 1e-9);
    EXPECT_NEAR(out.signature.pattern["vacuum_mbar"].get<double>(),
                0.05, 1e-9);
    EXPECT_FALSE(out.signature.pattern["geometric_change"].get<bool>());
}

// ─── 4. Recognize metadata replay ──────────────────────────────────────────
TEST(SkillLyophilizePharma, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(15.0, 15.0, 3.0);

    skill::lyophilize_pharma::Input in;
    in.primary_dry_temp_c   = -40.0;
    in.secondary_dry_temp_c = 35.0;
    in.vacuum_mbar          = 0.08;

    auto out   = skill::lyophilize_pharma::apply(*stock, in);
    auto cands = skill::lyophilize_pharma::recognize(*out.workpiece);

    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_NEAR(
        cands[0].recovered_params["primary_dry_temp_c"].get<double>(),
        -40.0, 1e-9);
    EXPECT_NEAR(
        cands[0].recovered_params["secondary_dry_temp_c"].get<double>(),
        35.0, 1e-9);

    // Stripped of metadata → recognize empty.
    skill::Workpiece raw(out.workpiece->shape());
    auto cands2 = skill::lyophilize_pharma::recognize(raw);
    EXPECT_TRUE(cands2.empty());
}

// ─── 5. High secondary temp emits denat info, still proceeds ───────────────
TEST(SkillLyophilizePharma, HighSecondaryTempEmitsDenatInfo)
{
    auto stock = skill::createCuboidStock(10.0, 10.0, 2.0);

    skill::lyophilize_pharma::Input in;
    in.primary_dry_temp_c   = -20.0;
    in.secondary_dry_temp_c = 70.0;      // > 60 → info
    in.vacuum_mbar          = 0.1;

    auto r = skill::lyophilize_pharma::validate(*stock, in);
    EXPECT_TRUE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-LYP-SEC-TEMP"));

    auto out = skill::lyophilize_pharma::apply(*stock, in);
    EXPECT_EQ(out.signature.tooling.tool_type,
              std::string("pharma_freeze_dryer"));
}
