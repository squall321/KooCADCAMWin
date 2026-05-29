// @lat: [[process/test-strategy#skill round-trip]]
//
// ore_flotation — froth flotation bank.  Metadata-only.
// Cases (5):
//   1. ApplyClonesGeometryUnchanged
//   2. ValidateRejectsBadInput
//   3. SignatureRecordsKindAndParams
//   4. RecognizeMetadataReplay
//   5. TotalBankTimeAndCycleTimeMatch

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/ore_flotation.hpp"

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
TEST(SkillOreFlotation, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(200.0, 200.0, 50.0);
    const double volBefore = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    skill::ore_flotation::Input in;
    in.cell_count         = 8;
    in.retention_time_min = 12.0;
    in.recovery_pct       = 85.0;

    auto out = skill::ore_flotation::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("ore_flotation"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. Validate rejects bad input ────────────────────────────────────────
TEST(SkillOreFlotation, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(100.0, 100.0, 50.0);

    skill::ore_flotation::Input badCells;
    badCells.cell_count         = 0;
    badCells.retention_time_min = 12.0;
    badCells.recovery_pct       = 80.0;
    {
        auto r = skill::ore_flotation::validate(*stock, badCells);
        EXPECT_FALSE(r.passed);
        EXPECT_TRUE(hasFinding(r, "DFM-INPUT"));
    }
    EXPECT_THROW(skill::ore_flotation::apply(*stock, badCells), skill::SkillError);

    skill::ore_flotation::Input badRecovery;
    badRecovery.cell_count         = 8;
    badRecovery.retention_time_min = 12.0;
    badRecovery.recovery_pct       = 120.0;    // > 100
    {
        auto r = skill::ore_flotation::validate(*stock, badRecovery);
        EXPECT_FALSE(r.passed);
        EXPECT_TRUE(hasFinding(r, "DFM-INPUT"));
    }
    EXPECT_THROW(skill::ore_flotation::apply(*stock, badRecovery),
                 skill::SkillError);
}

// ─── 3. Signature records kind + key params ───────────────────────────────
TEST(SkillOreFlotation, SignatureRecordsKindAndParams)
{
    auto stock = skill::createCuboidStock(100.0, 100.0, 50.0);

    skill::ore_flotation::Input in;
    in.cell_count         = 10;
    in.retention_time_min = 20.0;
    in.recovery_pct       = 92.5;

    auto out = skill::ore_flotation::apply(*stock, in);

    EXPECT_EQ(out.signature.skill_id, std::string("ore_flotation"));
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("ore_flotation"));
    EXPECT_EQ(out.signature.pattern["cell_count"].get<int>(), 10);
    EXPECT_NEAR(out.signature.pattern["retention_time_min"].get<double>(),
                20.0, 1e-9);
    EXPECT_NEAR(out.signature.pattern["recovery_pct"].get<double>(),
                92.5, 1e-9);
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
}

// ─── 4. Recognize metadata replay ─────────────────────────────────────────
TEST(SkillOreFlotation, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(100.0, 100.0, 50.0);

    skill::ore_flotation::Input in;
    in.cell_count         = 6;
    in.retention_time_min = 10.0;
    in.recovery_pct       = 78.0;

    auto out   = skill::ore_flotation::apply(*stock, in);
    auto cands = skill::ore_flotation::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["cell_count"].get<int>(), 6);
    EXPECT_NEAR(cands[0].recovered_params["recovery_pct"].get<double>(),
                78.0, 1e-9);

    skill::Workpiece raw(out.workpiece->shape());
    auto cands2 = skill::ore_flotation::recognize(raw);
    EXPECT_TRUE(cands2.empty());
}

// ─── 5. Specific: bank time + cycle time consistency ─────────────────────
TEST(SkillOreFlotation, TotalBankTimeAndCycleTimeMatch)
{
    auto stock = skill::createCuboidStock(100.0, 100.0, 50.0);

    skill::ore_flotation::Input in;
    in.cell_count         = 8;
    in.retention_time_min = 15.0;
    in.recovery_pct       = 88.0;

    auto out = skill::ore_flotation::apply(*stock, in);
    EXPECT_NEAR(out.signature.pattern["total_bank_time_min"].get<double>(),
                120.0, 1e-9);
    EXPECT_NEAR(out.signature.tooling.est_cycle_time_s, 120.0 * 60.0, 1e-6);
    EXPECT_EQ(out.signature.tooling.tool_type,
              std::string("flotation_cell_bank"));
}
