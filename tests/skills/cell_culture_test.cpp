// @lat: [[process/test-strategy#skill round-trip]]
//
// cell_culture — in-vitro culture run record.  Metadata-only.
// Cases (5):
//   1. ApplyClonesGeometryUnchanged
//   2. ValidateRejectsBadInput
//   3. SignatureRecordsKindAndKeyParams
//   4. RecognizeMetadataReplay
//   5. LongRunSelectsBioreactor

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/cell_culture.hpp"

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

skill::cell_culture::Input goodInput()
{
    skill::cell_culture::Input in;
    in.cell_line             = "HEK293";
    in.media_type            = "DMEM";
    in.days_in_culture       = 3.0;
    in.target_confluence_pct = 80.0;
    return in;
}

}  // namespace

// ─── 1. Apply clones geometry unchanged ───────────────────────────────────
TEST(SkillCellCulture, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(80.0, 120.0, 25.0);
    const double volBefore = volumeOf(stock->shape());
    const int faceBefore   = stock->faceCount();
    const int edgeBefore   = stock->edgeCount();

    auto out = skill::cell_culture::apply(*stock, goodInput());
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("cell_culture"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. Validate rejects bad input ────────────────────────────────────────
TEST(SkillCellCulture, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(80.0, 120.0, 25.0);

    // Empty cell line.
    auto empty = goodInput();
    empty.cell_line = "";
    {
        auto r = skill::cell_culture::validate(*stock, empty);
        EXPECT_FALSE(r.passed);
        EXPECT_TRUE(hasFinding(r, "DFM-INPUT"));
    }
    EXPECT_THROW(skill::cell_culture::apply(*stock, empty), skill::SkillError);

    // Non-positive days.
    auto zeroDays = goodInput();
    zeroDays.days_in_culture = 0.0;
    {
        auto r = skill::cell_culture::validate(*stock, zeroDays);
        EXPECT_FALSE(r.passed);
        EXPECT_TRUE(hasFinding(r, "DFM-CULT-TIME"));
    }
    EXPECT_THROW(skill::cell_culture::apply(*stock, zeroDays),
                 skill::SkillError);

    // Confluence > 100.
    auto over = goodInput();
    over.target_confluence_pct = 120.0;
    {
        auto r = skill::cell_culture::validate(*stock, over);
        EXPECT_FALSE(r.passed);
        EXPECT_TRUE(hasFinding(r, "DFM-CULT-CONFL"));
    }
    EXPECT_THROW(skill::cell_culture::apply(*stock, over),
                 skill::SkillError);
}

// ─── 3. Signature records kind + key params ───────────────────────────────
TEST(SkillCellCulture, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCuboidStock(80.0, 120.0, 25.0);

    auto in = goodInput();
    in.cell_line             = "CHO-K1";
    in.media_type            = "F-12K";
    in.days_in_culture       = 7.0;
    in.target_confluence_pct = 90.0;

    auto out = skill::cell_culture::apply(*stock, in);

    EXPECT_EQ(out.signature.skill_id, std::string("cell_culture"));
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("cell_culture"));
    EXPECT_EQ(out.signature.pattern["cell_line"].get<std::string>(),
              std::string("CHO-K1"));
    EXPECT_EQ(out.signature.pattern["media_type"].get<std::string>(),
              std::string("F-12K"));
    EXPECT_NEAR(out.signature.pattern["days_in_culture"].get<double>(),
                7.0, 1e-9);
    EXPECT_NEAR(out.signature.pattern["target_confluence_pct"].get<double>(),
                90.0, 1e-9);
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
}

// ─── 4. Recognize via metadata replay ─────────────────────────────────────
TEST(SkillCellCulture, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(80.0, 120.0, 25.0);

    auto out   = skill::cell_culture::apply(*stock, goodInput());
    auto cands = skill::cell_culture::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["cell_line"].get<std::string>(),
              std::string("HEK293"));
    EXPECT_EQ(cands[0].recovered_params["media_type"].get<std::string>(),
              std::string("DMEM"));

    skill::Workpiece raw(out.workpiece->shape());
    auto cands2 = skill::cell_culture::recognize(raw);
    EXPECT_TRUE(cands2.empty());
}

// ─── 5. Specific: long-duration run (>14 d) selects bioreactor tool ───────
TEST(SkillCellCulture, LongRunSelectsBioreactor)
{
    auto stock = skill::createCuboidStock(80.0, 120.0, 25.0);

    auto shortRun = goodInput();
    shortRun.days_in_culture = 3.0;
    auto shortOut = skill::cell_culture::apply(*stock, shortRun);
    EXPECT_EQ(shortOut.signature.tooling.tool_type,
              std::string("co2_incubator"));

    auto longRun = goodInput();
    longRun.days_in_culture = 30.0;
    auto longOut = skill::cell_culture::apply(*stock, longRun);
    EXPECT_EQ(longOut.signature.tooling.tool_type,
              std::string("bioreactor"));

    // Bioreactor must reflect proportionally longer cycle time.
    EXPECT_GT(longOut.signature.tooling.est_cycle_time_s,
              shortOut.signature.tooling.est_cycle_time_s);
}
