// @lat: [[process/test-strategy#skill round-trip]]
//
// fermentation — microbial fermentation metadata stamp.  Metadata-only.
// Cases (5):
//   1. ApplyClonesGeometryUnchanged
//   2. ValidateRejectsBadInput
//   3. SignatureRecordsKindAndParams
//   4. RecognizeMetadataReplay
//   5. HighPhTargetEmitsInfo

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/fermentation.hpp"

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
TEST(SkillFermentation, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(30.0, 30.0, 5.0);
    const double volBefore = volumeOf(stock->shape());
    const int faceBefore   = stock->faceCount();
    const int edgeBefore   = stock->edgeCount();

    skill::fermentation::Input in;
    in.temp_c          = 30.0;
    in.time_h          = 24.0;
    in.starter_culture = "lactobacillus_plantarum";
    in.ph_target       = 4.2;

    auto out = skill::fermentation::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. Validate rejects bad input ─────────────────────────────────────────
TEST(SkillFermentation, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(20.0, 20.0, 3.0);

    // Empty starter.
    skill::fermentation::Input noStarter;
    noStarter.temp_c          = 30.0;
    noStarter.time_h          = 24.0;
    noStarter.starter_culture = "";
    noStarter.ph_target       = 4.5;
    {
        auto r = skill::fermentation::validate(*stock, noStarter);
        EXPECT_FALSE(r.passed);
        EXPECT_TRUE(hasFinding(r, "DFM-INPUT"));
    }
    EXPECT_THROW(skill::fermentation::apply(*stock, noStarter),
                 skill::SkillError);

    // Temp too high.
    skill::fermentation::Input hotTemp;
    hotTemp.temp_c          = 75.0;     // > 60
    hotTemp.time_h          = 12.0;
    hotTemp.starter_culture = "saccharomyces_cerevisiae";
    hotTemp.ph_target       = 4.5;
    {
        auto r = skill::fermentation::validate(*stock, hotTemp);
        EXPECT_FALSE(r.passed);
        EXPECT_TRUE(hasFinding(r, "DFM-FERM-TEMP"));
    }

    // pH target out of range low.
    skill::fermentation::Input badPh;
    badPh.temp_c          = 30.0;
    badPh.time_h          = 24.0;
    badPh.starter_culture = "lactobacillus_plantarum";
    badPh.ph_target       = 1.0;        // < 2.5
    {
        auto r = skill::fermentation::validate(*stock, badPh);
        EXPECT_FALSE(r.passed);
        EXPECT_TRUE(hasFinding(r, "DFM-FERM-PH"));
    }
}

// ─── 3. Signature records kind + key params ────────────────────────────────
TEST(SkillFermentation, SignatureRecordsKindAndParams)
{
    auto stock = skill::createCuboidStock(15.0, 15.0, 3.0);

    skill::fermentation::Input in;
    in.temp_c          = 35.0;
    in.time_h          = 18.0;
    in.starter_culture = "streptococcus_thermophilus";
    in.ph_target       = 4.5;

    auto out = skill::fermentation::apply(*stock, in);

    EXPECT_EQ(out.signature.skill_id, std::string("fermentation"));
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("fermentation"));
    EXPECT_NEAR(out.signature.pattern["temp_c"].get<double>(),
                35.0, 1e-9);
    EXPECT_NEAR(out.signature.pattern["time_h"].get<double>(),
                18.0, 1e-9);
    EXPECT_EQ(out.signature.pattern["starter_culture"].get<std::string>(),
              std::string("streptococcus_thermophilus"));
    EXPECT_NEAR(out.signature.pattern["ph_target"].get<double>(),
                4.5, 1e-9);
    EXPECT_FALSE(out.signature.pattern["geometric_change"].get<bool>());
}

// ─── 4. Recognize metadata replay ──────────────────────────────────────────
TEST(SkillFermentation, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(15.0, 15.0, 3.0);

    skill::fermentation::Input in;
    in.temp_c          = 22.0;
    in.time_h          = 72.0;
    in.starter_culture = "saccharomyces_cerevisiae";
    in.ph_target       = 3.8;

    auto out   = skill::fermentation::apply(*stock, in);
    auto cands = skill::fermentation::recognize(*out.workpiece);

    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(
        cands[0].recovered_params["starter_culture"].get<std::string>(),
        std::string("saccharomyces_cerevisiae"));
    EXPECT_NEAR(cands[0].recovered_params["ph_target"].get<double>(),
                3.8, 1e-9);

    // Stripped of metadata → recognize empty.
    skill::Workpiece raw(out.workpiece->shape());
    auto cands2 = skill::fermentation::recognize(raw);
    EXPECT_TRUE(cands2.empty());
}

// ─── 5. Specific: high pH target emits info, still proceeds ────────────────
TEST(SkillFermentation, HighPhTargetEmitsInfo)
{
    auto stock = skill::createCuboidStock(10.0, 10.0, 2.0);

    skill::fermentation::Input in;
    in.temp_c          = 30.0;
    in.time_h          = 24.0;
    in.starter_culture = "lactobacillus_plantarum";
    in.ph_target       = 6.0;       // > 5.5 → info

    auto r = skill::fermentation::validate(*stock, in);
    EXPECT_TRUE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-FERM-PH-HIGH"));

    auto out = skill::fermentation::apply(*stock, in);
    EXPECT_EQ(out.signature.tooling.tool_type,
              std::string("fermentation_vessel"));
    EXPECT_NEAR(out.signature.tooling.est_cycle_time_s,
                24.0 * 3600.0, 1e-6);
}
