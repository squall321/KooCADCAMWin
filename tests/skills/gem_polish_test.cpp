// @lat: [[process/test-strategy#skill round-trip]]
//
// gem_polish — facet polishing to target lustre class.  Metadata-only.
// Cases (5):
//   1. ApplyClonesGeometryUnchanged
//   2. ValidateRejectsBadInput
//   3. SignatureRecordsKindAndParams
//   4. RecognizeMetadataReplay
//   5. ExpectedRaScalesInverselyWithGrit

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/gem_polish.hpp"

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
TEST(SkillGemPolish, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(8.0, 8.0, 5.0);
    const double volBefore = volumeOf(stock->shape());
    const int faceBefore   = stock->faceCount();
    const int edgeBefore   = stock->edgeCount();

    skill::gem_polish::Input in;
    in.gem_species         = "diamond";
    in.target_polish_class = "excellent";
    in.lap_grit            = 50000;

    auto out = skill::gem_polish::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("gem_polish"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. Validate rejects bad input ────────────────────────────────────────
TEST(SkillGemPolish, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(8.0, 8.0, 5.0);

    skill::gem_polish::Input badClass;
    badClass.gem_species         = "diamond";
    badClass.target_polish_class = "perfect";  // not in class set
    badClass.lap_grit            = 50000;
    {
        auto r = skill::gem_polish::validate(*stock, badClass);
        EXPECT_FALSE(r.passed);
        EXPECT_TRUE(hasFinding(r, "DFM-POLISH-CLASS"));
    }
    EXPECT_THROW(skill::gem_polish::apply(*stock, badClass), skill::SkillError);

    skill::gem_polish::Input zeroGrit;
    zeroGrit.gem_species         = "diamond";
    zeroGrit.target_polish_class = "excellent";
    zeroGrit.lap_grit            = 0;
    {
        auto r = skill::gem_polish::validate(*stock, zeroGrit);
        EXPECT_FALSE(r.passed);
        EXPECT_TRUE(hasFinding(r, "DFM-LAP-GRIT"));
    }
    EXPECT_THROW(skill::gem_polish::apply(*stock, zeroGrit), skill::SkillError);
}

// ─── 3. Signature records kind + key params ───────────────────────────────
TEST(SkillGemPolish, SignatureRecordsKindAndParams)
{
    auto stock = skill::createCuboidStock(8.0, 8.0, 5.0);

    skill::gem_polish::Input in;
    in.gem_species         = "sapphire";
    in.target_polish_class = "very_good";
    in.lap_grit            = 14000;

    auto out = skill::gem_polish::apply(*stock, in);

    EXPECT_EQ(out.signature.skill_id, std::string("gem_polish"));
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("gem_polish"));
    EXPECT_EQ(out.signature.pattern["gem_species"].get<std::string>(),
              std::string("sapphire"));
    EXPECT_EQ(out.signature.pattern["target_polish_class"].get<std::string>(),
              std::string("very_good"));
    EXPECT_EQ(out.signature.pattern["lap_grit"].get<int>(), 14000);
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
}

// ─── 4. Recognize metadata replay ─────────────────────────────────────────
TEST(SkillGemPolish, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(8.0, 8.0, 5.0);

    skill::gem_polish::Input in;
    in.gem_species         = "diamond";
    in.target_polish_class = "excellent";
    in.lap_grit            = 100000;

    auto out   = skill::gem_polish::apply(*stock, in);
    auto cands = skill::gem_polish::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["target_polish_class"].get<std::string>(),
              std::string("excellent"));
    EXPECT_EQ(cands[0].recovered_params["lap_grit"].get<int>(), 100000);

    skill::Workpiece raw(out.workpiece->shape());
    auto cands2 = skill::gem_polish::recognize(raw);
    EXPECT_TRUE(cands2.empty());
}

// ─── 5. Specific: Ra inversely scales with grit (50000/grit, capped 200) ──
TEST(SkillGemPolish, ExpectedRaScalesInverselyWithGrit)
{
    auto stock = skill::createCuboidStock(8.0, 8.0, 5.0);

    skill::gem_polish::Input fine;
    fine.gem_species         = "diamond";
    fine.target_polish_class = "excellent";
    fine.lap_grit            = 50000;
    auto outFine = skill::gem_polish::apply(*stock, fine);

    skill::gem_polish::Input coarse;
    coarse.gem_species         = "diamond";
    coarse.target_polish_class = "good";
    coarse.lap_grit            = 2000;
    auto outCoarse = skill::gem_polish::apply(*stock, coarse);

    const double raFine   = outFine  .signature.pattern["expected_surface_ra_nm"].get<double>();
    const double raCoarse = outCoarse.signature.pattern["expected_surface_ra_nm"].get<double>();
    EXPECT_NEAR(raFine, 1.0, 1e-6);          // 50000 / 50000
    EXPECT_NEAR(raCoarse, 25.0, 1e-6);       // 50000 / 2000
    EXPECT_LT(raFine, raCoarse);
}
