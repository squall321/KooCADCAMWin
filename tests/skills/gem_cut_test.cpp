// @lat: [[process/test-strategy#skill round-trip]]
//
// gem_cut — primary faceting of a rough gemstone.  Metadata-only.
// Cases (5):
//   1. ApplyClonesGeometryUnchanged
//   2. ValidateRejectsBadInput
//   3. SignatureRecordsKindAndParams
//   4. RecognizeMetadataReplay
//   5. RoundStyleProducesFiftySevenFacets

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/gem_cut.hpp"

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
TEST(SkillGemCut, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(10.0, 10.0, 6.0);
    const double volBefore = volumeOf(stock->shape());
    const int faceBefore   = stock->faceCount();
    const int edgeBefore   = stock->edgeCount();

    skill::gem_cut::Input in;
    in.gem_species = "diamond";
    in.cut_style   = "round";
    in.carat       = 1.0;

    auto out = skill::gem_cut::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("gem_cut"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. Validate rejects bad input ────────────────────────────────────────
TEST(SkillGemCut, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(8.0, 8.0, 6.0);

    skill::gem_cut::Input badStyle;
    badStyle.gem_species = "diamond";
    badStyle.cut_style   = "octahedron";   // not in {round, oval, princess}
    badStyle.carat       = 1.0;
    {
        auto r = skill::gem_cut::validate(*stock, badStyle);
        EXPECT_FALSE(r.passed);
        EXPECT_TRUE(hasFinding(r, "DFM-CUT-STYLE"));
    }
    EXPECT_THROW(skill::gem_cut::apply(*stock, badStyle), skill::SkillError);

    skill::gem_cut::Input zeroCarat;
    zeroCarat.gem_species = "diamond";
    zeroCarat.cut_style   = "round";
    zeroCarat.carat       = 0.0;
    {
        auto r = skill::gem_cut::validate(*stock, zeroCarat);
        EXPECT_FALSE(r.passed);
        EXPECT_TRUE(hasFinding(r, "DFM-CARAT"));
    }
    EXPECT_THROW(skill::gem_cut::apply(*stock, zeroCarat), skill::SkillError);
}

// ─── 3. Signature records kind + key params ───────────────────────────────
TEST(SkillGemCut, SignatureRecordsKindAndParams)
{
    auto stock = skill::createCuboidStock(10.0, 10.0, 6.0);

    skill::gem_cut::Input in;
    in.gem_species = "sapphire";
    in.cut_style   = "oval";
    in.carat       = 2.5;

    auto out = skill::gem_cut::apply(*stock, in);

    EXPECT_EQ(out.signature.skill_id, std::string("gem_cut"));
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("gem_cut"));
    EXPECT_EQ(out.signature.pattern["gem_species"].get<std::string>(),
              std::string("sapphire"));
    EXPECT_EQ(out.signature.pattern["cut_style"].get<std::string>(),
              std::string("oval"));
    EXPECT_NEAR(out.signature.pattern["carat"].get<double>(), 2.5, 1e-9);
    EXPECT_NEAR(out.signature.pattern["mass_g"].get<double>(), 0.5, 1e-9);
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
}

// ─── 4. Recognize metadata replay ─────────────────────────────────────────
TEST(SkillGemCut, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(10.0, 10.0, 6.0);

    skill::gem_cut::Input in;
    in.gem_species = "ruby";
    in.cut_style   = "princess";
    in.carat       = 0.75;

    auto out   = skill::gem_cut::apply(*stock, in);
    auto cands = skill::gem_cut::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["gem_species"].get<std::string>(),
              std::string("ruby"));
    EXPECT_EQ(cands[0].recovered_params["cut_style"].get<std::string>(),
              std::string("princess"));
    EXPECT_NEAR(cands[0].recovered_params["carat"].get<double>(), 0.75, 1e-9);

    skill::Workpiece raw(out.workpiece->shape());
    auto cands2 = skill::gem_cut::recognize(raw);
    EXPECT_TRUE(cands2.empty());
}

// ─── 5. Specific: round brilliant gives 57 facets ─────────────────────────
TEST(SkillGemCut, RoundStyleProducesFiftySevenFacets)
{
    auto stock = skill::createCuboidStock(10.0, 10.0, 6.0);

    skill::gem_cut::Input in;
    in.gem_species = "diamond";
    in.cut_style   = "round";
    in.carat       = 1.0;

    auto out = skill::gem_cut::apply(*stock, in);
    EXPECT_EQ(out.signature.pattern["facet_count"].get<int>(), 57);
    EXPECT_EQ(out.signature.tooling.tool_type, std::string("facet_lap"));
}
