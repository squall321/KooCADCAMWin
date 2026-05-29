// @lat: [[process/test-strategy#skill round-trip]]
//
// glass_melt skill — metadata-only batch fusion.  Covers:
//   1. apply clones geometry unchanged
//   2. validate rejects bad input (inverted fining vs melt temp)
//   3. signature records kind + key params
//   4. recognize via metadata replay
//   5. batch_composition is faithfully serialized into pattern + params

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/glass_melt.hpp"

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

}  // namespace

// ─── 1. Apply clones geometry unchanged ───────────────────────────────────
TEST(SkillGlassMelt, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(100.0, 100.0, 10.0, "soda_lime_glass");
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    skill::glass_melt::Input in;
    in.melt_temp_c   = 1450.0;
    in.fining_temp_c = 1550.0;

    auto out = skill::glass_melt::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("glass_melt"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. Validate rejects bad input (fining_temp_c < melt_temp_c) ──────────
TEST(SkillGlassMelt, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 5.0, "soda_lime_glass");

    skill::glass_melt::Input in;
    in.melt_temp_c   = 1500.0;
    in.fining_temp_c = 1400.0;       // < melt_temp_c → error

    auto r = skill::glass_melt::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-MELT-FINING") { found = true; break; }
    EXPECT_TRUE(found);
    EXPECT_THROW(skill::glass_melt::apply(*stock, in), skill::SkillError);
}

// ─── 3. Signature records kind + key params ───────────────────────────────
TEST(SkillGlassMelt, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 5.0, "soda_lime_glass");

    skill::glass_melt::Input in;
    in.melt_temp_c   = 1480.0;
    in.fining_temp_c = 1580.0;

    auto out = skill::glass_melt::apply(*stock, in);
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("glass_melt"));
    EXPECT_NEAR(out.signature.pattern["melt_temp_c"].get<double>(),   1480.0, 1e-9);
    EXPECT_NEAR(out.signature.pattern["fining_temp_c"].get<double>(), 1580.0, 1e-9);
    EXPECT_FALSE(out.signature.pattern["geometric_change"].get<bool>());
    EXPECT_EQ(out.signature.tooling.tool_type, std::string("glass_furnace"));
}

// ─── 4. Recognize via metadata replay ─────────────────────────────────────
TEST(SkillGlassMelt, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 6.0, "soda_lime_glass");

    skill::glass_melt::Input in;
    in.melt_temp_c   = 1500.0;
    in.fining_temp_c = 1600.0;

    auto out = skill::glass_melt::apply(*stock, in);
    auto cands = skill::glass_melt::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-9);
    EXPECT_NEAR(cands[0].recovered_params["melt_temp_c"].get<double>(), 1500.0, 1e-9);

    skill::Workpiece raw(out.workpiece->shape());
    auto cands2 = skill::glass_melt::recognize(raw);
    EXPECT_TRUE(cands2.empty())
        << "glass_melt cannot be recognized from B-Rep alone";
}

// ─── 5. Specific: batch_composition round-trips into pattern + params ─────
TEST(SkillGlassMelt, BatchCompositionRoundTrips)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 4.0, "borosilicate_glass");

    skill::glass_melt::Input in;
    in.melt_temp_c       = 1600.0;
    in.fining_temp_c     = 1650.0;
    in.batch_composition = {
        { "SiO2",  81.0 },
        { "B2O3",  12.5 },
        { "Na2O",   4.0 },
        { "Al2O3",  2.5 },
    };

    auto out = skill::glass_melt::apply(*stock, in);
    EXPECT_NEAR(out.signature.pattern["batch_composition"]["SiO2"].get<double>(),
                81.0, 1e-9);
    EXPECT_NEAR(out.signature.pattern["batch_composition"]["B2O3"].get<double>(),
                12.5, 1e-9);
    EXPECT_NEAR(out.signature.params["batch_composition"]["Na2O"].get<double>(),
                4.0, 1e-9);
}
