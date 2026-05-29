// @lat: [[process/test-strategy#skill round-trip]]
//
// blast_mining — production blast record.  Metadata-only.
// Cases (5):
//   1. ApplyClonesGeometryUnchanged
//   2. ValidateRejectsBadInput
//   3. SignatureRecordsKindAndParams
//   4. RecognizeMetadataReplay
//   5. TotalChargeAndEnergyComputedCorrectly

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/blast_mining.hpp"

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
TEST(SkillBlastMining, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(500.0, 500.0, 100.0);
    const double volBefore = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    skill::blast_mining::Input in;
    in.pattern_holes      = 48;
    in.charge_per_hole_kg = 120.0;
    in.explosive_type     = "anfo";

    auto out = skill::blast_mining::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("blast_mining"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. Validate rejects bad input ────────────────────────────────────────
TEST(SkillBlastMining, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(200.0, 200.0, 50.0);

    skill::blast_mining::Input badHoles;
    badHoles.pattern_holes      = 0;
    badHoles.charge_per_hole_kg = 100.0;
    badHoles.explosive_type     = "anfo";
    {
        auto r = skill::blast_mining::validate(*stock, badHoles);
        EXPECT_FALSE(r.passed);
        EXPECT_TRUE(hasFinding(r, "DFM-INPUT"));
    }
    EXPECT_THROW(skill::blast_mining::apply(*stock, badHoles), skill::SkillError);

    skill::blast_mining::Input badExp;
    badExp.pattern_holes      = 24;
    badExp.charge_per_hole_kg = 100.0;
    badExp.explosive_type     = "dynamite";
    {
        auto r = skill::blast_mining::validate(*stock, badExp);
        EXPECT_FALSE(r.passed);
        EXPECT_TRUE(hasFinding(r, "DFM-EXPLOSIVE-TYPE"));
    }
    EXPECT_THROW(skill::blast_mining::apply(*stock, badExp), skill::SkillError);
}

// ─── 3. Signature records kind + key params ───────────────────────────────
TEST(SkillBlastMining, SignatureRecordsKindAndParams)
{
    auto stock = skill::createCuboidStock(200.0, 200.0, 50.0);

    skill::blast_mining::Input in;
    in.pattern_holes      = 36;
    in.charge_per_hole_kg = 150.0;
    in.explosive_type     = "emulsion";

    auto out = skill::blast_mining::apply(*stock, in);

    EXPECT_EQ(out.signature.skill_id, std::string("blast_mining"));
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("blast_mining"));
    EXPECT_EQ(out.signature.pattern["pattern_holes"].get<int>(), 36);
    EXPECT_NEAR(out.signature.pattern["charge_per_hole_kg"].get<double>(),
                150.0, 1e-9);
    EXPECT_EQ(out.signature.pattern["explosive_type"].get<std::string>(),
              std::string("emulsion"));
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
}

// ─── 4. Recognize metadata replay ─────────────────────────────────────────
TEST(SkillBlastMining, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(200.0, 200.0, 50.0);

    skill::blast_mining::Input in;
    in.pattern_holes      = 24;
    in.charge_per_hole_kg = 90.0;
    in.explosive_type     = "watergel";

    auto out   = skill::blast_mining::apply(*stock, in);
    auto cands = skill::blast_mining::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["explosive_type"].get<std::string>(),
              std::string("watergel"));
    EXPECT_EQ(cands[0].recovered_params["pattern_holes"].get<int>(), 24);

    skill::Workpiece raw(out.workpiece->shape());
    auto cands2 = skill::blast_mining::recognize(raw);
    EXPECT_TRUE(cands2.empty());
}

// ─── 5. Specific: total charge + energy computed correctly ───────────────
TEST(SkillBlastMining, TotalChargeAndEnergyComputedCorrectly)
{
    auto stock = skill::createCuboidStock(200.0, 200.0, 50.0);

    skill::blast_mining::Input in;
    in.pattern_holes      = 50;
    in.charge_per_hole_kg = 100.0;
    in.explosive_type     = "anfo";   // 3.7 MJ/kg

    auto out = skill::blast_mining::apply(*stock, in);
    EXPECT_NEAR(out.signature.pattern["total_charge_kg"].get<double>(),
                5000.0, 1e-6);
    EXPECT_NEAR(out.signature.pattern["total_energy_MJ"].get<double>(),
                5000.0 * 3.7, 1e-6);
    EXPECT_EQ(out.signature.tooling.tool_type, std::string("anfo_loader"));
}
