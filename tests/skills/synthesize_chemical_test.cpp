// @lat: [[process/test-strategy#skill round-trip]]
//
// synthesize_chemical — wet-lab chemical synthesis record.  Metadata-only.
// Cases (5):
//   1. ApplyClonesGeometryUnchanged
//   2. ValidateRejectsBadInput
//   3. SignatureRecordsKindAndKeyParams
//   4. RecognizeMetadataReplay
//   5. HotReactionSelectsRefluxApparatus

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/synthesize_chemical.hpp"

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

skill::synthesize_chemical::Input goodInput()
{
    skill::synthesize_chemical::Input in;
    in.reaction_id     = "RXN-AMIDE-COUPLING-001";
    in.reactants_count = 3;
    in.solvent         = "DMF";
    in.temp_c          = 25.0;
    in.time_h          = 6.0;
    return in;
}

}  // namespace

// ─── 1. Apply clones geometry unchanged ───────────────────────────────────
TEST(SkillSynthesizeChemical, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(80.0, 80.0, 120.0);
    const double volBefore = volumeOf(stock->shape());
    const int faceBefore   = stock->faceCount();
    const int edgeBefore   = stock->edgeCount();

    auto out = skill::synthesize_chemical::apply(*stock, goodInput());
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("synthesize_chemical"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. Validate rejects bad input ────────────────────────────────────────
TEST(SkillSynthesizeChemical, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 100.0);

    // Empty reaction_id.
    auto emptyId = goodInput();
    emptyId.reaction_id = "";
    {
        auto r = skill::synthesize_chemical::validate(*stock, emptyId);
        EXPECT_FALSE(r.passed);
        EXPECT_TRUE(hasFinding(r, "DFM-INPUT"));
    }
    EXPECT_THROW(skill::synthesize_chemical::apply(*stock, emptyId),
                 skill::SkillError);

    // Non-positive time.
    auto zeroTime = goodInput();
    zeroTime.time_h = 0.0;
    {
        auto r = skill::synthesize_chemical::validate(*stock, zeroTime);
        EXPECT_FALSE(r.passed);
        EXPECT_TRUE(hasFinding(r, "DFM-RXN-TIME"));
    }
    EXPECT_THROW(skill::synthesize_chemical::apply(*stock, zeroTime),
                 skill::SkillError);

    // Reactants_count < 1.
    auto noReactants = goodInput();
    noReactants.reactants_count = 0;
    {
        auto r = skill::synthesize_chemical::validate(*stock, noReactants);
        EXPECT_FALSE(r.passed);
        EXPECT_TRUE(hasFinding(r, "DFM-INPUT"));
    }
}

// ─── 3. Signature records kind + key params ───────────────────────────────
TEST(SkillSynthesizeChemical, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCuboidStock(80.0, 80.0, 100.0);

    auto in = goodInput();
    in.reaction_id     = "RXN-SUZUKI-042";
    in.reactants_count = 4;
    in.solvent         = "toluene";
    in.temp_c          = 110.0;
    in.time_h          = 12.0;

    auto out = skill::synthesize_chemical::apply(*stock, in);

    EXPECT_EQ(out.signature.skill_id, std::string("synthesize_chemical"));
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("synthesize_chemical"));
    EXPECT_EQ(out.signature.pattern["reaction_id"].get<std::string>(),
              std::string("RXN-SUZUKI-042"));
    EXPECT_EQ(out.signature.pattern["reactants_count"].get<int>(), 4);
    EXPECT_EQ(out.signature.pattern["solvent"].get<std::string>(),
              std::string("toluene"));
    EXPECT_NEAR(out.signature.pattern["temp_c"].get<double>(), 110.0, 1e-9);
    EXPECT_NEAR(out.signature.pattern["time_h"].get<double>(),  12.0, 1e-9);
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
}

// ─── 4. Recognize via metadata replay ─────────────────────────────────────
TEST(SkillSynthesizeChemical, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(70.0, 70.0, 90.0);

    auto in = goodInput();
    in.reaction_id = "RXN-ESTERIFICATION-007";
    in.solvent     = "ethanol";

    auto out   = skill::synthesize_chemical::apply(*stock, in);
    auto cands = skill::synthesize_chemical::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["reaction_id"].get<std::string>(),
              std::string("RXN-ESTERIFICATION-007"));
    EXPECT_EQ(cands[0].recovered_params["solvent"].get<std::string>(),
              std::string("ethanol"));

    skill::Workpiece raw(out.workpiece->shape());
    auto cands2 = skill::synthesize_chemical::recognize(raw);
    EXPECT_TRUE(cands2.empty());
}

// ─── 5. Specific: hot reaction (>= 80 C) selects reflux apparatus ─────────
TEST(SkillSynthesizeChemical, HotReactionSelectsRefluxApparatus)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 100.0);

    auto cold = goodInput();
    cold.temp_c = 25.0;
    auto coldOut = skill::synthesize_chemical::apply(*stock, cold);
    EXPECT_EQ(coldOut.signature.tooling.tool_type,
              std::string("magnetic_stirrer"));

    auto hot = goodInput();
    hot.temp_c = 120.0;
    auto hotOut = skill::synthesize_chemical::apply(*stock, hot);
    EXPECT_EQ(hotOut.signature.tooling.tool_type,
              std::string("reflux_apparatus"));
}
