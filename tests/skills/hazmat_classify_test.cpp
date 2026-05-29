// @lat: [[process/test-strategy#skill round-trip]]
//
// hazmat_classify — dangerous-goods classification metadata skill.
// 5-case sweep:
//   1. apply clones geometry unchanged
//   2. validate rejects bad input
//   3. signature records kind + un_number + hazard_class + packing_group
//   4. recognize metadata replay
//   5. specific: class 1/2/7 + packing_group ⇒ DFM-HM-PKG-MISMATCH info,
//      booleans is_radioactive/is_explosive/is_gas set correctly

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/hazmat_classify.hpp"

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

skill::hazmat_classify::Input goodInput()
{
    skill::hazmat_classify::Input in;
    in.un_number     = "UN1090";   // acetone
    in.hazard_class  = "3";        // flammable liquid
    in.packing_group = "II";
    return in;
}

}  // namespace

// ─── 1. Apply clones geometry unchanged ──────────────────────────────────
TEST(SkillHazmatClassify, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    auto out = skill::hazmat_classify::apply(*stock, goodInput());
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. Validate rejects bad input ───────────────────────────────────────
TEST(SkillHazmatClassify, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    // Malformed un_number → error.
    auto in1 = goodInput();
    in1.un_number = "1090";        // missing "UN" prefix
    auto r1 = skill::hazmat_classify::validate(*stock, in1);
    EXPECT_FALSE(r1.passed);
    EXPECT_TRUE(hasFinding(r1, "DFM-HM-UNNUMBER"));
    EXPECT_THROW(skill::hazmat_classify::apply(*stock, in1),
                 skill::SkillError);

    // Empty un_number → error.
    auto in2 = goodInput();
    in2.un_number = "";
    auto r2 = skill::hazmat_classify::validate(*stock, in2);
    EXPECT_FALSE(r2.passed);
    EXPECT_TRUE(hasFinding(r2, "DFM-INPUT"));

    // Out-of-range hazard_class → error.
    auto in3 = goodInput();
    in3.hazard_class = "10";
    auto r3 = skill::hazmat_classify::validate(*stock, in3);
    EXPECT_FALSE(r3.passed);
    EXPECT_TRUE(hasFinding(r3, "DFM-HM-CLASS"));

    // Unknown packing_group → error.
    auto in4 = goodInput();
    in4.packing_group = "IV";
    auto r4 = skill::hazmat_classify::validate(*stock, in4);
    EXPECT_FALSE(r4.passed);
    EXPECT_TRUE(hasFinding(r4, "DFM-HM-PKG"));
}

// ─── 3. Signature records kind + key params ──────────────────────────────
TEST(SkillHazmatClassify, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::hazmat_classify::Input in;
    in.un_number     = "UN1950";    // aerosols
    in.hazard_class  = "2";         // gases
    in.packing_group = "";          // class 2 has no PG

    auto out = skill::hazmat_classify::apply(*stock, in);

    EXPECT_EQ(out.signature.skill_id, std::string("hazmat_classify"));
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("hazmat_classify"));
    EXPECT_EQ(out.signature.pattern["un_number"].get<std::string>(),
              std::string("UN1950"));
    EXPECT_EQ(out.signature.pattern["hazard_class"].get<std::string>(),
              std::string("2"));
    EXPECT_EQ(out.signature.pattern["packing_group"].get<std::string>(),
              std::string(""));
    EXPECT_FALSE(out.signature.pattern["geometric_change"].get<bool>());
}

// ─── 4. Recognize metadata replay ────────────────────────────────────────
TEST(SkillHazmatClassify, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);
    auto out = skill::hazmat_classify::apply(*stock, goodInput());

    auto cands = skill::hazmat_classify::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].skill_id, std::string("hazmat_classify"));
    EXPECT_EQ(cands[0].recovered_params["un_number"].get<std::string>(),
              std::string("UN1090"));
    EXPECT_EQ(cands[0].recovered_params["hazard_class"].get<std::string>(),
              std::string("3"));

    // Stripped workpiece → recognize empty.
    skill::Workpiece raw(out.workpiece->shape(), out.workpiece->material());
    EXPECT_TRUE(skill::hazmat_classify::recognize(raw).empty());
}

// ─── 5. Specific: class 1/2/7 mismatch + class booleans ──────────────────
TEST(SkillHazmatClassify, ClassNoPackingGroupMismatch)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    // Class 1 (explosives) WITH a packing group → info finding +
    // is_explosive=true, packing_group_effective="".
    skill::hazmat_classify::Input expl;
    expl.un_number     = "UN0027";    // black powder
    expl.hazard_class  = "1";
    expl.packing_group = "II";         // SHOULD NOT BE SET
    auto rE = skill::hazmat_classify::validate(*stock, expl);
    EXPECT_TRUE(rE.passed);            // info-only
    EXPECT_TRUE(hasFinding(rE, "DFM-HM-PKG-MISMATCH"));
    auto outE = skill::hazmat_classify::apply(*stock, expl);
    EXPECT_TRUE(outE.signature.pattern["is_explosive"].get<bool>());
    EXPECT_FALSE(outE.signature.pattern["is_radioactive"].get<bool>());
    EXPECT_FALSE(outE.signature.pattern["is_gas"].get<bool>());
    EXPECT_EQ(outE.signature.pattern["packing_group_effective"].get<std::string>(),
              std::string(""));

    // Class 7 (radioactive) with a packing group → also info finding.
    skill::hazmat_classify::Input rad;
    rad.un_number     = "UN2912";      // radioactive material LSA-I
    rad.hazard_class  = "7";
    rad.packing_group = "III";
    auto rR = skill::hazmat_classify::validate(*stock, rad);
    EXPECT_TRUE(rR.passed);
    EXPECT_TRUE(hasFinding(rR, "DFM-HM-PKG-MISMATCH"));
    auto outR = skill::hazmat_classify::apply(*stock, rad);
    EXPECT_TRUE(outR.signature.pattern["is_radioactive"].get<bool>());

    // Class 3 with proper packing_group → NO mismatch finding.
    auto good = goodInput();
    auto rG = skill::hazmat_classify::validate(*stock, good);
    EXPECT_TRUE(rG.passed);
    EXPECT_FALSE(hasFinding(rG, "DFM-HM-PKG-MISMATCH"));
    auto outG = skill::hazmat_classify::apply(*stock, good);
    EXPECT_EQ(outG.signature.pattern["packing_group_effective"].get<std::string>(),
              std::string("II"));
    EXPECT_FALSE(outG.signature.pattern["is_explosive"].get<bool>());
    EXPECT_FALSE(outG.signature.pattern["is_radioactive"].get<bool>());
    EXPECT_FALSE(outG.signature.pattern["is_gas"].get<bool>());
}
