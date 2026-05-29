// @lat: [[process/test-strategy#skill round-trip]]
//
// customs_declaration — trade-compliance metadata skill.  5-case sweep:
//   1. apply clones geometry unchanged
//   2. validate rejects bad input
//   3. signature records kind + hs_code + country_origin + value_usd
//   4. recognize metadata replay
//   5. specific: entry_type classified by value (de_minimis / informal / formal)

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/customs_declaration.hpp"

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

skill::customs_declaration::Input goodInput()
{
    skill::customs_declaration::Input in;
    in.hs_code        = "847790";       // 6-digit HS
    in.country_origin = "KR";
    in.value_usd      = 25000.0;        // informal entry range
    return in;
}

}  // namespace

// ─── 1. Apply clones geometry unchanged ──────────────────────────────────
TEST(SkillCustomsDeclaration, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    auto out = skill::customs_declaration::apply(*stock, goodInput());
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. Validate rejects bad input ───────────────────────────────────────
TEST(SkillCustomsDeclaration, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    // Empty hs_code → error.
    auto in1 = goodInput();
    in1.hs_code = "";
    auto r1 = skill::customs_declaration::validate(*stock, in1);
    EXPECT_FALSE(r1.passed);
    EXPECT_TRUE(hasFinding(r1, "DFM-INPUT"));
    EXPECT_THROW(skill::customs_declaration::apply(*stock, in1),
                 skill::SkillError);

    // Malformed hs_code (alpha) → error.
    auto in2 = goodInput();
    in2.hs_code = "84AB90";
    auto r2 = skill::customs_declaration::validate(*stock, in2);
    EXPECT_FALSE(r2.passed);
    EXPECT_TRUE(hasFinding(r2, "DFM-CUS-HSCODE"));

    // Bad country_origin (1 char) → error.
    auto in3 = goodInput();
    in3.country_origin = "K";
    auto r3 = skill::customs_declaration::validate(*stock, in3);
    EXPECT_FALSE(r3.passed);
    EXPECT_TRUE(hasFinding(r3, "DFM-CUS-ORIGIN"));

    // Non-positive value → error.
    auto in4 = goodInput();
    in4.value_usd = 0.0;
    auto r4 = skill::customs_declaration::validate(*stock, in4);
    EXPECT_FALSE(r4.passed);
    EXPECT_TRUE(hasFinding(r4, "DFM-INPUT"));
}

// ─── 3. Signature records kind + key params ──────────────────────────────
TEST(SkillCustomsDeclaration, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::customs_declaration::Input in;
    in.hs_code        = "8471300000";  // 10-digit
    in.country_origin = "USA";
    in.value_usd      = 12500.0;

    auto out = skill::customs_declaration::apply(*stock, in);

    EXPECT_EQ(out.signature.skill_id,
              std::string("customs_declaration"));
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("customs_declaration"));
    EXPECT_EQ(out.signature.pattern["hs_code"].get<std::string>(),
              std::string("8471300000"));
    EXPECT_EQ(out.signature.pattern["country_origin"].get<std::string>(),
              std::string("USA"));
    EXPECT_NEAR(out.signature.pattern["value_usd"].get<double>(),
                12500.0, 1e-9);
    EXPECT_EQ(out.signature.pattern["hs_code_length"].get<int>(), 10);
    EXPECT_FALSE(out.signature.pattern["geometric_change"].get<bool>());
}

// ─── 4. Recognize metadata replay ────────────────────────────────────────
TEST(SkillCustomsDeclaration, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);
    auto out = skill::customs_declaration::apply(*stock, goodInput());

    auto cands = skill::customs_declaration::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].skill_id, std::string("customs_declaration"));
    EXPECT_EQ(cands[0].recovered_params["hs_code"].get<std::string>(),
              std::string("847790"));
    EXPECT_NEAR(cands[0].recovered_params["value_usd"].get<double>(),
                25000.0, 1e-9);

    // Stripped workpiece → recognize empty.
    skill::Workpiece raw(out.workpiece->shape(), out.workpiece->material());
    EXPECT_TRUE(skill::customs_declaration::recognize(raw).empty());
}

// ─── 5. Specific: entry_type classified by value ─────────────────────────
TEST(SkillCustomsDeclaration, EntryTypeClassifiedByValue)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    // De minimis: value_usd < 800 → "de_minimis" entry_type + info finding.
    auto small = goodInput();
    small.value_usd = 500.0;
    auto rS = skill::customs_declaration::validate(*stock, small);
    EXPECT_TRUE(rS.passed);              // info-only, not error
    EXPECT_TRUE(hasFinding(rS, "DFM-CUS-VALUE-LOW"));
    auto outS = skill::customs_declaration::apply(*stock, small);
    EXPECT_EQ(outS.signature.pattern["entry_type"].get<std::string>(),
              std::string("de_minimis"));

    // Informal: between 800 and 1,000,000.
    auto mid = goodInput();
    mid.value_usd = 50000.0;
    auto outM = skill::customs_declaration::apply(*stock, mid);
    EXPECT_EQ(outM.signature.pattern["entry_type"].get<std::string>(),
              std::string("informal"));

    // Formal: > 1,000,000 → info finding + "formal" entry_type.
    auto big = goodInput();
    big.value_usd = 2500000.0;
    auto rB = skill::customs_declaration::validate(*stock, big);
    EXPECT_TRUE(rB.passed);
    EXPECT_TRUE(hasFinding(rB, "DFM-CUS-VALUE-HIGH"));
    auto outB = skill::customs_declaration::apply(*stock, big);
    EXPECT_EQ(outB.signature.pattern["entry_type"].get<std::string>(),
              std::string("formal"));
}
