// @lat: [[process/test-strategy#skill round-trip]]
//
// deck_assembly skill — 5-case sweep covering identity-geometry synthesis,
// DFM rejection of bad input, signature-records-kind+params,
// metadata-only recognition, and the skill-specific assertion that
// pattern.members_per_m2 == member_count / area_m2.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/deck_assembly.hpp"

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

skill::deck_assembly::Input goodInput()
{
    skill::deck_assembly::Input in;
    in.deck_level   = "main";
    in.area_m2      = 200.0;
    in.member_count = 24;
    return in;
}

}  // namespace

// ─── 1. Apply: geometry CLONED unchanged ──────────────────────────────────
TEST(SkillDeckAssembly, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    auto out = skill::deck_assembly::apply(*stock, goodInput());
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("deck_assembly"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. Validate REJECTS bad input ───────────────────────────────────────
TEST(SkillDeckAssembly, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto in = goodInput();
    in.deck_level = "";                        // empty
    auto r = skill::deck_assembly::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-INPUT"));
    EXPECT_THROW(skill::deck_assembly::apply(*stock, in), skill::SkillError);

    auto in2 = goodInput();
    in2.area_m2 = -1.0;                        // ≤ 0
    auto r2 = skill::deck_assembly::validate(*stock, in2);
    EXPECT_FALSE(r2.passed);
    EXPECT_TRUE(hasFinding(r2, "DFM-INPUT"));

    auto in3 = goodInput();
    in3.member_count = 0;                      // ≤ 0
    auto r3 = skill::deck_assembly::validate(*stock, in3);
    EXPECT_FALSE(r3.passed);
    EXPECT_TRUE(hasFinding(r3, "DFM-INPUT"));
}

// ─── 3. Signature records kind + key params ──────────────────────────────
TEST(SkillDeckAssembly, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto in = goodInput();
    in.deck_level   = "upper";
    in.area_m2      = 350.0;
    in.member_count = 48;
    auto out = skill::deck_assembly::apply(*stock, in);

    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("deck_assembly"));
    EXPECT_EQ(out.signature.pattern["deck_level"].get<std::string>(),
              std::string("upper"));
    EXPECT_NEAR(out.signature.pattern["area_m2"].get<double>(), 350.0, 1e-9);
    EXPECT_EQ(out.signature.pattern["member_count"].get<int>(), 48);
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
}

// ─── 4. Recognize via metadata replay ─────────────────────────────────────
TEST(SkillDeckAssembly, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    auto out = skill::deck_assembly::apply(*stock, goodInput());

    auto cands = skill::deck_assembly::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["deck_level"].get<std::string>(),
              std::string("main"));

    skill::Workpiece raw(out.workpiece->shape());
    auto cands2 = skill::deck_assembly::recognize(raw);
    EXPECT_TRUE(cands2.empty())
        << "deck_assembly cannot be recognized geometrically";
}

// ─── 5. SPECIFIC: members_per_m2 == member_count / area_m2 ────────────────
TEST(SkillDeckAssembly, MembersPerSquareMeterMatchesRatio)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto in = goodInput();
    in.area_m2      = 100.0;
    in.member_count = 25;                      // ratio = 0.25 per m²
    auto out = skill::deck_assembly::apply(*stock, in);

    EXPECT_NEAR(out.signature.pattern["members_per_m2"].get<double>(),
                0.25, 1e-9);
    EXPECT_EQ(out.signature.tooling.tool_type,
              std::string("deck_assembly_jig"));
}
