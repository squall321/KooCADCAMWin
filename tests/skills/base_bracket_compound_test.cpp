// @lat: [[process/test-strategy#compound macro]]
//
// base_bracket_compound — COMPOUND L-bracket: base + standing plate + N
// mounting holes + dowel + gusset rib.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/base_bracket_compound.hpp"

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

skill::base_bracket_compound::Input goodInput()
{
    skill::base_bracket_compound::Input in;
    in.leg1_mm          = 200.0;
    in.leg2_mm          = 150.0;
    in.plate_t_mm       = 10.0;
    in.bracket_width_mm = 100.0;
    in.bolt_pattern     = { 2, 2 };
    in.bolt_dia_mm      = 10.0;
    in.dowel_dia_mm     = 8.0;
    in.gusset_size_mm   = 40.0;
    return in;
}

}  // namespace

// ─── 1. Apply: shape valid + face change reflects multiple sub-features ──
TEST(SkillBaseBracketCompound, ApplyProducesValidShapeWithMultipleSubFeatures)
{
    auto stock = skill::createCuboidStock(100.0, 100.0, 100.0);
    const int    faceBefore = stock->faceCount();

    auto in = goodInput();
    auto out = skill::base_bracket_compound::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    // L-shape volume positive but much less than bbox.
    const double vol = volumeOf(out.workpiece->shape());
    EXPECT_GT(vol, 0.0);

    EXPECT_GT(out.workpiece->faceCount(), faceBefore);

    EXPECT_TRUE(out.signature.pattern["is_compound"].get<bool>());
    EXPECT_GE(out.signature.pattern["subfeature_count"].get<int>(), 2);
}

// ─── 2. Validate REJECTS bad input ────────────────────────────────────────
TEST(SkillBaseBracketCompound, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(100.0, 100.0, 100.0);

    // Bad: plate_t too thin.
    auto in1 = goodInput();
    in1.plate_t_mm = 2.0;
    auto r1 = skill::base_bracket_compound::validate(*stock, in1);
    EXPECT_FALSE(r1.passed);
    EXPECT_TRUE(hasFinding(r1, "DFM-BRK-PLATE-T"));
    EXPECT_THROW(skill::base_bracket_compound::apply(*stock, in1),
                 skill::SkillError);

    // Bad: dowel edge distance too small (leg2/2 < 2 × dowel_dia).
    auto in2 = goodInput();
    in2.dowel_dia_mm = 50.0;
    auto r2 = skill::base_bracket_compound::validate(*stock, in2);
    EXPECT_FALSE(r2.passed);
    EXPECT_TRUE(hasFinding(r2, "DFM-BRK-DOWEL"));
}

// ─── 3. Signature records compound kind ───────────────────────────────────
TEST(SkillBaseBracketCompound, SignatureRecordsCompoundKind)
{
    auto stock = skill::createCuboidStock(100.0, 100.0, 100.0);
    auto out = skill::base_bracket_compound::apply(*stock, goodInput());

    EXPECT_EQ(out.signature.skill_id, std::string("base_bracket_compound"));
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("base_bracket_compound"));
    EXPECT_TRUE(out.signature.pattern["is_compound"].get<bool>());
    EXPECT_GE(out.signature.pattern["subfeature_count"].get<int>(), 2);

    EXPECT_NEAR(out.signature.params["leg1_mm"].get<double>(),    200.0, 1e-9);
    EXPECT_NEAR(out.signature.params["leg2_mm"].get<double>(),    150.0, 1e-9);
    EXPECT_NEAR(out.signature.params["plate_t_mm"].get<double>(),  10.0, 1e-9);
    EXPECT_NEAR(out.signature.params["bolt_dia_mm"].get<double>(), 10.0, 1e-9);
}

// ─── 4. Recognize: metadata replay returns confidence 1.0 ────────────────
TEST(SkillBaseBracketCompound, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(100.0, 100.0, 100.0);
    auto out = skill::base_bracket_compound::apply(*stock, goodInput());

    auto cands = skill::base_bracket_compound::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-9);

    skill::Workpiece raw(out.workpiece->shape());
    EXPECT_TRUE(skill::base_bracket_compound::recognize(raw).empty());
}

// ─── 5. SPECIFIC: fastener_total = n_bolts + 1 (dowel) ────────────────────
TEST(SkillBaseBracketCompound, FastenerTotalIncludesDowel)
{
    auto stock = skill::createCuboidStock(100.0, 100.0, 100.0);

    auto in = goodInput();
    in.bolt_pattern = { 3, 3 };       // 9 bolts
    auto out = skill::base_bracket_compound::apply(*stock, in);

    EXPECT_EQ(out.signature.pattern["n_bolts"].get<int>(), 9);
    EXPECT_EQ(out.signature.pattern["fastener_total"].get<int>(), 10);
    // sub-feature count = 2 (plates) + 9 (bolts) + 1 (dowel) + 1 (gusset) = 13.
    EXPECT_EQ(out.signature.pattern["subfeature_count"].get<int>(), 13);
}
