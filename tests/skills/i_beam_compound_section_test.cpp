// @lat: [[process/test-strategy#compound macro]]
//
// i_beam_compound_section — COMPOUND structural skill: top flange + web +
// bottom flange fused into an I-section.  5-case sweep.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/i_beam_compound_section.hpp"

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

skill::i_beam_compound_section::Input goodInput()
{
    skill::i_beam_compound_section::Input in;
    in.length_mm   = 1000.0;
    in.height_mm   = 200.0;
    in.flange_w_mm = 150.0;
    in.flange_t_mm = 10.0;
    in.web_t_mm    = 6.0;
    return in;
}

}  // namespace

// ─── 1. Apply: volume change reflects 3 fused sub-features ────────────────
TEST(SkillIBeamCompoundSection, ApplyProducesValidShapeWithMultipleSubFeatures)
{
    auto stock = skill::createCuboidStock(100.0, 100.0, 100.0);
    const int    faceBefore = stock->faceCount();

    auto in = goodInput();
    auto out = skill::i_beam_compound_section::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    // Expected section area = 2 × flange_w × flange_t + (height - 2 tf) × web_t
    const double webH = in.height_mm - 2.0 * in.flange_t_mm;
    const double areaExpected = 2.0 * in.flange_w_mm * in.flange_t_mm +
                                webH * in.web_t_mm;
    const double volExpected = areaExpected * in.length_mm;
    // Allow generous 5% tolerance for fused boolean precision.
    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volExpected, volExpected * 0.05);

    // Face-count change is consistent with a 3-piece fused shape — at least
    // more faces than a simple stock cuboid (6).
    EXPECT_GT(out.workpiece->faceCount(), faceBefore);

    // Compound metadata in the signature.
    EXPECT_TRUE(out.signature.pattern["is_compound"].get<bool>());
    EXPECT_EQ(out.signature.pattern["subfeature_count"].get<int>(), 3);
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("i_beam_compound_section"));
}

// ─── 2. Validate REJECTS bad input ────────────────────────────────────────
TEST(SkillIBeamCompoundSection, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(100.0, 100.0, 100.0);

    // Bad: flange_t × 2 >= height (no web).
    auto in1 = goodInput();
    in1.flange_t_mm = 100.0;
    in1.height_mm   = 150.0;
    auto r1 = skill::i_beam_compound_section::validate(*stock, in1);
    EXPECT_FALSE(r1.passed);
    EXPECT_TRUE(hasFinding(r1, "DFM-IBEAM-GEOM"));
    EXPECT_THROW(skill::i_beam_compound_section::apply(*stock, in1),
                 skill::SkillError);

    // Bad: zero dimension.
    auto in2 = goodInput();
    in2.web_t_mm = 0.0;
    auto r2 = skill::i_beam_compound_section::validate(*stock, in2);
    EXPECT_FALSE(r2.passed);
    EXPECT_TRUE(hasFinding(r2, "DFM-IBEAM-INPUT"));
}

// ─── 3. Signature records compound kind ───────────────────────────────────
TEST(SkillIBeamCompoundSection, SignatureRecordsCompoundKind)
{
    auto stock = skill::createCuboidStock(100.0, 100.0, 100.0);
    auto out = skill::i_beam_compound_section::apply(*stock, goodInput());

    EXPECT_EQ(out.signature.skill_id, std::string("i_beam_compound_section"));
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("i_beam_compound_section"));
    EXPECT_TRUE(out.signature.pattern["is_compound"].get<bool>());
    EXPECT_GE(out.signature.pattern["subfeature_count"].get<int>(), 2);

    // Verbatim params present.
    EXPECT_NEAR(out.signature.params["length_mm"].get<double>(),   1000.0, 1e-9);
    EXPECT_NEAR(out.signature.params["height_mm"].get<double>(),    200.0, 1e-9);
    EXPECT_NEAR(out.signature.params["flange_w_mm"].get<double>(),  150.0, 1e-9);
    EXPECT_NEAR(out.signature.params["flange_t_mm"].get<double>(),   10.0, 1e-9);
    EXPECT_NEAR(out.signature.params["web_t_mm"].get<double>(),       6.0, 1e-9);
}

// ─── 4. Recognize: metadata replay returns confidence 1.0 ────────────────
TEST(SkillIBeamCompoundSection, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(100.0, 100.0, 100.0);
    auto out = skill::i_beam_compound_section::apply(*stock, goodInput());

    auto cands = skill::i_beam_compound_section::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-9);
    EXPECT_NEAR(cands[0].recovered_params["height_mm"].get<double>(),
                200.0, 1e-9);

    // Strip metadata → recognize empty.
    skill::Workpiece raw(out.workpiece->shape());
    auto cands2 = skill::i_beam_compound_section::recognize(raw);
    EXPECT_TRUE(cands2.empty());
}

// ─── 5. SPECIFIC: total height = web + 2 × flange_t ───────────────────────
TEST(SkillIBeamCompoundSection, TotalHeightMatchesWebPlusTwoFlanges)
{
    auto stock = skill::createCuboidStock(100.0, 100.0, 100.0);
    auto in = goodInput();
    auto out = skill::i_beam_compound_section::apply(*stock, in);

    const double webH = out.signature.pattern["web_height_mm"].get<double>();
    const double flT  = out.signature.pattern["flange_t_mm"].get<double>();
    const double total = webH + 2.0 * flT;
    EXPECT_NEAR(total, in.height_mm, 1e-9);

    // And: pattern stores section_area = 2 × bf × tf + hw × tw.
    const double areaExpected = 2.0 * in.flange_w_mm * in.flange_t_mm +
                                webH * in.web_t_mm;
    EXPECT_NEAR(out.signature.pattern["section_area_mm2"].get<double>(),
                areaExpected, 1e-6);
}
