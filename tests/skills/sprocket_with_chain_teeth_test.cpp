// @lat: [[process/test-strategy#real compound feature]]
//
// sprocket_with_chain_teeth — REAL compound feature: cylindrical blank →
// 1 bore + N roller-pocket cuts per ANSI B29.1.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/sprocket_with_chain_teeth.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

#include <cmath>

using namespace koocadcam;

namespace {

double volumeOf(const TopoDS_Shape& s)
{
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}

skill::sprocket_with_chain_teeth::Input goodInput()
{
    skill::sprocket_with_chain_teeth::Input in;
    in.blank_dia_mm     = 100.0;     // PD for #40 N=20 = 12.7 / sin(9 deg) ≈ 81.2; OD ≈ 89
    in.blank_thick_mm   = 8.0;
    in.ansi_chain_size  = "40";
    in.num_teeth        = 20;
    in.bore_dia_mm      = 25.0;
    return in;
}

}  // namespace

// ─── 1. apply() — REAL volume delta + face count growth ──────────────────
TEST(SkillSprocketWithChainTeeth, ApplyRemovesRealGeometry)
{
    auto stock = skill::createCylindricalStock(100.0, 8.0);
    const double vStock = volumeOf(stock->shape());
    const int    fStock = stock->faceCount();

    auto in  = goodInput();
    auto out = skill::sprocket_with_chain_teeth::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    const double vSproc = volumeOf(out.workpiece->shape());
    EXPECT_LT(vSproc, vStock);

    // Expected: bore + N cylindrical pockets of roller radius
    const double bore_r = in.bore_dia_mm / 2.0;
    const double roller_r = 7.92 / 2.0;        // #40
    const int N = in.num_teeth;
    const double expected =
        M_PI * bore_r * bore_r * in.blank_thick_mm +
        N * M_PI * roller_r * roller_r * in.blank_thick_mm;
    const double delta = vStock - vSproc;
    EXPECT_GT(delta, expected * 0.8);
    EXPECT_LT(delta, expected * 1.2);

    // Face count must grow by at least N (each pocket adds 1+ cyl face).
    EXPECT_GT(out.workpiece->faceCount(), fStock + N);
}

// ─── 2. validate() rejects bad input ─────────────────────────────────────
TEST(SkillSprocketWithChainTeeth, ValidateRejectsBadInput)
{
    auto stock = skill::createCylindricalStock(100.0, 8.0);

    auto in = goodInput();
    in.ansi_chain_size = "999";        // unknown
    auto rep = skill::sprocket_with_chain_teeth::validate(*stock, in);
    EXPECT_FALSE(rep.passed);
    bool found = false;
    for (const auto& f : rep.findings) if (f.code == "DFM-CHAIN-SIZE") { found = true; break; }
    EXPECT_TRUE(found);
    EXPECT_THROW(skill::sprocket_with_chain_teeth::apply(*stock, in),
                 skill::SkillError);

    // Too few teeth
    in = goodInput();
    in.num_teeth = 5;
    rep = skill::sprocket_with_chain_teeth::validate(*stock, in);
    EXPECT_FALSE(rep.passed);
}

// ─── 3. signature is_compound + roundtrip ────────────────────────────────
TEST(SkillSprocketWithChainTeeth, SignatureIsCompound)
{
    auto stock = skill::createCylindricalStock(100.0, 8.0);
    auto in    = goodInput();
    auto out   = skill::sprocket_with_chain_teeth::apply(*stock, in);

    EXPECT_EQ(out.signature.skill_id, std::string("sprocket_with_chain_teeth"));
    EXPECT_TRUE(out.signature.pattern.at("is_compound").get<bool>());
    EXPECT_EQ(out.signature.pattern.at("subfeature_count").get<int>(), 1 + 20);
    EXPECT_EQ(out.signature.pattern.at("num_teeth").get<int>(), 20);
    EXPECT_EQ(out.signature.pattern.at("ansi_chain_size").get<std::string>(),
              std::string("40"));
}

// ─── 4. recognize() returns ≥1 candidate ─────────────────────────────────
TEST(SkillSprocketWithChainTeeth, RecognizeFindsCandidate)
{
    auto stock = skill::createCylindricalStock(100.0, 8.0);
    auto in    = goodInput();
    auto out   = skill::sprocket_with_chain_teeth::apply(*stock, in);

    auto cands = skill::sprocket_with_chain_teeth::recognize(*out.workpiece);
    ASSERT_FALSE(cands.empty());

    bool found_high_conf = false;
    for (const auto& c : cands) {
        if (c.confidence > 0.5) { found_high_conf = true; break; }
    }
    EXPECT_TRUE(found_high_conf);
}

// ─── 5. Feature-specific: PD = P / sin(180°/N) ANSI B29.1 ────────────────
TEST(SkillSprocketWithChainTeeth, PitchDiaMatchesANSIFormula)
{
    auto stock = skill::createCylindricalStock(100.0, 8.0);
    auto in    = goodInput();
    auto out   = skill::sprocket_with_chain_teeth::apply(*stock, in);

    const double P = 12.7;      // ANSI #40
    const double expectedPD = P / std::sin(M_PI / static_cast<double>(in.num_teeth));
    EXPECT_NEAR(out.signature.pattern.at("pitch_dia_mm").get<double>(),
                expectedPD, 1e-3);

    const double expectedOD = expectedPD + 7.92;     // PD + roller_dia
    EXPECT_NEAR(out.signature.pattern.at("outer_dia_mm").get<double>(),
                expectedOD, 1e-3);
}
