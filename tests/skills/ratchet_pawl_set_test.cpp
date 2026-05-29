// @lat: [[process/test-strategy#real compound feature]]
//
// ratchet_pawl_set — REAL compound feature: cylindrical blank → bore +
// N asymmetric tooth-gap cuts.  Pawl returned as metadata.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/ratchet_pawl_set.hpp"

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

skill::ratchet_pawl_set::Input goodInput()
{
    skill::ratchet_pawl_set::Input in;
    in.wheel_dia_mm        = 60.0;
    in.wheel_thick_mm      = 8.0;
    in.num_teeth           = 16;
    in.tooth_depth_mm      = 3.0;        // < 0.3 * 60
    in.steep_flank_deg     = 85.0;
    in.shallow_flank_deg   = 30.0;
    in.bore_dia_mm         = 10.0;
    in.ccw_locking         = true;
    return in;
}

}  // namespace

// ─── 1. apply() — REAL volume delta + face count growth ──────────────────
TEST(SkillRatchetPawlSet, ApplyRemovesRealGeometry)
{
    auto stock = skill::createCylindricalStock(60.0, 8.0);
    const double vStock = volumeOf(stock->shape());
    const int    fStock = stock->faceCount();

    auto in  = goodInput();
    auto out = skill::ratchet_pawl_set::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    const double vRatchet = volumeOf(out.workpiece->shape());
    EXPECT_LT(vRatchet, vStock);

    // Expected: bore + N tooth gaps.
    const double bore_r = in.bore_dia_mm / 2.0;
    const double rOut   = in.wheel_dia_mm / 2.0;
    const int N = in.num_teeth;
    const double tipArc  = 2.0 * M_PI / N * rOut;
    const double rootArc = std::max(0.0, tipArc - 0.5 * in.tooth_depth_mm);
    const double gapArea = 0.5 * (tipArc + rootArc) * in.tooth_depth_mm;
    const double expected =
        M_PI * bore_r * bore_r * in.wheel_thick_mm +
        N * gapArea * in.wheel_thick_mm;
    const double delta = vStock - vRatchet;
    EXPECT_GT(delta, expected * 0.4);
    EXPECT_LT(delta, expected * 2.0);

    EXPECT_GT(out.workpiece->faceCount(), fStock + N);
}

// ─── 2. validate() rejects bad input ─────────────────────────────────────
TEST(SkillRatchetPawlSet, ValidateRejectsBadInput)
{
    auto stock = skill::createCylindricalStock(60.0, 8.0);

    auto in = goodInput();
    in.steep_flank_deg = 50.0;       // < 70 violates DFM-RATCHET-STEEP
    auto rep = skill::ratchet_pawl_set::validate(*stock, in);
    EXPECT_FALSE(rep.passed);
    bool found = false;
    for (const auto& f : rep.findings) if (f.code == "DFM-RATCHET-STEEP") { found = true; break; }
    EXPECT_TRUE(found);
    EXPECT_THROW(skill::ratchet_pawl_set::apply(*stock, in),
                 skill::SkillError);

    // tooth depth too deep
    in = goodInput();
    in.tooth_depth_mm = 25.0;      // > 0.3 * 60
    rep = skill::ratchet_pawl_set::validate(*stock, in);
    EXPECT_FALSE(rep.passed);
}

// ─── 3. signature is_compound + asymmetric ───────────────────────────────
TEST(SkillRatchetPawlSet, SignatureIsCompound)
{
    auto stock = skill::createCylindricalStock(60.0, 8.0);
    auto in    = goodInput();
    auto out   = skill::ratchet_pawl_set::apply(*stock, in);

    EXPECT_EQ(out.signature.skill_id, std::string("ratchet_pawl_set"));
    EXPECT_TRUE(out.signature.pattern.at("is_compound").get<bool>());
    EXPECT_EQ(out.signature.pattern.at("subfeature_count").get<int>(),
              1 + 16);
    EXPECT_TRUE(out.signature.pattern.at("is_asymmetric").get<bool>());
    EXPECT_TRUE(out.signature.params.at("ccw_locking").get<bool>());
    EXPECT_TRUE(out.signature.tooling.extra.contains("pawl"));
}

// ─── 4. recognize() returns ≥1 candidate ─────────────────────────────────
TEST(SkillRatchetPawlSet, RecognizeFindsCandidate)
{
    auto stock = skill::createCylindricalStock(60.0, 8.0);
    auto in    = goodInput();
    auto out   = skill::ratchet_pawl_set::apply(*stock, in);

    auto cands = skill::ratchet_pawl_set::recognize(*out.workpiece);
    ASSERT_FALSE(cands.empty());

    bool found_replay = false;
    for (const auto& c : cands) {
        if (std::abs(c.confidence - 1.0) < 1e-9) { found_replay = true; break; }
    }
    EXPECT_TRUE(found_replay);
}

// ─── 5. Feature-specific: root_dia = wheel_dia - 2 * tooth_depth ─────────
TEST(SkillRatchetPawlSet, RootDiaMatchesGeometry)
{
    auto stock = skill::createCylindricalStock(60.0, 8.0);
    auto in    = goodInput();
    auto out   = skill::ratchet_pawl_set::apply(*stock, in);

    const double expectedRoot = in.wheel_dia_mm - 2.0 * in.tooth_depth_mm;
    EXPECT_NEAR(out.signature.pattern.at("root_dia_mm").get<double>(),
                expectedRoot, 1e-9);

    // Pawl metadata: tip_position should sit just outside wheel OD.
    const auto& pawl = out.signature.tooling.extra.at("pawl");
    EXPECT_GT(pawl.at("length_mm").get<double>(), 0.0);
    EXPECT_GT(pawl.at("width_mm").get<double>(), 0.0);
    const auto tipPos = pawl.at("tip_position");
    EXPECT_NEAR(tipPos[0].get<double>(), in.wheel_dia_mm / 2.0 + 1.0, 1e-6);
}
