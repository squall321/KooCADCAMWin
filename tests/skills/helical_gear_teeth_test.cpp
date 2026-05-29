// @lat: [[process/test-strategy#real compound feature]]
//
// helical_gear_teeth — REAL compound feature with N oblique-prism tooth
// cutters at helix_angle.  Tests verify volume delta, helical-projection,
// signature, recognize, and num-teeth derivation.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/helical_gear_teeth.hpp"

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

skill::helical_gear_teeth::Input goodInput()
{
    skill::helical_gear_teeth::Input in;
    in.blank_dia_mm        = 60.0;
    in.blank_thick_mm      = 20.0;       // > 2 * pi * m = 12.57
    in.normal_module_mm    = 2.0;
    in.pitch_dia_mm        = 48.0;       // N = 24
    in.helix_angle_deg     = 15.0;
    in.pressure_angle_deg  = 20.0;
    in.bore_dia_mm         = 14.0;
    in.right_hand          = true;
    return in;
}

}  // namespace

// ─── 1. apply() — REAL volume delta + face count growth ──────────────────
TEST(SkillHelicalGearTeeth, ApplyRemovesRealGeometry)
{
    auto stock = skill::createCylindricalStock(60.0, 20.0);
    const double vStock = volumeOf(stock->shape());
    const int    fStock = stock->faceCount();

    auto in  = goodInput();
    auto out = skill::helical_gear_teeth::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    const double vGear = volumeOf(out.workpiece->shape());
    EXPECT_LT(vGear, vStock);

    const int N = 24;
    const double bore_r = in.bore_dia_mm / 2.0;
    const double m = in.normal_module_mm;
    const double pitchR = in.pitch_dia_mm / 2.0;
    const double rRoot = pitchR - 1.25 * m;
    const double rOut  = pitchR + 1.00 * m;
    const double gulletWedgeVol = (rOut * rOut - rRoot * rRoot) * 0.5 *
                                  (2.0 * M_PI / N) * 0.6 * in.blank_thick_mm;
    const double expected =
        M_PI * bore_r * bore_r * in.blank_thick_mm +
        N * gulletWedgeVol;
    const double delta = vStock - vGear;
    EXPECT_GT(delta, expected * 0.5);
    EXPECT_LT(delta, expected * 2.0);

    EXPECT_GT(out.workpiece->faceCount(), fStock + N);
}

// ─── 2. validate() rejects bad input ─────────────────────────────────────
TEST(SkillHelicalGearTeeth, ValidateRejectsBadInput)
{
    auto stock = skill::createCylindricalStock(60.0, 20.0);

    auto in = goodInput();
    in.helix_angle_deg = 60.0;        // > 45 deg
    auto rep = skill::helical_gear_teeth::validate(*stock, in);
    EXPECT_FALSE(rep.passed);
    bool found = false;
    for (const auto& f : rep.findings) if (f.code == "DFM-GEAR-HELIX") { found = true; break; }
    EXPECT_TRUE(found);
    EXPECT_THROW(skill::helical_gear_teeth::apply(*stock, in),
                 skill::SkillError);
}

// ─── 3. signature is_compound + subfeature_count ─────────────────────────
TEST(SkillHelicalGearTeeth, SignatureIsCompound)
{
    auto stock = skill::createCylindricalStock(60.0, 20.0);
    auto in    = goodInput();
    auto out   = skill::helical_gear_teeth::apply(*stock, in);

    EXPECT_EQ(out.signature.skill_id, std::string("helical_gear_teeth"));
    EXPECT_TRUE(out.signature.pattern.at("is_compound").get<bool>());
    EXPECT_GE(out.signature.pattern.at("subfeature_count").get<int>(), 2);
    EXPECT_EQ(out.signature.pattern.at("num_teeth").get<int>(), 24);
    EXPECT_NEAR(out.signature.params.at("helix_angle_deg").get<double>(),
                15.0, 1e-9);
    EXPECT_TRUE(out.signature.params.at("right_hand").get<bool>());
}

// ─── 4. recognize() returns ≥1 candidate ─────────────────────────────────
TEST(SkillHelicalGearTeeth, RecognizeFindsCandidate)
{
    auto stock = skill::createCylindricalStock(60.0, 20.0);
    auto in    = goodInput();
    auto out   = skill::helical_gear_teeth::apply(*stock, in);

    auto cands = skill::helical_gear_teeth::recognize(*out.workpiece);
    ASSERT_FALSE(cands.empty());

    bool found_replay = false;
    for (const auto& c : cands) {
        if (std::abs(c.confidence - 1.0) < 1e-9) { found_replay = true; break; }
    }
    EXPECT_TRUE(found_replay);
}

// ─── 5. Feature-specific: AGMA OD = (N+2)*normal_module ──────────────────
TEST(SkillHelicalGearTeeth, OuterDiaMatchesAGMAFormula)
{
    auto stock = skill::createCylindricalStock(60.0, 20.0);
    auto in    = goodInput();
    auto out   = skill::helical_gear_teeth::apply(*stock, in);

    const int N = out.signature.pattern.at("num_teeth").get<int>();
    const double expectedOD =
        (static_cast<double>(N) + 2.0) * in.normal_module_mm;
    EXPECT_NEAR(out.signature.pattern.at("outer_dia_mm").get<double>(),
                expectedOD, 1e-9);
}
