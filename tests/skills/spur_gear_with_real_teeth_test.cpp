// @lat: [[process/test-strategy#real compound feature]]
//
// spur_gear_with_real_teeth — REAL multi-cut geometry: cylindrical blank →
// 1 bore + N gullet wedge cuts.  Tests verify volume delta, face-count
// growth, signature, recognize, and num-teeth derivation.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/spur_gear_with_real_teeth.hpp"

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

skill::spur_gear_with_real_teeth::Input goodInput()
{
    skill::spur_gear_with_real_teeth::Input in;
    in.blank_dia_mm        = 60.0;       // (N+2)*m = (24+2)*2 = 52 < 60 ✓
    in.blank_thick_mm      = 10.0;
    in.module_mm           = 2.0;
    in.pitch_dia_mm        = 48.0;       // N = 48 / 2 = 24 teeth
    in.pressure_angle_deg  = 20.0;
    in.bore_dia_mm         = 12.0;
    return in;
}

}  // namespace

// ─── 1. apply() — REAL volume delta + face count growth ──────────────────
TEST(SkillSpurGearWithRealTeeth, ApplyRemovesRealGeometry)
{
    auto stock = skill::createCylindricalStock(60.0, 10.0);
    const double vStock = volumeOf(stock->shape());
    const int    fStock = stock->faceCount();

    auto in  = goodInput();
    auto out = skill::spur_gear_with_real_teeth::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    const double vGear = volumeOf(out.workpiece->shape());
    EXPECT_LT(vGear, vStock) << "real geometry must be removed";

    // Expected delta = bore + N gullets.
    const int N = 24;
    const double bore_r = in.bore_dia_mm / 2.0;
    const double m = in.module_mm;
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

    // Face count must grow significantly (N gullets each leave multiple faces).
    EXPECT_GT(out.workpiece->faceCount(), fStock + N);
}

// ─── 2. validate() rejects bad input + apply throws ──────────────────────
TEST(SkillSpurGearWithRealTeeth, ValidateRejectsBadInput)
{
    auto stock = skill::createCylindricalStock(60.0, 10.0);

    // OD = (N+2)*m exceeds blank
    auto in = goodInput();
    in.blank_dia_mm = 30.0;        // OD = 52 > 30 ✗
    auto rep = skill::spur_gear_with_real_teeth::validate(*stock, in);
    EXPECT_FALSE(rep.passed);
    bool found = false;
    for (const auto& f : rep.findings) if (f.code == "DFM-GEAR-OD") { found = true; break; }
    EXPECT_TRUE(found);
    EXPECT_THROW(skill::spur_gear_with_real_teeth::apply(*stock, in),
                 skill::SkillError);

    // Bad pressure angle
    in = goodInput();
    in.pressure_angle_deg = 30.0;
    rep = skill::spur_gear_with_real_teeth::validate(*stock, in);
    EXPECT_FALSE(rep.passed);
}

// ─── 3. signature is_compound + subfeature_count + roundtrip ─────────────
TEST(SkillSpurGearWithRealTeeth, SignatureIsCompound)
{
    auto stock = skill::createCylindricalStock(60.0, 10.0);
    auto in    = goodInput();
    auto out   = skill::spur_gear_with_real_teeth::apply(*stock, in);

    EXPECT_EQ(out.signature.skill_id, std::string("spur_gear_with_real_teeth"));
    EXPECT_TRUE(out.signature.pattern.at("is_compound").get<bool>());
    EXPECT_GE(out.signature.pattern.at("subfeature_count").get<int>(), 2);
    EXPECT_EQ(out.signature.pattern.at("num_teeth").get<int>(), 24);
    EXPECT_NEAR(out.signature.params.at("module_mm").get<double>(), 2.0, 1e-9);
    EXPECT_NEAR(out.signature.params.at("pitch_dia_mm").get<double>(), 48.0, 1e-9);
}

// ─── 4. recognize() returns ≥1 candidate ─────────────────────────────────
TEST(SkillSpurGearWithRealTeeth, RecognizeFindsCandidate)
{
    auto stock = skill::createCylindricalStock(60.0, 10.0);
    auto in    = goodInput();
    auto out   = skill::spur_gear_with_real_teeth::apply(*stock, in);

    auto cands = skill::spur_gear_with_real_teeth::recognize(*out.workpiece);
    ASSERT_FALSE(cands.empty());

    bool found_high_conf = false;
    for (const auto& c : cands) {
        if (c.confidence > 0.5) { found_high_conf = true; break; }
    }
    EXPECT_TRUE(found_high_conf);

    // Metadata replay must have confidence 1.0
    bool found_replay = false;
    for (const auto& c : cands) {
        if (std::abs(c.confidence - 1.0) < 1e-9) { found_replay = true; break; }
    }
    EXPECT_TRUE(found_replay);
}

// ─── 5. Feature-specific: num_teeth = round(pitch_dia / module) ──────────
TEST(SkillSpurGearWithRealTeeth, ToothCountMatchesIsoFormula)
{
    auto stock = skill::createCylindricalStock(60.0, 10.0);
    auto in    = goodInput();
    auto out   = skill::spur_gear_with_real_teeth::apply(*stock, in);

    const int expectedN =
        static_cast<int>(std::round(in.pitch_dia_mm / in.module_mm));
    EXPECT_EQ(out.signature.pattern.at("num_teeth").get<int>(), expectedN);

    // Outer dia = (N+2) * module
    const double expectedOD =
        (static_cast<double>(expectedN) + 2.0) * in.module_mm;
    EXPECT_NEAR(out.signature.pattern.at("outer_dia_mm").get<double>(),
                expectedOD, 1e-9);
}
