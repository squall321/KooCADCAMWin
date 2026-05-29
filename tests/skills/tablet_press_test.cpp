// @lat: [[process/test-strategy#skill round-trip]]
//
// tablet_press — REAL geometric tablet disc rebuild.
// Cases (5):
//   1. ApplyRebuildsAsDisc        — volume matches mass/density, faces == 3
//   2. ValidateRejectsBadInput
//   3. SignatureRecordsDiscParams
//   4. RecognizeGeometricHeuristic
//   5. HighForceEmitsCappingInfo

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/tablet_press.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

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

// Blend density 1.5 mg/mm³ (USP <1216> compendial midpoint).
constexpr double kRho = 1.5;

}  // namespace

// ─── 1. Apply rebuilds blank as a tablet disc ─────────────────────────────
//
// Expected disc volume = tablet_mass_mg / 1.5 mg/mm³.
// For 250 mg → V = 166.667 mm³ → t ≈ 2.371 mm, dia ≈ 9.484 mm
// (aspect d/t = 4 → V = π·(2t)²·t = 4π·t³ → t = (V/4π)^(1/3)).
// A right cylinder has exactly 3 topological faces (side + 2 caps).
TEST(SkillTabletPress, ApplyRebuildsAsDisc)
{
    auto stock = skill::createCuboidStock(20.0, 20.0, 4.0);
    const double volBefore = volumeOf(stock->shape());

    skill::tablet_press::Input in;
    in.press_force_kn  = 12.0;
    in.dwell_time_ms   = 60.0;
    in.tablet_mass_mg  = 250.0;

    auto out = skill::tablet_press::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    const double expectedV = in.tablet_mass_mg / kRho;          // 166.667 mm³
    EXPECT_NEAR(volumeOf(out.workpiece->shape()), expectedV, 1e-3);
    EXPECT_LT(volumeOf(out.workpiece->shape()), volBefore);     // disc < blank

    EXPECT_EQ(out.workpiece->faceCount(), 3);
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    // stock_removed_mm3 is negative (additive convention) and equals -V.
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, -expectedV, 1e-3);
}

// ─── 2. Validate rejects bad input ─────────────────────────────────────────
TEST(SkillTabletPress, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(20.0, 20.0, 3.0);

    skill::tablet_press::Input zeroF;
    zeroF.press_force_kn  = 0.0;
    zeroF.dwell_time_ms   = 50.0;
    zeroF.tablet_mass_mg  = 250.0;
    {
        auto r = skill::tablet_press::validate(*stock, zeroF);
        EXPECT_FALSE(r.passed);
        EXPECT_TRUE(hasFinding(r, "DFM-INPUT"));
    }
    EXPECT_THROW(skill::tablet_press::apply(*stock, zeroF), skill::SkillError);

    skill::tablet_press::Input zeroD;
    zeroD.press_force_kn  = 10.0;
    zeroD.dwell_time_ms   = -5.0;
    zeroD.tablet_mass_mg  = 250.0;
    {
        auto r = skill::tablet_press::validate(*stock, zeroD);
        EXPECT_FALSE(r.passed);
        EXPECT_TRUE(hasFinding(r, "DFM-INPUT"));
    }

    skill::tablet_press::Input zeroM;
    zeroM.press_force_kn  = 10.0;
    zeroM.dwell_time_ms   = 50.0;
    zeroM.tablet_mass_mg  = 0.0;
    {
        auto r = skill::tablet_press::validate(*stock, zeroM);
        EXPECT_FALSE(r.passed);
        EXPECT_TRUE(hasFinding(r, "DFM-INPUT"));
    }
}

// ─── 3. Signature records disc geometry params ─────────────────────────────
TEST(SkillTabletPress, SignatureRecordsDiscParams)
{
    auto stock = skill::createCuboidStock(20.0, 20.0, 5.0);

    skill::tablet_press::Input in;
    in.press_force_kn  = 18.5;
    in.dwell_time_ms   = 75.0;
    in.tablet_mass_mg  = 400.0;

    auto out = skill::tablet_press::apply(*stock, in);

    EXPECT_EQ(out.signature.skill_id, std::string("tablet_press"));
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("tablet_press"));
    EXPECT_NEAR(out.signature.pattern["tablet_mass_mg"].get<double>(),
                400.0, 1e-9);
    EXPECT_TRUE(out.signature.pattern["geometric_change"].get<bool>());

    // V = 400 / 1.5 = 266.67 mm³; t = (V/4π)^(1/3) ≈ 2.776 mm; dia ≈ 11.104 mm.
    const double expectedV = 400.0 / kRho;
    const double expectedT = std::cbrt(expectedV / (M_PI * 4.0));
    const double expectedD = 4.0 * expectedT;
    EXPECT_NEAR(out.signature.pattern["tablet_dia_mm"].get<double>(),
                expectedD, 1e-6);
    EXPECT_NEAR(out.signature.pattern["tablet_thick_mm"].get<double>(),
                expectedT, 1e-6);
    EXPECT_NEAR(out.signature.pattern["tablet_volume_mm3"].get<double>(),
                expectedV, 1e-6);
}

// ─── 4. Recognize geometric heuristic on a raw disc ────────────────────────
TEST(SkillTabletPress, RecognizeGeometricHeuristic)
{
    auto stock = skill::createCuboidStock(20.0, 20.0, 4.0);

    skill::tablet_press::Input in;
    in.press_force_kn  = 9.0;
    in.dwell_time_ms   = 40.0;
    in.tablet_mass_mg  = 180.0;

    auto out   = skill::tablet_press::apply(*stock, in);

    // Metadata replay: confidence 1.0.
    auto cands = skill::tablet_press::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_NEAR(cands[0].recovered_params["tablet_mass_mg"].get<double>(),
                180.0, 1e-9);

    // Stripped of metadata → geometric heuristic kicks in on the disc shape
    // at confidence in [0.4, 0.7].
    skill::Workpiece raw(out.workpiece->shape());
    auto cands2 = skill::tablet_press::recognize(raw);
    ASSERT_EQ(cands2.size(), 1u);
    EXPECT_GE(cands2[0].confidence, 0.4);
    EXPECT_LE(cands2[0].confidence, 0.7);
    // Recovered mass = volume × density ≈ original mass (within bbox noise).
    EXPECT_NEAR(cands2[0].recovered_params["tablet_mass_mg"].get<double>(),
                180.0, 1.0);
}

// ─── 5. High force emits capping info finding, still proceeds ──────────────
TEST(SkillTabletPress, HighForceEmitsCappingInfo)
{
    auto stock = skill::createCuboidStock(15.0, 15.0, 3.0);

    skill::tablet_press::Input in;
    in.press_force_kn  = 65.0;       // > 50 → info
    in.dwell_time_ms   = 50.0;
    in.tablet_mass_mg  = 300.0;

    auto r = skill::tablet_press::validate(*stock, in);
    EXPECT_TRUE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-TP-FORCE-HIGH"));

    auto out = skill::tablet_press::apply(*stock, in);
    EXPECT_EQ(out.signature.tooling.tool_type,
              std::string("tablet_press"));
    // Tool diameter is now the produced disc diameter.
    EXPECT_GT(out.signature.tooling.tool_dia_mm, 0.0);
}
