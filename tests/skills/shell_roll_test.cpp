// @lat: [[process/test-strategy#skill round-trip]]
//
// shell_roll — 5-case sweep for REAL-GEOMETRIC plate-to-shell rolling.
// Verifies that apply() rebuilds the workpiece as a hollow cylindrical
// shell with correct outer/inner radii, volume DELTA matches the analytic
// expectation, and recognize() can recover the wall thickness from the
// resulting geometry.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/shell_roll.hpp"

#include <BRepAdaptor_Surface.hxx>
#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>
#include <gp_Cylinder.hxx>

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

}  // namespace

// ─── 1. ApplyRebuildsAsHollowCylinder — geometry CHANGES to a shell ──────
//
// Plate stock vol = π·D × L × t.  Shell wall vol = π·L × (R_o² - R_i²) =
//   π × L × (D·t - t²) = π × D × L × t × (1 - t/D).
// For D=600, L=3000, t=12 → plate vol = 67,858,400 mm³;
//   shell vol = π × 3000 × (300² - 288²) = π × 3000 × 7056 ≈ 66,500,000 mm³;
//   ratio ≈ 0.980.  Volume DELTA ≈ -1.36e6 mm³ (small negative because
//   the rolled shell has less material than the flat-plate developed
//   blank — wall thickness slightly displaces the neutral axis).
TEST(SkillShellRoll, ApplyRebuildsAsHollowCylinder)
{
    auto stock = skill::createCuboidStock(1885.0, 3000.0, 12.0, "carbon_steel_1045");
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();

    skill::shell_roll::Input in;
    in.plate_thick_mm  = 12.0;
    in.shell_dia_mm    = 600.0;
    in.shell_length_mm = 3000.0;

    auto out = skill::shell_roll::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    const double volAfter = volumeOf(out.workpiece->shape());
    const double outerR   = in.shell_dia_mm / 2.0;
    const double innerR   = outerR - in.plate_thick_mm;
    const double expectedShellVol = M_PI * in.shell_length_mm *
        ((outerR * outerR) - (innerR * innerR));

    // Shell volume matches analytic formula within 0.1%.
    EXPECT_NEAR(volAfter, expectedShellVol, expectedShellVol * 0.001);
    // The volume CHANGED vs the plate stock (geometry actually rebuilt).
    EXPECT_NE(volAfter, volBefore);
    // Hollow cylinder has 4 faces in OCCT (outer cyl, inner cyl, 2 end annuli).
    EXPECT_GE(out.workpiece->faceCount(), 4);
    EXPECT_NE(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->features().size(), 1u);
}

// ─── 2. Validate rejects bad input and thin-wall regime ──────────────────
TEST(SkillShellRoll, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(1000.0, 1000.0, 10.0);

    skill::shell_roll::Input bad;
    bad.plate_thick_mm  = 0.0;     // invalid
    bad.shell_dia_mm    = 600.0;
    bad.shell_length_mm = 3000.0;
    auto r1 = skill::shell_roll::validate(*stock, bad);
    EXPECT_FALSE(r1.passed);
    EXPECT_THROW(skill::shell_roll::apply(*stock, bad), skill::SkillError);

    skill::shell_roll::Input bad2;
    bad2.plate_thick_mm  = 10.0;
    bad2.shell_dia_mm    = -10.0;  // invalid
    bad2.shell_length_mm = 1000.0;
    auto r2 = skill::shell_roll::validate(*stock, bad2);
    EXPECT_FALSE(r2.passed);

    // Thin-wall regime: D/t = 600/8 = 75 ≥ 30 → warning.
    skill::shell_roll::Input thin;
    thin.plate_thick_mm  = 8.0;
    thin.shell_dia_mm    = 600.0;
    thin.shell_length_mm = 2000.0;
    auto r3 = skill::shell_roll::validate(*stock, thin);
    EXPECT_TRUE(r3.passed);  // warning, not error
    EXPECT_TRUE(hasFinding(r3, "DFM-ROLL-THIN-WALL"));
}

// ─── 3. Signature records kind + geometric params + stock_added_mm3 ──────
TEST(SkillShellRoll, SignatureRecordsKindAndShellGeometry)
{
    auto stock = skill::createCuboidStock(1000.0, 1000.0, 10.0);

    skill::shell_roll::Input in;
    in.plate_thick_mm  = 10.0;
    in.shell_dia_mm    = 800.0;
    in.shell_length_mm = 2400.0;

    auto out = skill::shell_roll::apply(*stock, in);

    EXPECT_EQ(out.signature.skill_id, std::string("shell_roll"));
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("shell_roll"));
    EXPECT_NEAR(out.signature.pattern["plate_thick_mm"].get<double>(),  10.0,   1e-6);
    EXPECT_NEAR(out.signature.pattern["shell_dia_mm"].get<double>(),    800.0,  1e-6);
    EXPECT_NEAR(out.signature.pattern["outer_radius_mm"].get<double>(), 400.0,  1e-6);
    EXPECT_NEAR(out.signature.pattern["inner_radius_mm"].get<double>(), 390.0,  1e-6);
    EXPECT_TRUE(out.signature.pattern["geometry_changed"].get<bool>());
    EXPECT_EQ(out.signature.tooling.tool_type,
              std::string("three_roll_plate_bender"));
    // shell_volume_mm3 in the pattern is non-zero.
    EXPECT_GT(out.signature.pattern["shell_volume_mm3"].get<double>(), 0.0);
}

// ─── 4. Recognize via GEOMETRY (primary) — recovers wall from cyl faces ──
TEST(SkillShellRoll, RecognizeGeometricallyFromHollowCylinder)
{
    auto stock = skill::createCuboidStock(1000.0, 1000.0, 10.0);

    skill::shell_roll::Input in;
    in.plate_thick_mm  = 8.0;
    in.shell_dia_mm    = 500.0;
    in.shell_length_mm = 3000.0;     // L/t = 375 >> 5 → long-shell heuristic OK.

    auto out = skill::shell_roll::apply(*stock, in);

    // (a) Metadata replay — confidence 1.0.
    auto cands = skill::shell_roll::recognize(*out.workpiece);
    ASSERT_GE(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_NEAR(cands[0].recovered_params["shell_dia_mm"].get<double>(),
                500.0, 1e-6);

    // (b) Strip features — pure GEOMETRIC recognition should still detect
    //     the hollow shell at confidence 0.6.
    skill::Workpiece raw(out.workpiece->shape());
    auto geomCands = skill::shell_roll::recognize(raw);
    ASSERT_GE(geomCands.size(), 1u);
    EXPECT_NEAR(geomCands[0].confidence, 0.60, 1e-6);
    EXPECT_NEAR(geomCands[0].recovered_params["plate_thick_mm"].get<double>(),
                8.0, 0.1);
    EXPECT_NEAR(geomCands[0].recovered_params["shell_dia_mm"].get<double>(),
                500.0, 0.1);
}

// ─── 5. Specific: outer cylindrical face matches D/2 and developed_length ─
// The rebuilt shell must have at least one Z-axis cylindrical face whose
// radius equals shell_dia_mm/2 (within OCCT tolerance).  Also verifies
// the signature's developed_length_mm = π × D.
TEST(SkillShellRoll, OuterCylinderFaceMatchesDiameter)
{
    auto stock = skill::createCuboidStock(1000.0, 1000.0, 10.0);

    skill::shell_roll::Input in;
    in.plate_thick_mm  = 12.0;
    in.shell_dia_mm    = 600.0;
    in.shell_length_mm = 3000.0;

    auto out = skill::shell_roll::apply(*stock, in);

    bool foundOuter = false;
    bool foundInner = false;
    for (int i = 0; i < out.workpiece->faceCount(); ++i) {
        if (!out.workpiece->isFaceCylinder(i)) continue;
        BRepAdaptor_Surface s(out.workpiece->face(i));
        const gp_Cylinder cyl = s.Cylinder();
        if (std::abs(std::abs(cyl.Axis().Direction().Z()) - 1.0) > 1e-3)
            continue;
        const double r = cyl.Radius();
        if (std::abs(r - 300.0) < 0.05) foundOuter = true;
        if (std::abs(r - 288.0) < 0.05) foundInner = true;
    }
    EXPECT_TRUE(foundOuter) << "outer Ø600 cylindrical face missing";
    EXPECT_TRUE(foundInner) << "inner Ø576 cylindrical face missing";

    const double expectedDevL = M_PI * 600.0;
    EXPECT_NEAR(out.signature.pattern["developed_length_mm"].get<double>(),
                expectedDevL, 1e-6);
    EXPECT_NEAR(out.signature.pattern["D_over_t"].get<double>(),
                600.0 / 12.0, 1e-6);
}
