// @lat: [[process/test-strategy#skill round-trip]]
//
// hemispherical_head_form — 5-case sweep for REAL-GEOMETRIC hemi-head
// forming.  Verifies apply() builds a hemispherical shell with correct
// volume, that recognize() can recover the diameter from the spherical
// face curvature, and that DFM gates on ASME B&PV thin-wall envelope.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/hemispherical_head_form.hpp"

#include <BRepAdaptor_Surface.hxx>
#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>
#include <gp_Sphere.hxx>

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

// ─── 1. ApplyBuildsHemisphericalShell — geometry CHANGES to a hemi ───────
//
// Hemi shell volume V = (2/3)·π·(R_o³ - R_i³).
// For D=600, t=12: R_o=300, R_i=288.
//   V_outer = (2/3)·π·300³  = 5.655e7 mm³ (full hemi)
//   V_inner = (2/3)·π·288³  = 4.997e7 mm³
//   V_wall  = 6.586e6 mm³
TEST(SkillHemiHeadForm, ApplyBuildsHemisphericalShell)
{
    auto stock = skill::createCylindricalStock(762.0, 12.0, "carbon_steel_1045");
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();

    skill::hemispherical_head_form::Input in;
    in.dia_mm         = 600.0;
    in.plate_thick_mm = 12.0;

    auto out = skill::hemispherical_head_form::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    const double volAfter = volumeOf(out.workpiece->shape());
    const double outerR   = in.dia_mm / 2.0;
    const double innerR   = outerR - in.plate_thick_mm;
    const double expectedHemiVol = (2.0 / 3.0) * M_PI *
        (outerR * outerR * outerR - innerR * innerR * innerR);

    // Hemispherical shell volume matches analytic formula within 0.5%.
    EXPECT_NEAR(volAfter, expectedHemiVol, expectedHemiVol * 0.005);
    EXPECT_NE(volAfter, volBefore);                           // geometry CHANGED
    EXPECT_NE(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    // Stock_removed_mm3 is 0 (forming, not cutting).
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. Validate rejects bad input and gates wall-vs-radius ──────────────
TEST(SkillHemiHeadForm, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(1000.0, 1000.0, 10.0);

    skill::hemispherical_head_form::Input bad;
    bad.dia_mm         = -50.0;      // invalid
    bad.plate_thick_mm = 12.0;
    auto r1 = skill::hemispherical_head_form::validate(*stock, bad);
    EXPECT_FALSE(r1.passed);
    EXPECT_THROW(skill::hemispherical_head_form::apply(*stock, bad),
                 skill::SkillError);

    skill::hemispherical_head_form::Input bad2;
    bad2.dia_mm         = 600.0;
    bad2.plate_thick_mm = 0.0;       // invalid
    auto r2 = skill::hemispherical_head_form::validate(*stock, bad2);
    EXPECT_FALSE(r2.passed);

    // Wall >= radius is impossible (cannot make hollow hemisphere).
    skill::hemispherical_head_form::Input bad3;
    bad3.dia_mm         = 100.0;
    bad3.plate_thick_mm = 60.0;     // > 50 (= R)
    auto r3 = skill::hemispherical_head_form::validate(*stock, bad3);
    EXPECT_FALSE(r3.passed);
    EXPECT_TRUE(hasFinding(r3, "DFM-HEAD-WALL-GE-R"));
}

// ─── 3. Signature records kind + geometric pattern ───────────────────────
TEST(SkillHemiHeadForm, SignatureRecordsKindAndGeometry)
{
    auto stock = skill::createCuboidStock(1000.0, 1000.0, 10.0);

    skill::hemispherical_head_form::Input in;
    in.dia_mm         = 800.0;
    in.plate_thick_mm = 16.0;

    auto out = skill::hemispherical_head_form::apply(*stock, in);

    EXPECT_EQ(out.signature.skill_id,
              std::string("hemispherical_head_form"));
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("hemispherical_head_form"));
    EXPECT_NEAR(out.signature.pattern["dia_mm"].get<double>(),         800.0, 1e-6);
    EXPECT_NEAR(out.signature.pattern["plate_thick_mm"].get<double>(),  16.0, 1e-6);
    EXPECT_NEAR(out.signature.pattern["outer_radius_mm"].get<double>(), 400.0, 1e-6);
    EXPECT_NEAR(out.signature.pattern["inner_radius_mm"].get<double>(), 384.0, 1e-6);
    EXPECT_TRUE(out.signature.pattern["geometry_changed"].get<bool>());
    EXPECT_GT(out.signature.pattern["hemi_volume_mm3"].get<double>(), 0.0);
    EXPECT_EQ(out.signature.tooling.tool_type,
              std::string("hemi_head_spin_mandrel"));
}

// ─── 4. Recognize via GEOMETRY (primary) — recovers D from sphere face ───
TEST(SkillHemiHeadForm, RecognizeGeometricallyFromSphericalFace)
{
    auto stock = skill::createCuboidStock(1000.0, 1000.0, 10.0);

    skill::hemispherical_head_form::Input in;
    in.dia_mm         = 500.0;
    in.plate_thick_mm = 10.0;

    auto out = skill::hemispherical_head_form::apply(*stock, in);

    // (a) Metadata replay — confidence 1.0.
    auto cands = skill::hemispherical_head_form::recognize(*out.workpiece);
    ASSERT_GE(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_NEAR(cands[0].recovered_params["dia_mm"].get<double>(),
                500.0, 1e-6);

    // (b) Strip features — geometric fallback recovers D from sphere radius.
    skill::Workpiece raw(out.workpiece->shape());
    auto geomCands = skill::hemispherical_head_form::recognize(raw);
    ASSERT_GE(geomCands.size(), 1u);
    EXPECT_NEAR(geomCands[0].confidence, 0.65, 1e-6);
    EXPECT_NEAR(geomCands[0].recovered_params["dia_mm"].get<double>(),
                500.0, 0.5);
    EXPECT_NEAR(geomCands[0].recovered_params["plate_thick_mm"].get<double>(),
                10.0, 0.2);
}

// ─── 5. Specific: spherical face curvature + thin-wall warning ────────────
// True hemisphere ⇒ at least one face is GeomAbs_Sphere of radius D/2.
// Verifies the thin-shell DFM warning at t/D = 0.0025 < 0.005.
TEST(SkillHemiHeadForm, SphericalFaceAndThinShellWarning)
{
    auto stock = skill::createCuboidStock(2000.0, 2000.0, 5.0);

    skill::hemispherical_head_form::Input in;
    in.dia_mm         = 2000.0;
    in.plate_thick_mm = 5.0;        // t/D = 0.0025 < 0.005

    auto out = skill::hemispherical_head_form::apply(*stock, in);
    EXPECT_NEAR(out.signature.pattern["crown_radius_mm"].get<double>(),
                1000.0, 1e-6);
    EXPECT_NEAR(out.signature.pattern["t_over_D"].get<double>(),
                0.0025, 1e-6);

    // The rebuilt shell must contain at least one spherical face whose
    // radius equals D/2 = 1000 mm.
    bool foundOuterSphere = false;
    for (int i = 0; i < out.workpiece->faceCount(); ++i) {
        BRepAdaptor_Surface s(out.workpiece->face(i));
        if (s.GetType() != GeomAbs_Sphere) continue;
        const gp_Sphere sph = s.Sphere();
        if (std::abs(sph.Radius() - 1000.0) < 0.1) {
            foundOuterSphere = true;
            break;
        }
    }
    EXPECT_TRUE(foundOuterSphere) << "outer spherical face Ø2000 not found";

    auto r = skill::hemispherical_head_form::validate(*stock, in);
    EXPECT_TRUE(r.passed);      // warning, not error
    EXPECT_TRUE(hasFinding(r, "DFM-HEAD-T-OVER-D"));
}
