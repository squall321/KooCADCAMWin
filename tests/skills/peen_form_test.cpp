// @lat: [[process/test-strategy#skill round-trip]]
//
// peen_form — shot-peen forming.  Real geometric impl: rebuilds the plate
// as thinner-larger (volume preserving).

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/peen_form.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

#include <cmath>

using namespace koocadcam;

namespace {
double volumeOf(const TopoDS_Shape& s) {
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}
}  // namespace

// ── 1. Apply: thickness reduces, in-plane footprint grows, volume preserved ─
TEST(SkillPeenForm, ApplyRebuildsThinnerLargerPlate)
{
    auto stock = skill::createCuboidStock(200.0, 100.0, 2.0, "aluminum_7075");
    const double v0 = volumeOf(stock->shape());

    skill::peen_form::Input in;
    in.peen_face               = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.peening_intensity       = 0.20;       // Almen-A intensity
    in.target_curvature_mm_inv = 0.001;      // R = 1000 mm
    in.coverage_pct            = 200.0;

    auto out = skill::peen_form::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    // Volume preserved within ±5 % (the rebuild is volume-preserving by
    // construction, so this is much tighter in practice).
    const double v1 = volumeOf(out.workpiece->shape());
    EXPECT_NEAR(v1, v0, v0 * 0.05);

    // Thickness REDUCED: expected strain ε = 0.025 · 0.20 · 2.0 = 0.010
    // → t1 = 2.0 · (1 − 0.010) = 1.98 mm.
    double xMin, yMin, zMin, xMax, yMax, zMax;
    out.workpiece->boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double t1 = std::min({ xMax - xMin, yMax - yMin, zMax - zMin });
    const double expectedStrain = 0.025 * 0.20 * 2.0;     // 0.010
    const double expectedT1     = 2.0 * (1.0 - expectedStrain);
    EXPECT_NEAR(t1, expectedT1, 1e-6);
    EXPECT_LT(t1, 2.0);     // strictly reduced

    // Signature flags geometry_changed = true and records the strain.
    EXPECT_TRUE(out.signature.pattern["geometry_changed"].get<bool>());
    EXPECT_NEAR(out.signature.pattern["applied_strain"].get<double>(),
                expectedStrain, 1e-6);
    EXPECT_NEAR(out.signature.pattern["original_thickness_mm"].get<double>(),
                2.0, 1e-6);
    EXPECT_NEAR(out.signature.pattern["final_thickness_mm"].get<double>(),
                expectedT1, 1e-6);
}

// ── 2. DFM: bad input rejected (zero intensity, negative curvature) ─────────
TEST(SkillPeenForm, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(200.0, 100.0, 2.0);

    {
        skill::peen_form::Input in;
        in.peen_face               = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
        in.peening_intensity       = 0.0;
        in.target_curvature_mm_inv = 0.001;
        auto r = skill::peen_form::validate(*stock, in);
        EXPECT_FALSE(r.passed);
        EXPECT_THROW(skill::peen_form::apply(*stock, in), skill::SkillError);
    }
    {
        skill::peen_form::Input in;
        in.peen_face               = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
        in.peening_intensity       = 0.20;
        in.target_curvature_mm_inv = -0.001;
        auto r = skill::peen_form::validate(*stock, in);
        EXPECT_FALSE(r.passed);
    }
}

// ── 3. DFM-PEEN-CURV-MAX: curvature > 0.02 1/mm rejected (MIL-S-13165C) ────
TEST(SkillPeenForm, ValidateRejectsExtremeCurvature)
{
    auto stock = skill::createCuboidStock(200.0, 100.0, 2.0);

    skill::peen_form::Input in;
    in.peen_face               = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.peening_intensity       = 0.20;
    in.target_curvature_mm_inv = 0.05;       // R = 20 mm — exceeds capability
    in.coverage_pct            = 200.0;

    auto r = skill::peen_form::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-PEEN-CURV-MAX") { found = true; break; }
    EXPECT_TRUE(found);
    EXPECT_THROW(skill::peen_form::apply(*stock, in), skill::SkillError);
}

// ── 4. Recognize — metadata replay (1.0) + geometric fallback (0.40) ──────
TEST(SkillPeenForm, RecognizeMetadataAndGeometric)
{
    auto stock = skill::createCuboidStock(200.0, 100.0, 2.0);

    skill::peen_form::Input in;
    in.peen_face               = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.peening_intensity       = 0.15;
    in.target_curvature_mm_inv = 0.002;
    in.coverage_pct            = 180.0;

    auto out = skill::peen_form::apply(*stock, in);
    auto cands = skill::peen_form::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_NEAR(cands[0].recovered_params["peening_intensity"].get<double>(),
                0.15, 1e-9);

    // Strip metadata → geometric fallback at 0.40 confidence.
    skill::Workpiece raw(out.workpiece->shape());
    auto rawCands = skill::peen_form::recognize(raw);
    ASSERT_EQ(rawCands.size(), 1u);
    EXPECT_NEAR(rawCands[0].confidence, 0.40, 1e-6);
    EXPECT_EQ(rawCands[0].matched_geometry["source"].get<std::string>(),
              std::string("geometric_fallback"));
}

// ── 5. Tight curvature warning (R = 50–100 mm) still proceeds ──────────────
TEST(SkillPeenForm, TightCurvatureWarning)
{
    auto stock = skill::createCuboidStock(200.0, 100.0, 2.0);

    skill::peen_form::Input in;
    in.peen_face               = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.peening_intensity       = 0.20;
    in.target_curvature_mm_inv = 0.015;        // R = ~67 mm — tight but OK
    in.coverage_pct            = 200.0;

    auto r = skill::peen_form::validate(*stock, in);
    EXPECT_TRUE(r.passed)
        << "tight curvature should warn (not block) below 0.02 1/mm";
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-PEEN-CURV") { found = true; break; }
    EXPECT_TRUE(found);
    EXPECT_NO_THROW(skill::peen_form::apply(*stock, in));
}
