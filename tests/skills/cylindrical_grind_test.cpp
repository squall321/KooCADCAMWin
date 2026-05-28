// @lat: [[process/test-strategy#skill round-trip]]
//
// cylindrical_grind — EXTERNAL precision grind on an outer cylindrical face.
// Reduces the OD radius by removal_mm.
//
// Tests (5):
//   1. apply() reduces the outer cylinder radius by removal_mm.
//   2. DFM rejects out-of-range removal_mm (below 0.005, above 0.1) AND a
//      removal larger than the existing radius.
//   3. apply() throws when datum doesn't resolve to a cylinder.
//   4. apply() carries grinding metadata + correct annulus stock-removed.
//   5. recognize() emits at most LOW-confidence candidates (metadata-only).

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/cylindrical_grind.hpp"

#include <BRepAdaptor_Surface.hxx>
#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>
#include <gp_Cylinder.hxx>

#include <algorithm>
#include <cmath>

using namespace koocadcam;

namespace {

double volumeOf(const TopoDS_Shape& s)
{
    GProp_GProps p;
    BRepGProp::VolumeProperties(s, p);
    return p.Mass();
}

// Largest cylindrical face on the workpiece — proxy for the OD.
double largestCylinderRadius(const skill::Workpiece& wp)
{
    double best = 0.0;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFaceCylinder(i)) continue;
        BRepAdaptor_Surface surf(wp.face(i));
        best = std::max(best, surf.Cylinder().Radius());
    }
    return best;
}

}  // namespace

// ─── 1. Apply: reduces OD radius by removal_mm ────────────────────────────
TEST(SkillCylindricalGrind, ApplyReducesOuterRadius)
{
    // Round shaft of dia 20 mm × length 50 mm.
    auto stock = skill::createCylindricalStock(20.0, 50.0);
    const double rBefore = largestCylinderRadius(*stock);
    EXPECT_NEAR(rBefore, 10.0, 1e-3);

    skill::cylindrical_grind::Input in;
    in.cylindrical_face_datum = skill::FaceCylinderByAxis{
        gp_Ax1(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1)) };
    in.removal_mm = 0.05;
    in.surface_finish = "ra_0.2";

    auto out = skill::cylindrical_grind::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    const double rAfter = largestCylinderRadius(*out.workpiece);
    EXPECT_NEAR(rAfter, rBefore - 0.05, 1e-3)
        << "cylindrical_grind should shrink OD radius by removal_mm";

    EXPECT_EQ(out.signature.skill_id, std::string("cylindrical_grind"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
}

// ─── 2. DFM rejects out-of-range removal_mm ───────────────────────────────
TEST(SkillCylindricalGrind, ValidateRejectsOutOfRange)
{
    auto stock = skill::createCylindricalStock(20.0, 50.0);

    // Too small (< 0.005)
    {
        skill::cylindrical_grind::Input in;
        in.cylindrical_face_datum = skill::FaceCylinderByAxis{
            gp_Ax1(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1)) };
        in.removal_mm = 0.001;
        auto r = skill::cylindrical_grind::validate(*stock, in);
        EXPECT_FALSE(r.passed);
        EXPECT_THROW(skill::cylindrical_grind::apply(*stock, in), skill::SkillError);
    }

    // Too large (> 0.1)
    {
        skill::cylindrical_grind::Input in;
        in.cylindrical_face_datum = skill::FaceCylinderByAxis{
            gp_Ax1(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1)) };
        in.removal_mm = 0.5;
        auto r = skill::cylindrical_grind::validate(*stock, in);
        EXPECT_FALSE(r.passed);
        EXPECT_THROW(skill::cylindrical_grind::apply(*stock, in), skill::SkillError);
    }

    // Removal larger than radius (impossible)
    {
        auto small = skill::createCylindricalStock(0.16, 50.0);  // r = 0.08 mm
        skill::cylindrical_grind::Input in;
        in.cylindrical_face_datum = skill::FaceCylinderByAxis{
            gp_Ax1(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1)) };
        in.removal_mm = 0.09;       // > r AND ≤ DFM upper bound 0.1
        auto r = skill::cylindrical_grind::validate(*small, in);
        EXPECT_FALSE(r.passed);
        bool foundRadius = false;
        for (const auto& f : r.findings)
            if (f.code == "DFM-CG-RADIUS") { foundRadius = true; break; }
        EXPECT_TRUE(foundRadius);
    }
}

// ─── 3. apply() throws when no cylindrical face is present ────────────────
TEST(SkillCylindricalGrind, ApplyThrowsOnNoCylinder)
{
    auto cuboid = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::cylindrical_grind::Input in;
    in.cylindrical_face_datum = skill::FaceCylinderByAxis{
        gp_Ax1(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1)) };
    in.removal_mm = 0.02;

    auto r = skill::cylindrical_grind::validate(*cuboid, in);
    EXPECT_FALSE(r.passed);
    EXPECT_THROW(skill::cylindrical_grind::apply(*cuboid, in), skill::SkillError);
}

// ─── 4. Apply carries grinding metadata + correct removed volume ──────────
TEST(SkillCylindricalGrind, ApplyCarriesGrindingMetadata)
{
    auto stock = skill::createCylindricalStock(20.0, 50.0);

    skill::cylindrical_grind::Input in;
    in.cylindrical_face_datum = skill::FaceCylinderByAxis{
        gp_Ax1(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1)) };
    in.removal_mm = 0.05;

    const double volBefore = volumeOf(stock->shape());
    auto out = skill::cylindrical_grind::apply(*stock, in);
    const double volAfter  = volumeOf(out.workpiece->shape());

    // Annulus volume: π × (R² − (R−Δ)²) × L
    const double R = 10.0;
    const double Rnew = 9.95;
    const double L = 50.0;
    const double approx = M_PI * (R * R - Rnew * Rnew) * L;
    EXPECT_NEAR(volBefore - volAfter, approx, approx * 0.10);

    EXPECT_EQ(out.signature.tooling.tool_type, std::string("grinding_wheel"));
    EXPECT_EQ(out.signature.tooling.flute_count, 0);
    EXPECT_GT(out.signature.tooling.cutting_speed_sfm, 3000.0);
    EXPECT_EQ(out.signature.tooling.extra["grind_type"].get<std::string>(),
              std::string("external_OD"));
    EXPECT_NEAR(out.signature.pattern["removal_mm"].get<double>(), 0.05, 1e-9);
}

// ─── 5. recognize() returns LOW-confidence candidates only ────────────────
TEST(SkillCylindricalGrind, RecognizeIsLowConfidence)
{
    // Stock with radius 10.000 mm — a "round" precision value.
    auto stock = skill::createCylindricalStock(20.0, 50.0);

    auto cands = skill::cylindrical_grind::recognize(*stock);
    // May or may not emit, but if it does, confidence must be low.
    for (const auto& c : cands) {
        EXPECT_LE(c.confidence, 0.50)
            << "cylindrical_grind recognize should not be high-confidence "
               "(looks like any turned cylinder)";
        EXPECT_EQ(c.skill_id, std::string("cylindrical_grind"));
    }
}
