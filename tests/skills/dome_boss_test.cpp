// @lat: [[process/test-strategy#skill round-trip]]
//
// dome_boss — a freeform spherical-cap dome fused onto a flat face.  The
// apply()-side partner the dome recognizer needs (domeSolid was an engine
// primitive with no skill, so a recovered dome had nothing to regenerate into).
//
// Cases:
//   1. apply adds ~ spherical-cap volume on a flat plate.
//   2. recognize (foreign geometry, no history) recovers base_radius + height
//      from the BSpline cap faces.
//   3. round-trip: recover -> regenerate reproduces the dome volume.
//   4. metadata replay returns the exact params at full confidence.
//   5. DFM rejects non-positive radius / height.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/dome_boss.hpp"
#include "skills/extrude_boss_from_sketch.hpp"
#include "skills/revolve_boss.hpp"

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
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}
double capVol(double r, double h) { return M_PI * h * (3.0 * r * r + h * h) / 6.0; }
}  // namespace

// ─── 1. apply adds a spherical-cap volume ──────────────────────────────────
TEST(SkillDomeBoss, ApplyAddsCapVolume)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 10.0);
    const double vBefore = volumeOf(stock->shape());

    skill::dome_boss::Input in;
    in.entry_face     = skill::FaceByNormal{ gp_Dir(0, 0, 1), 5.0, "largest" };
    in.base_radius_mm = 12.0;
    in.height_mm      = 4.0;

    const auto out = skill::dome_boss::apply(*stock, in);
    ASSERT_NE(out.workpiece, nullptr);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    const double added = volumeOf(out.workpiece->shape()) - vBefore;
    const double expect = capVol(12.0, 4.0);
    // The loft approximates the spherical cap with a small flat apex; allow 12%.
    EXPECT_NEAR(added, expect, expect * 0.12)
        << "dome must add ~ a spherical cap of (r=12, h=4)";
}

// ─── 2. recognize recovers a foreign dome from its BSpline cap ──────────────
TEST(SkillDomeBoss, RecognizesForeignDome)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 10.0);
    skill::dome_boss::Input in;
    in.entry_face     = skill::FaceByNormal{ gp_Dir(0, 0, 1), 5.0, "largest" };
    in.base_radius_mm = 12.0;
    in.height_mm      = 4.0;
    const auto built = skill::dome_boss::apply(*stock, in);

    skill::Workpiece foreign(built.workpiece->shape());   // NO history
    const auto cands = skill::dome_boss::recognize(foreign);
    const skill::RecognizedFeature* g = nullptr;
    for (const auto& c : cands)
        if (c.matched_geometry.value("source", std::string{}) == "geometry") { g = &c; break; }
    ASSERT_NE(g, nullptr) << "a foreign dome must be recognised geometrically";
    EXPECT_NEAR(g->recovered_params.value("base_radius_mm", 0.0), 12.0, 1.5)
        << "base radius recovered";
    EXPECT_NEAR(g->recovered_params.value("height_mm", 0.0), 4.0, 1.0)
        << "rise recovered";
}

// ─── 3. round-trip: recover -> regenerate matches volume ───────────────────
TEST(SkillDomeBoss, RoundTripsByVolume)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 10.0);
    skill::dome_boss::Input in;
    in.entry_face     = skill::FaceByNormal{ gp_Dir(0, 0, 1), 5.0, "largest" };
    in.base_radius_mm = 10.0;
    in.height_mm      = 3.0;
    const auto built = skill::dome_boss::apply(*stock, in);
    const double vOrig = volumeOf(built.workpiece->shape());

    skill::Workpiece foreign(built.workpiece->shape());
    const auto cands = skill::dome_boss::recognize(foreign);
    const skill::RecognizedFeature* g = nullptr;
    for (const auto& c : cands)
        if (c.matched_geometry.value("source", std::string{}) == "geometry") { g = &c; break; }
    ASSERT_NE(g, nullptr);

    auto fresh = skill::createCuboidStock(60.0, 60.0, 10.0);
    skill::dome_boss::Input in2;
    in2.entry_face     = skill::FaceByNormal{ gp_Dir(0, 0, 1), 5.0, "largest" };
    in2.base_radius_mm = g->recovered_params["base_radius_mm"].get<double>();
    in2.height_mm      = g->recovered_params["height_mm"].get<double>();
    const auto out2 = skill::dome_boss::apply(*fresh, in2);
    ASSERT_NE(out2.workpiece, nullptr);
    EXPECT_NEAR(volumeOf(out2.workpiece->shape()), vOrig, vOrig * 0.05)
        << "regenerated dome reproduces the original volume";
}

// ─── 4. metadata replay returns exact params ───────────────────────────────
TEST(SkillDomeBoss, MetadataReplay)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 10.0);
    skill::dome_boss::Input in;
    in.entry_face     = skill::FaceByNormal{ gp_Dir(0, 0, 1), 5.0, "largest" };
    in.base_radius_mm = 9.0;
    in.height_mm      = 2.5;
    const auto out = skill::dome_boss::apply(*stock, in);

    const auto cands = skill::dome_boss::recognize(*out.workpiece);   // history present
    ASSERT_GE(cands.size(), 1u);
    EXPECT_EQ(cands[0].matched_geometry.value("source", std::string{}),
              std::string("metadata_replay"));
    EXPECT_NEAR(cands[0].recovered_params.value("base_radius_mm", 0.0), 9.0, 1e-6);
    EXPECT_NEAR(cands[0].recovered_params.value("height_mm", 0.0), 2.5, 1e-6);
}

// ─── 5. DFM rejects bad input ──────────────────────────────────────────────
TEST(SkillDomeBoss, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 10.0);
    skill::dome_boss::Input in;
    in.entry_face     = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.base_radius_mm = 0.0;     // invalid
    in.height_mm      = 4.0;
    EXPECT_THROW(skill::dome_boss::apply(*stock, in), skill::SkillError);
    EXPECT_FALSE(skill::dome_boss::validate(*stock, in).passed);
}

// ─── 6. NO false dome: a bare prism (no freeform cap) must not recognise. ───
TEST(SkillDomeBoss, NoFalseDomeOnPrism)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    skill::Workpiece foreign(stock->shape());
    EXPECT_TRUE(skill::dome_boss::recognize(foreign).empty())
        << "a plain box has no BSpline cap — never a dome";

    // An extruded polygonal boss is all planar — still no dome.
    skill::extrude_boss_from_sketch::Input boss;
    boss.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1), 5.0, "largest" };
    boss.polygon    = { { -8, -8 }, { 8, -8 }, { 8, 8 }, { -8, 8 } };
    boss.height_mm  = 5.0;
    auto wp = skill::extrude_boss_from_sketch::apply(*stock, boss);
    skill::Workpiece foreign2(wp.workpiece->shape());
    EXPECT_TRUE(skill::dome_boss::recognize(foreign2).empty())
        << "a prismatic boss has no freeform cap — never a dome";
}

// ─── 7. The SPHERE-RESIDUAL gate rejects a non-spherical freeform cap. ──────
// A revolved boss with a CONICAL meridian (a straight slanted profile) makes a
// surface of revolution that, when it surfaces as BSpline, is a CONE — not a
// sphere — so its points do NOT lie on a sphere and the residual gate rejects
// it.  (If it surfaces as GeomAbs_Cone/SurfaceOfRevolution it isn't even a
// BSpline face, so it's rejected earlier — either way, not a dome.)
TEST(SkillDomeBoss, SphereResidualRejectsCone)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 10.0);
    skill::revolve_boss::Input rv;
    // Cone-frustum meridian (r,z): wide base, narrow top, straight slant.
    rv.profile_polyline     = { { 0.0, 0.0 }, { 12.0, 0.0 }, { 4.0, 6.0 }, { 0.0, 6.0 } };
    rv.axis_origin          = gp_Pnt(0, 0, 10);   // on the top face
    rv.axis_dir             = gp_Dir(0, 0, 1);
    rv.revolution_angle_deg = 360.0;
    auto wp = skill::revolve_boss::apply(*stock, rv);
    ASSERT_NE(wp.workpiece, nullptr);

    skill::Workpiece foreign(wp.workpiece->shape());
    auto cands = skill::dome_boss::recognize(foreign);
    EXPECT_TRUE(cands.empty())
        << "a conical (non-spherical) revolution must not be recovered as a dome";
}
