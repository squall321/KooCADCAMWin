// @lat: [[process/test-strategy#skill round-trip]]
//
// crown_knurl — a ring of N radial notches around a cone knob (watch crown grip).
// Recognised by N equal-radius cylinders whose axes are PERPENDICULAR to a
// cone-knob axis, evenly spaced.  The axial crown shaft and non-uniform side
// bores are rejected.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/crown_knurl.hpp"
#include "skills/drill_hole.hpp"

#include "engine/primitives/Tools.hpp"
#include "engine/primitives/Cuts.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>
#include <gp_Ax2.hxx>

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
const skill::RecognizedFeature* geomCand(const std::vector<skill::RecognizedFeature>& cands)
{
    for (const auto& c : cands)
        if (c.matched_geometry.value("source", std::string{}) == "geometry") return &c;
    return nullptr;
}
// Build a cone knob on a stock top and knurl it, returning the workpiece.
std::shared_ptr<skill::Workpiece> knurledKnob(int count, double knobR = 1.75,
                                              double notchR = 0.21)
{
    namespace pr = koocadcam::engine::prim;
    auto stock = skill::createCuboidStock(30.0, 30.0, 10.0);
    // A cone knob rising from the top face at the centre (axis +Z).
    const gp_Ax2 knobAx(gp_Pnt(15.0, 15.0, 10.0), gp_Dir(0, 0, 1));
    const TopoDS_Shape knob = pr::coneFrustum(knobAx, knobR, knobR * 0.85, 2.3);
    const TopoDS_Shape fused = pr::fuse(stock->shape(), knob);
    auto wp = std::make_shared<skill::Workpiece>(fused);

    skill::crown_knurl::Input in;
    in.world_cx_mm = 15.0; in.world_cy_mm = 15.0; in.world_cz_mm = 11.0;  // mid-knob
    in.world_ax = 0.0; in.world_ay = 0.0; in.world_az = 1.0;
    in.knob_radius_mm  = knobR;
    in.notch_radius_mm = notchR;
    in.count           = count;
    return skill::crown_knurl::apply(*wp, in).workpiece;
}
}  // namespace

// ─── 1. apply cuts N notches (removes material) ────────────────────────────
TEST(SkillCrownKnurl, ApplyCutsNotches)
{
    namespace pr = koocadcam::engine::prim;
    auto stock = skill::createCuboidStock(30.0, 30.0, 10.0);
    const gp_Ax2 knobAx(gp_Pnt(15, 15, 10), gp_Dir(0, 0, 1));
    const TopoDS_Shape fused = pr::fuse(stock->shape(),
        pr::coneFrustum(knobAx, 1.75, 1.75 * 0.85, 2.3));
    auto wp = std::make_shared<skill::Workpiece>(fused);
    const double vBefore = volumeOf(wp->shape());

    skill::crown_knurl::Input in;
    in.world_cx_mm = 15; in.world_cy_mm = 15; in.world_cz_mm = 11;
    in.world_az = 1.0; in.knob_radius_mm = 1.75; in.notch_radius_mm = 0.21; in.count = 8;
    const auto out = skill::crown_knurl::apply(*wp, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    EXPECT_LT(volumeOf(out.workpiece->shape()), vBefore) << "knurl must remove material";
    EXPECT_EQ(out.signature.pattern["instance_count"].get<int>(), 8);
}

// ─── 2. recognize recovers the notch count + spacing (KEY test) ────────────
TEST(SkillCrownKnurl, RecognizesForeignKnurl)
{
    auto knobWp = knurledKnob(8);
    skill::Workpiece foreign(knobWp->shape());   // NO history
    const auto cands = skill::crown_knurl::recognize(foreign);
    const skill::RecognizedFeature* g = geomCand(cands);
    ASSERT_NE(g, nullptr) << "a foreign crown knurl must be recognised";
    EXPECT_EQ(g->recovered_params.value("count", 0), 8) << "all 8 notches counted";
    EXPECT_NEAR(g->recovered_params.value("notch_radius_mm", 0.0), 0.21, 0.05);
    EXPECT_NEAR(g->recovered_params.value("knob_radius_mm", 0.0), 1.75, 0.3);
    // Axis must be ~+Z (the knob axis).
    const auto& ax = g->recovered_params["world_axis"];
    EXPECT_NEAR(std::abs(ax[2].get<double>()), 1.0, 1e-2);
}

// ─── 3. a different count (6) is recovered correctly ───────────────────────
TEST(SkillCrownKnurl, RecoversDifferentCount)
{
    auto knobWp = knurledKnob(6);
    skill::Workpiece foreign(knobWp->shape());
    const auto cands = skill::crown_knurl::recognize(foreign);
    const skill::RecognizedFeature* g = geomCand(cands);
    ASSERT_NE(g, nullptr);
    EXPECT_EQ(g->recovered_params.value("count", 0), 6);
}

// ─── 4. NO false on a bare cone knob with only an AXIAL shaft bore ──────────
// The crown SHAFT is a single cylinder whose axis is PARALLEL to the knob — it
// must NOT be mistaken for a radial notch ring.
TEST(SkillCrownKnurl, NoFalseOnAxialShaft)
{
    namespace pr = koocadcam::engine::prim;
    auto stock = skill::createCuboidStock(30.0, 30.0, 10.0);
    const gp_Ax2 knobAx(gp_Pnt(15, 15, 10), gp_Dir(0, 0, 1));
    TopoDS_Shape s = pr::fuse(stock->shape(),
        pr::coneFrustum(knobAx, 1.75, 1.75 * 0.85, 2.3));
    // Axial shaft bore straight down the knob axis.
    s = pr::cut(s, pr::cylinder(gp_Ax2(gp_Pnt(15, 15, 12.5), gp_Dir(0, 0, -1)), 0.75, 3.0));
    skill::Workpiece foreign(s);
    const auto cands = skill::crown_knurl::recognize(foreign);
    EXPECT_EQ(geomCand(cands), nullptr)
        << "a single axial shaft bore is not a radial knurl ring";
}

// ─── 5. NO false with fewer than 3 notches ─────────────────────────────────
// apply() rejects count<3, so cut 2 radial notches directly (bypassing apply) and
// confirm recognize does not call them a knurl.
TEST(SkillCrownKnurl, NoFalseBelowThree)
{
    namespace pr = koocadcam::engine::prim;
    auto stock = skill::createCuboidStock(30.0, 30.0, 10.0);
    const gp_Ax2 knobAx(gp_Pnt(15, 15, 10), gp_Dir(0, 0, 1));
    TopoDS_Shape s = pr::fuse(stock->shape(),
        pr::coneFrustum(knobAx, 1.75, 1.75 * 0.85, 2.3));
    // Only 2 radial notches (phi = 0 and pi) — below the >=3 ring floor.
    for (int i = 0; i < 2; ++i) {
        const double phi = M_PI * i;
        const gp_Pnt c(15 + std::cos(phi) * 1.75, 15 + std::sin(phi) * 1.75, 11.15);
        const gp_Dir inDir(-std::cos(phi), -std::sin(phi), 0);
        s = pr::cut(s, pr::cylinder(gp_Ax2(c, inDir), 0.21, 0.42));
    }
    skill::Workpiece foreign(s);
    const auto cands = skill::crown_knurl::recognize(foreign);
    EXPECT_EQ(geomCand(cands), nullptr) << "fewer than 3 notches is not a knurl";
}

// ─── 6. NO false on a plain cone knob (no notches) ─────────────────────────
TEST(SkillCrownKnurl, NoFalseOnPlainKnob)
{
    namespace pr = koocadcam::engine::prim;
    auto stock = skill::createCuboidStock(30.0, 30.0, 10.0);
    const gp_Ax2 knobAx(gp_Pnt(15, 15, 10), gp_Dir(0, 0, 1));
    const TopoDS_Shape s = pr::fuse(stock->shape(),
        pr::coneFrustum(knobAx, 1.75, 1.75 * 0.85, 2.3));
    skill::Workpiece foreign(s);
    const auto cands = skill::crown_knurl::recognize(foreign);
    EXPECT_EQ(geomCand(cands), nullptr) << "a plain cone knob has no knurl";
}

// ─── 6b. NO false on a cone + a DISTANT radial hole ring (near-knob gate) ──
// A cone knob plus an evenly-spaced radial hole ring FAR from the knob (not on
// its rim) must NOT be read as that knob's knurl — the ring radius must match
// the knob radius.
TEST(SkillCrownKnurl, NoFalseOnDistantRadialRing)
{
    namespace pr = koocadcam::engine::prim;
    auto stock = skill::createCuboidStock(60.0, 60.0, 20.0);
    // A small cone knob at the centre top (knob radius ~1.75).
    const gp_Ax2 knobAx(gp_Pnt(30, 30, 20), gp_Dir(0, 0, 1));
    TopoDS_Shape s = pr::fuse(stock->shape(),
        pr::coneFrustum(knobAx, 1.75, 1.75 * 0.85, 2.3));
    // A ring of radial holes at a LARGE radius (12mm) about the same +Z axis —
    // evenly spaced, same cyl radius, perpendicular axis — but NOT on the knob rim.
    for (int i = 0; i < 6; ++i) {
        const double phi = 2.0 * M_PI * i / 6;
        const gp_Pnt c(30 + std::cos(phi) * 12.0, 30 + std::sin(phi) * 12.0, 10.0);
        const gp_Dir inDir(-std::cos(phi), -std::sin(phi), 0);
        s = pr::cut(s, pr::cylinder(gp_Ax2(c, inDir), 1.0, 4.0));
    }
    skill::Workpiece foreign(s);
    const auto cands = skill::crown_knurl::recognize(foreign);
    EXPECT_EQ(geomCand(cands), nullptr)
        << "a radial hole ring far from the knob rim is not that knob's knurl";
}

// ─── ADV-A. Grub-screw collar: chamfer CONE + radial OD set-screw ring ──────
// A cylindrical clamping collar with a CHAMFERED top edge (a cone face whose
// gp_Cone::RefRadius == the boss radius) and >=3 evenly-spaced radial set-screw
// holes drilled at the OD (ringR ~= boss radius).  This is a textbook mechanical
// part.  It satisfies perpendicular-axis, even-spacing, count>=3, equal-radius,
// AND the near-knob radial gate (ringR ~= refR) — the finding predicts it FIRES.
TEST(SkillCrownKnurl, AdvGrubScrewCollarFalsePositive)
{
    namespace pr = koocadcam::engine::prim;
    const double R = 8.0;
    auto stock = skill::createCuboidStock(40.0, 40.0, 6.0);   // base plate, top z=6
    // Boss cylinder rising from the plate, radius R, height 12 (z 6..18).
    TopoDS_Shape s = pr::fuse(stock->shape(),
        pr::cylinder(gp_Ax2(gp_Pnt(20, 20, 6), gp_Dir(0, 0, 1)), R, 12.0));
    // Chamfered top edge as a short cone frustum whose RefRadius ~= R (the chamfer
    // cone) — this is the GeomAbs_Cone anchor the recognizer keys on.
    s = pr::fuse(s, pr::coneFrustum(gp_Ax2(gp_Pnt(20, 20, 18), gp_Dir(0, 0, 1)),
                                    R, R - 1.5, 1.5));   // refR ~= 8
    // 4 radial set-screw holes at the OD, 90deg apart, mid-boss (z=12).
    for (int i = 0; i < 4; ++i) {
        const double phi = 2.0 * M_PI * i / 4;
        const gp_Pnt c(20 + std::cos(phi) * R, 20 + std::sin(phi) * R, 12.0);
        const gp_Dir inDir(-std::cos(phi), -std::sin(phi), 0);
        s = pr::cut(s, pr::cylinder(gp_Ax2(c, inDir), 2.0, 4.0));
    }
    skill::Workpiece foreign(s);
    const auto cands = skill::crown_knurl::recognize(foreign);
    const skill::RecognizedFeature* g = geomCand(cands);
    if (g != nullptr) {
        ADD_FAILURE() << "FIRED: grub-screw collar (chamfer cone + radial OD "
                         "set-screws) misrecognised as crown_knurl; conf="
                      << g->confidence
                      << " count=" << g->recovered_params.value("count", 0)
                      << " knobR=" << g->recovered_params.value("knob_radius_mm", 0.0);
    }
}

// ─── ADV-B. Axially-DISTANT matching-radius ring (unbounded axis) ───────────
// The near-knob gate anchors the ring RADIUS to the cone's RefRadius but there is
// NO axial-containment gate: a radial hole ring at the MATCHING radius but far
// along the (infinite) cone axis from the frustum should still qualify.
TEST(SkillCrownKnurl, AdvAxiallyDistantMatchingRadiusRing)
{
    namespace pr = koocadcam::engine::prim;
    auto stock = skill::createCuboidStock(30.0, 30.0, 80.0);   // tall bar, top z=80
    const double R = 6.0;
    // Cone knob at the TOP (z=80), RefRadius ~= 6.
    TopoDS_Shape s = pr::fuse(stock->shape(),
        pr::coneFrustum(gp_Ax2(gp_Pnt(15, 15, 80), gp_Dir(0, 0, 1)), R, R * 0.85, 3.0));
    // 6 radial holes at the SAME radial distance (~6mm) but ~70mm BELOW the cone.
    for (int i = 0; i < 6; ++i) {
        const double phi = 2.0 * M_PI * i / 6;
        const gp_Pnt c(15 + std::cos(phi) * R, 15 + std::sin(phi) * R, 10.0);
        const gp_Dir inDir(-std::cos(phi), -std::sin(phi), 0);
        s = pr::cut(s, pr::cylinder(gp_Ax2(c, inDir), 0.6, 3.0));
    }
    skill::Workpiece foreign(s);
    const auto cands = skill::crown_knurl::recognize(foreign);
    const skill::RecognizedFeature* g = geomCand(cands);
    if (g != nullptr) {
        ADD_FAILURE() << "FIRED: a matching-radius radial ring 70mm from the cone "
                         "frustum was read as that cone's knurl (axis is unbounded); "
                         "conf=" << g->confidence
                      << " count=" << g->recovered_params.value("count", 0);
    }
}

// ─── 7. DFM rejects count < 3 ──────────────────────────────────────────────
TEST(SkillCrownKnurl, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(30.0, 30.0, 10.0);
    skill::crown_knurl::Input in;
    in.world_az = 1.0; in.knob_radius_mm = 1.75; in.notch_radius_mm = 0.21; in.count = 2;
    EXPECT_FALSE(skill::crown_knurl::validate(*stock, in).passed);
    EXPECT_THROW(skill::crown_knurl::apply(*stock, in), skill::SkillError);
}
