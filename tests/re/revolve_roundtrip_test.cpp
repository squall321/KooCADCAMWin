// @lat: [[process/test-strategy#skill round-trip]]
//
// Sketch-REVOLVE reverse-engineering round-trip — the second non-machining
// feature the recognizer handles (after extrude).  A profile revolved into a
// solid of revolution must recognise and regenerate from the recovered profile
// + axis + angle, like the extrusion does.
//
// This proves the metadata-replay path.  Geometric recovery of a revolution
// from foreign surfaces needs Workpiece surface-of-revolution detection — a
// follow-up slice.

#include <gtest/gtest.h>

#include "re/Recognizer.hpp"
#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/drill_hole.hpp"
#include "skills/revolve_boss.hpp"

#include "skills/chamfer_edge.hpp"
#include "skills/countersink.hpp"
#include "skills/fillet_edge.hpp"

#include <BRepBndLib.hxx>
#include <Bnd_Box.hxx>
#include <BRepGProp.hxx>
#include <BRepAlgoAPI_Fuse.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeCone.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>
#include <gp_Ax2.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>

#include <cmath>
#include <string>
#include <utility>
#include <vector>

using namespace koocadcam;

namespace {
double volumeOf(const TopoDS_Shape& s)
{
    GProp_GProps p;
    BRepGProp::VolumeProperties(s, p);
    return p.Mass();
}
}  // namespace

TEST(RevolveRoundTrip, RecogniseThenRegenerateMatchesGeometry)
{
    // GENERATE: a ring (4x2 section at r in [10,14], z in [0,2]) revolved 360°
    // about +Z, fused onto a stock.
    auto stock = skill::createCuboidStock(60.0, 60.0, 10.0);
    ASSERT_FALSE(stock->shape().IsNull());

    skill::revolve_boss::Input in;
    in.profile_polyline     = { { 10.0, 0.0 }, { 14.0, 0.0 }, { 14.0, 2.0 }, { 10.0, 2.0 } };
    in.axis_origin          = gp_Pnt(0.0, 0.0, 0.0);
    in.axis_dir             = gp_Dir(0.0, 0.0, 1.0);
    in.revolution_angle_deg = 360.0;

    const auto out = skill::revolve_boss::apply(*stock, in);
    ASSERT_NE(out.workpiece, nullptr);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    const double vOrig = volumeOf(out.workpiece->shape());

    // RECOGNISE: the revolve is found at/above the 0.7 threshold (it was
    // previously capped to 0.5 by the "feature_history" source bug, and the
    // recognizer was never even registered).
    const auto cands = re::analyzeFiltered(*out.workpiece, 0.7);
    const skill::RecognizedFeature* found = nullptr;
    for (const auto& c : cands)
        if (c.skill_id == "revolve_boss") { found = &c; break; }
    ASSERT_NE(found, nullptr)
        << "a generated revolve must be recognised at >= 0.7 confidence";
    EXPECT_NEAR(found->recovered_params.value("revolution_angle_deg", 0.0), 360.0, 1e-6);
    ASSERT_TRUE(found->recovered_params.contains("profile_polyline"));
    EXPECT_EQ(found->recovered_params["profile_polyline"].size(), 4u)
        << "the 4-vertex profile must round-trip";

    // REGENERATE from the recovered profile + axis + angle → identical geometry.
    skill::revolve_boss::Input in2;
    in2.revolution_angle_deg = found->recovered_params["revolution_angle_deg"].get<double>();
    for (const auto& p : found->recovered_params["profile_polyline"])
        in2.profile_polyline.emplace_back(p["r"].get<double>(), p["z"].get<double>());
    const auto ao = found->recovered_params["axis_origin"];
    const auto ad = found->recovered_params["axis_dir"];
    in2.axis_origin = gp_Pnt(ao[0].get<double>(), ao[1].get<double>(), ao[2].get<double>());
    in2.axis_dir    = gp_Dir(ad[0].get<double>(), ad[1].get<double>(), ad[2].get<double>());

    const auto out2 = skill::revolve_boss::apply(*stock, in2);
    ASSERT_NE(out2.workpiece, nullptr);
    ASSERT_FALSE(out2.workpiece->shape().IsNull());
    EXPECT_NEAR(volumeOf(out2.workpiece->shape()), vOrig, 1e-3)
        << "recovered params must regenerate the same revolution";
}

// LEGACY -Z PARITY: a ring revolved about -Z must produce the SAME solid as the
// same profile about +Z (a 360° revolve is axisymmetric).  Guards the arbitrary-
// axis refactor: the frame must place the profile at world (r,0,z) for BOTH ±Z
// axes — mapping a -Z axis to (r,0,-z) would mirror the solid across Z=0.  Since
// the struct/Executor default axis_dir is -Z, this is a common path.
TEST(RevolveRoundTrip, NegativeZAxisMatchesPositiveZSolid)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 10.0);
    skill::revolve_boss::Input base;
    base.profile_polyline     = { { 10.0, 1.0 }, { 14.0, 1.0 }, { 14.0, 4.0 }, { 10.0, 4.0 } };
    base.axis_origin          = gp_Pnt(0.0, 0.0, 0.0);
    base.revolution_angle_deg = 360.0;

    skill::revolve_boss::Input pz = base; pz.axis_dir = gp_Dir(0.0, 0.0,  1.0);
    skill::revolve_boss::Input nz = base; nz.axis_dir = gp_Dir(0.0, 0.0, -1.0);
    const auto op = skill::revolve_boss::apply(*stock, pz);
    const auto on = skill::revolve_boss::apply(*stock, nz);
    ASSERT_NE(op.workpiece, nullptr);
    ASSERT_NE(on.workpiece, nullptr);
    // Same fused volume AND same bounding box (not a mirror across Z=0).
    EXPECT_NEAR(volumeOf(on.workpiece->shape()), volumeOf(op.workpiece->shape()), 1e-3)
        << "a -Z revolve must equal the +Z revolve (axisymmetric), not mirror it";
    Bnd_Box bp, bn;
    BRepBndLib::Add(op.workpiece->shape(), bp);
    BRepBndLib::Add(on.workpiece->shape(), bn);
    double pxl, pyl, pzl, pxh, pyh, pzh, nxl, nyl, nzl, nxh, nyh, nzh;
    bp.Get(pxl, pyl, pzl, pxh, pyh, pzh);
    bn.Get(nxl, nyl, nzl, nxh, nyh, nzh);
    EXPECT_NEAR(nzl, pzl, 1e-6) << "ring z-min must match (mirror would flip it)";
    EXPECT_NEAR(nzh, pzh, 1e-6) << "ring z-max must match";
}

// GEOMETRIC path B: a foreign solid of revolution (a plain cylinder, no feature
// history) is detected by the coaxial-revolution-face grouping, recovering the
// axis + radial/axial envelope.
TEST(RevolveRoundTrip, RecognisesForeignCylinderAxisAndEnvelope)
{
    const double R = 5.0, H = 10.0;
    const TopoDS_Shape cyl =
        BRepPrimAPI_MakeCylinder(gp_Ax2(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1)), R, H).Shape();
    skill::Workpiece wp(cyl);

    const auto cands = skill::revolve_boss::recognize(wp);
    const skill::RecognizedFeature* g = nullptr;
    for (const auto& c : cands)
        if (c.matched_geometry.value("source", std::string{}) == "geometry") { g = &c; break; }
    ASSERT_NE(g, nullptr) << "a foreign solid of revolution must be detected";
    EXPECT_NEAR(g->recovered_params.value("max_radius_mm", 0.0), R, 1e-3);
    EXPECT_NEAR(g->recovered_params.value("axial_span_mm", 0.0), H, 1e-3);
    // Axis is +Z.
    ASSERT_TRUE(g->recovered_params.contains("axis_dir"));
    EXPECT_NEAR(std::abs(g->recovered_params["axis_dir"][2].get<double>()), 1.0, 1e-6);

    // MERIDIAN: a single solid cylinder recovers the rectangle meridian
    // {(0,0),(R,0),(R,H),(0,H)} which revolves back to the same cylinder.  Verify
    // the recovered (r,z) profile directly (revolving it is exact by Pappus:
    // 2*pi * r_centroid(R/2) * area(R*H) = pi*R^2*H).
    ASSERT_TRUE(g->recovered_params.contains("profile_polyline"));
    const auto& prof = g->recovered_params["profile_polyline"];
    ASSERT_EQ(prof.size(), 4u) << "a solid cylinder's meridian is a 4-point rectangle";

    double maxR = 0.0, zLo = 1e30, zHi = -1e30, area = 0.0;
    for (std::size_t i = 0; i < prof.size(); ++i) {
        const double r = prof[i]["r"].get<double>(), z = prof[i]["z"].get<double>();
        maxR = std::max(maxR, r);
        zLo = std::min(zLo, z); zHi = std::max(zHi, z);
        const auto& q = prof[(i + 1) % prof.size()];
        area += r * q["z"].get<double>() - q["r"].get<double>() * z;   // shoelace (r,z)
    }
    area = std::abs(area) * 0.5;
    EXPECT_NEAR(maxR, R, 1e-3)        << "meridian max radius == cylinder radius";
    EXPECT_NEAR(zHi - zLo, H, 1e-3)   << "meridian z-span == cylinder height";
    EXPECT_NEAR(area, R * H, R * H * 0.01)
        << "meridian area == R*H (revolves to pi*R^2*H by Pappus)";
}

// A CONE FRUSTUM solid of revolution (the watch crown knob) recovers its
// meridian as a trapezoid {(0,z0),(r0,z0),(r1,z1),(0,z1)}, read from the cone
// surface's two end radii, and revolves back to the same frustum volume.
TEST(RevolveRoundTrip, ConeFrustumRecoversTrapezoidMeridianAndRoundTrips)
{
    // r1=8 at z=0, r2=3 at z=10 (a downward-tapering frustum about +Z).
    const double R0 = 8.0, R1 = 3.0, H = 10.0;
    const TopoDS_Shape cone =
        BRepPrimAPI_MakeCone(gp_Ax2(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1)), R0, R1, H).Shape();
    const double vOrig = volumeOf(cone);   // frustum vol = pi*H/3*(R0^2+R0*R1+R1^2)

    skill::Workpiece wp(cone);
    const auto cands = skill::revolve_boss::recognize(wp);
    const skill::RecognizedFeature* g = nullptr;
    for (const auto& c : cands)
        if (c.matched_geometry.value("source", std::string{}) == "geometry") { g = &c; break; }
    ASSERT_NE(g, nullptr) << "a foreign cone frustum must be detected";

    // The meridian is now RECOVERED (a 4-point trapezoid) and surfaces at full
    // confidence — apply() can regenerate it.
    ASSERT_TRUE(g->recovered_params.contains("profile_polyline"));
    const auto& prof = g->recovered_params["profile_polyline"];
    ASSERT_EQ(prof.size(), 4u) << "a cone frustum's meridian is a 4-point trapezoid";
    EXPECT_GE(g->confidence, 0.7) << "a recoverable frustum must reach the Executor";

    // The two non-zero radii must match the cone's end radii (order-independent).
    double rA = 0.0, rB = 0.0;
    for (const auto& p : prof) {
        const double r = p["r"].get<double>();
        if (r > rA) { rB = rA; rA = r; } else if (r > rB) { rB = r; }
    }
    EXPECT_NEAR(std::max(rA, rB), std::max(R0, R1), 1e-2) << "larger end radius";
    EXPECT_NEAR(std::min(rA, rB), std::min(R0, R1), 1e-2) << "smaller end radius";

    // REGENERATE from the recovered profile + axis + angle → same frustum volume.
    skill::revolve_boss::Input in2;
    in2.revolution_angle_deg = g->recovered_params.value("revolution_angle_deg", 360.0);
    for (const auto& p : prof)
        in2.profile_polyline.emplace_back(p["r"].get<double>(), p["z"].get<double>());
    const auto ao = g->recovered_params["axis_origin"];
    const auto ad = g->recovered_params["axis_dir"];
    in2.axis_origin = gp_Pnt(ao[0].get<double>(), ao[1].get<double>(), ao[2].get<double>());
    in2.axis_dir    = gp_Dir(ad[0].get<double>(), ad[1].get<double>(), ad[2].get<double>());

    // REGENERATE on a base placed FAR from the frustum (the cone sits at z∈[0,10]
    // about +Z through the origin; a tiny base off to the side stays disjoint) so
    // the fused volume is exactly base + the regenerated frustum.  This actually
    // validates apply()'s regenerated SOLID (not just the recovered radii).
    auto stock = skill::createCuboidStock(2.0, 2.0, 2.0);
    const double vBase = volumeOf(stock->shape());          // box at x,y∈[0,2]
    // Shift the recovered axis well clear of that box so the revolved frustum
    // (radius ≤ 8 about the axis) cannot touch it.
    in2.axis_origin = gp_Pnt(50.0, 0.0, 0.0);
    in2.axis_dir    = gp_Dir(0.0, 0.0, 1.0);
    const auto out2 = skill::revolve_boss::apply(*stock, in2);
    ASSERT_NE(out2.workpiece, nullptr);
    ASSERT_FALSE(out2.workpiece->shape().IsNull());
    EXPECT_NEAR(volumeOf(out2.workpiece->shape()) - vBase, vOrig, vOrig * 0.02)
        << "the regenerated frustum SOLID must reproduce the original cone volume";
}

// RADIAL AXIS: a cone frustum about +X (the watch crown knob points sideways,
// not up) must recover its meridian AND its real axis, then regenerate in place.
TEST(RevolveRoundTrip, RadialAxisConeFrustumRecoversAxisAndRegenerates)
{
    const double R0 = 4.0, R1 = 3.4, H = 5.0;   // knob-like proportions, +X axis
    const gp_Ax2 ax(gp_Pnt(10.0, 0.0, 0.0), gp_Dir(1.0, 0.0, 0.0));
    const TopoDS_Shape cone = BRepPrimAPI_MakeCone(ax, R0, R1, H).Shape();
    const double vOrig = volumeOf(cone);

    skill::Workpiece wp(cone);
    const auto cands = skill::revolve_boss::recognize(wp);
    const skill::RecognizedFeature* g = nullptr;
    for (const auto& c : cands)
        if (c.matched_geometry.value("source", std::string{}) == "geometry") { g = &c; break; }
    ASSERT_NE(g, nullptr) << "a foreign radial cone frustum must be detected";
    ASSERT_TRUE(g->recovered_params.contains("profile_polyline"));
    ASSERT_EQ(g->recovered_params["profile_polyline"].size(), 4u);
    EXPECT_GE(g->confidence, 0.7);

    // The recovered axis must be ±X (not the legacy ±Z hard-code).
    const auto ad = g->recovered_params["axis_dir"];
    EXPECT_NEAR(std::abs(ad[0].get<double>()), 1.0, 1e-6) << "axis must be ±X";
    EXPECT_NEAR(ad[2].get<double>(), 0.0, 1e-6);

    // REGENERATE: revolve the recovered profile about the recovered ±X axis onto
    // a thin base far from the knob, so the fused volume = base + frustum.  The
    // frustum portion must match the original cone volume.
    skill::revolve_boss::Input in2;
    in2.revolution_angle_deg = g->recovered_params.value("revolution_angle_deg", 360.0);
    for (const auto& p : g->recovered_params["profile_polyline"])
        in2.profile_polyline.emplace_back(p["r"].get<double>(), p["z"].get<double>());
    const auto ao = g->recovered_params["axis_origin"];
    in2.axis_origin = gp_Pnt(ao[0].get<double>(), ao[1].get<double>(), ao[2].get<double>());
    in2.axis_dir    = gp_Dir(ad[0].get<double>(), ad[1].get<double>(), ad[2].get<double>());

    // A bare stock whose only solid is the revolved frustum (placed far from the
    // box so they don't interpenetrate): a thin plate at the origin, knob at x=10.
    auto base = skill::createCuboidStock(2.0, 2.0, 2.0);   // tiny, away from x∈[10,15]
    const double vBase = volumeOf(base->shape());
    const auto out2 = skill::revolve_boss::apply(*base, in2);
    ASSERT_NE(out2.workpiece, nullptr);
    ASSERT_FALSE(out2.workpiece->shape().IsNull());
    EXPECT_NEAR(volumeOf(out2.workpiece->shape()) - vBase, vOrig, vOrig * 0.02)
        << "radial cone frustum must regenerate to the same volume, in place";
}

// SPECIFICITY: a drilled block has a cylinder (the hole) but also walls PARALLEL
// to the axis, so it must NOT be mis-read as a solid of revolution.
TEST(RevolveRoundTrip, DrilledBlockIsNotARevolution)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 20.0);
    skill::drill_hole::Input in;
    in.position_x_mm = 20.0; in.position_y_mm = 20.0;
    in.diameter_mm = 8.0; in.depth_mm = 12.0;
    const auto out = skill::drill_hole::apply(*stock, in);
    ASSERT_NE(out.workpiece, nullptr);

    // Rebuild a fresh Workpiece (no feature history) so only the geometric path
    // can fire — the wall gate must reject it.
    skill::Workpiece wp(out.workpiece->shape());
    const auto cands = skill::revolve_boss::recognize(wp);
    for (const auto& c : cands)
        EXPECT_NE(c.matched_geometry.value("source", std::string{}), "geometry")
            << "a drilled block (axis-parallel walls) must not read as a revolution";
}

namespace {
// Does the geometric path fire at executor-reaching confidence with a meridian?
struct GeomFire { bool fired = false; double conf = 0.0; bool meridian = false; };
GeomFire probe(const TopoDS_Shape& s)
{
    skill::Workpiece wp(s);
    GeomFire g;
    for (const auto& c : skill::revolve_boss::recognize(wp))
        if (c.matched_geometry.value("source", std::string{}) == "geometry") {
            g.fired = true; g.conf = c.confidence;
            g.meridian = c.recovered_params.contains("profile_polyline") &&
                         !c.recovered_params["profile_polyline"].empty();
        }
    return g;
}
}  // namespace

// ADV-1: a block with a +Z countersink (cone) — side walls perpendicular to the
// cone axis must trip the wall gate.  MUST NOT reach the executor.
TEST(RevolveRoundTripAdv, BlockCountersinkIsNotARevolution)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 20.0);
    skill::countersink::Input in;
    in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 20; in.position_y_mm = 20; in.axis_dir = gp_Dir(0, 0, -1);
    in.pilot_dia_mm = 5; in.pilot_depth_mm = 10; in.cone_top_dia_mm = 10; in.cone_angle_deg = 90;
    const auto out = skill::countersink::apply(*stock, in);
    const auto g = probe(out.workpiece->shape());
    EXPECT_FALSE(g.meridian && g.conf >= 0.7)
        << "block+countersink fired revolve_boss at " << g.conf << " (over-cut)";
}

// ADV-2: a block with a chamfered through-hole (drill + chamfer = cone) — same.
TEST(RevolveRoundTripAdv, BlockChamferedHoleIsNotARevolution)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 20.0);
    skill::drill_hole::Input dh;
    dh.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    dh.position_x_mm = 20; dh.position_y_mm = 20;
    dh.axis_dir = gp_Dir(0, 0, -1); dh.diameter_mm = 8; dh.through_hole = true;
    auto drilled = skill::drill_hole::apply(*stock, dh).workpiece;
    skill::chamfer_edge::Input ch;
    ch.edge_selector = skill::fillet_edge::EdgesAtZBand{ 20.0, 1e-3 };
    ch.chamfer_size_mm = 1.5; ch.angle_deg = 45.0;
    const auto out = skill::chamfer_edge::apply(*drilled, ch);
    const auto g = probe(out.workpiece->shape());
    EXPECT_FALSE(g.meridian && g.conf >= 0.7)
        << "block+chamfered-hole fired revolve_boss at " << g.conf << " (over-cut)";
}

// ADV-3: a chamfered CYLINDER puck (one cone + one cylinder, all caps) — the
// cone branch requires cylCount==0 so it must stay sub-threshold (empty meridian).
TEST(RevolveRoundTripAdv, ChamferedCylinderPuckCapsOnly)
{
    const TopoDS_Shape cyl = BRepPrimAPI_MakeCylinder(
        gp_Ax2(gp_Pnt(0,0,0), gp_Dir(0,0,1)), 10.0, 20.0).Shape();
    skill::Workpiece w0(cyl);
    skill::chamfer_edge::Input ch;
    ch.edge_selector = skill::fillet_edge::EdgesAtZBand{ 20.0, 1e-3 };
    ch.chamfer_size_mm = 2.0; ch.angle_deg = 45.0;
    const auto out = skill::chamfer_edge::apply(w0, ch);
    const auto g = probe(out.workpiece->shape());
    EXPECT_FALSE(g.meridian && g.conf >= 0.7)
        << "chamfered cylinder puck fired full revolve_boss at " << g.conf;
}

// ADV-4: an isolated lone cone IS a solid of revolution — fires at 0.9 by design
// (documents the boundary: a boolean-isolated tapered solid replays as a revolve).
TEST(RevolveRoundTripAdv, LoneConeFiresByDesign)
{
    const TopoDS_Shape cone = BRepPrimAPI_MakeCone(
        gp_Ax2(gp_Pnt(0,0,0), gp_Dir(0,0,1)), 8.0, 3.0, 10.0).Shape();
    const auto g = probe(cone);
    std::printf("[ADV-4] lone cone: fired=%d conf=%.2f meridian=%d\n",
                g.fired, g.conf, g.meridian);
    EXPECT_TRUE(g.fired);
}

// ADV-5 (−Z cylinder recognition): a foreign cylinder about −Z must regenerate to
// the same solid as a +Z one — guards the frame z-sign fix through recognize→apply.
TEST(RevolveRoundTripAdv, NegativeZCylinderRecognisesAndRegenerates)
{
    const double R = 5.0, H = 8.0;
    const TopoDS_Shape cyl =
        BRepPrimAPI_MakeCylinder(gp_Ax2(gp_Pnt(0,0,0), gp_Dir(0,0,-1)), R, H).Shape();
    const double vOrig = volumeOf(cyl);
    skill::Workpiece wp(cyl);
    const auto cands = skill::revolve_boss::recognize(wp);   // bind: avoid dangling ref
    const skill::RecognizedFeature* g = nullptr;
    for (const auto& c : cands)
        if (c.matched_geometry.value("source", std::string{}) == "geometry") { g = &c; break; }
    ASSERT_NE(g, nullptr) << "a foreign -Z cylinder must be detected";
    ASSERT_TRUE(g->recovered_params.contains("profile_polyline") &&
                g->recovered_params["profile_polyline"].is_array());
    ASSERT_GE(g->recovered_params["profile_polyline"].size(), 3u);

    skill::revolve_boss::Input in;
    in.revolution_angle_deg = g->recovered_params.value("revolution_angle_deg", 360.0);
    for (const auto& p : g->recovered_params["profile_polyline"])
        in.profile_polyline.emplace_back(p["r"].get<double>(), p["z"].get<double>());
    auto base = skill::createCuboidStock(2.0, 2.0, 2.0);
    in.axis_origin = gp_Pnt(50.0, 0.0, 0.0);   // clear of the base box at [0,2]^3
    in.axis_dir    = gp_Dir(0.0, 0.0, 1.0);
    const double vBase = volumeOf(base->shape());
    const auto out = skill::revolve_boss::apply(*base, in);
    ASSERT_NE(out.workpiece, nullptr);
    EXPECT_NEAR(volumeOf(out.workpiece->shape()) - vBase, vOrig, vOrig * 0.02)
        << "the recovered -Z cylinder must regenerate the same volume (no mirror)";
}

// ADV-6 (TILTED-axis cone): a cone about (1,1,0) must recover its TRUE end radii
// and span — the old world-AABB projection over-stated the span for any tilt.
TEST(RevolveRoundTripAdv, TiltedAxisConeRecoversTrueRadii)
{
    const double R0 = 4.0, R1 = 3.4, H = 5.0;
    const TopoDS_Shape cone =
        BRepPrimAPI_MakeCone(gp_Ax2(gp_Pnt(0,0,0), gp_Dir(1,1,0)), R0, R1, H).Shape();
    skill::Workpiece wp(cone);
    const auto tc = skill::revolve_boss::recognize(wp);   // bind: avoid dangling ref
    const skill::RecognizedFeature* g = nullptr;
    for (const auto& c : tc)
        if (c.matched_geometry.value("source", std::string{}) == "geometry") { g = &c; break; }
    ASSERT_NE(g, nullptr) << "a foreign tilted cone must be detected";
    ASSERT_TRUE(g->recovered_params.contains("profile_polyline") &&
                g->recovered_params["profile_polyline"].is_array());
    const auto& prof = g->recovered_params["profile_polyline"];
    ASSERT_EQ(prof.size(), 4u);
    double rA = 0.0, rB = 0.0, zLo = 1e30, zHi = -1e30;
    for (const auto& p : prof) {
        const double r = p["r"].get<double>(), z = p["z"].get<double>();
        if (r > rA) { rB = rA; rA = r; } else if (r > rB) { rB = r; }
        zLo = std::min(zLo, z); zHi = std::max(zHi, z);
    }
    EXPECT_NEAR(std::max(rA, rB), std::max(R0, R1), 5e-2) << "wide radius (tilt-robust)";
    EXPECT_NEAR(std::min(rA, rB), std::min(R0, R1), 5e-2) << "narrow radius (tilt-robust)";
    EXPECT_NEAR(zHi - zLo, H, 1e-1)
        << "axial span must be the TRUE 5mm, not the inflated world-AABB span";
}

// ADV-7 (cone-protrusion no-false): a lone cone must NOT be claimed as a
// host-fused protrusion; a tapered knob fused to a plate MUST be, axis outward.
TEST(RevolveRoundTripAdv, ConeProtrusionOnlyFiresOnHostFusedCone)
{
    const TopoDS_Shape lone = BRepPrimAPI_MakeCone(
        gp_Ax2(gp_Pnt(0,0,0), gp_Dir(0,0,1)), 8.0, 3.0, 10.0).Shape();
    skill::Workpiece wlone(lone);
    const auto lc = skill::revolve_boss::recognize(wlone);
    for (const auto& c : lc)
        EXPECT_FALSE(c.matched_geometry.value("cone_protrusion", false))
            << "a free-standing cone must not be read as a host-fused protrusion";

    const TopoDS_Shape plate =
        BRepPrimAPI_MakeBox(gp_Pnt(-10,-10,-2), 20.0, 20.0, 2.0).Shape();   // top at z=0
    const TopoDS_Shape knob = BRepPrimAPI_MakeCone(
        gp_Ax2(gp_Pnt(0,0,-0.3), gp_Dir(0,0,1)), 3.0, 2.4, 2.3).Shape();    // rises past z=0
    const TopoDS_Shape fused = BRepAlgoAPI_Fuse(plate, knob).Shape();
    skill::Workpiece wknob(fused);
    const auto kc = skill::revolve_boss::recognize(wknob);
    const skill::RecognizedFeature* p = nullptr;
    for (const auto& c : kc)
        if (c.matched_geometry.value("cone_protrusion", false)) { p = &c; break; }
    ASSERT_NE(p, nullptr) << "a tapered knob fused to a plate must be a cone protrusion";
    ASSERT_TRUE(p->recovered_params.contains("axis_dir"));
    const auto& ad = p->recovered_params["axis_dir"];
    ASSERT_TRUE(ad.is_array() && ad.size() == 3u);
    EXPECT_GT(ad[2].get<double>(), 0.5)
        << "the protrusion axis must point OUTWARD (+Z, toward the free tip)";
}
