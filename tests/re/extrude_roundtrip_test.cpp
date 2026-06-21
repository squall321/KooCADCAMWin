// @lat: [[process/test-strategy#skill round-trip]]
//
// Sketch-extrusion reverse-engineering round-trip — the first NON-machining
// feature the recognizer handles.  Most real CAD is sketch->extrude, not
// drill/pocket; the RE loop must recognise an extrusion and regenerate it from
// the recovered profile + height, exactly as it does for machining features.
//
// This proves the metadata-replay path (recognize -> regenerate).  Recovering
// the profile from FOREIGN geometry (no feature history) is a follow-up slice.

#include <gtest/gtest.h>

#include "re/Recognizer.hpp"
#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/extrude_boss_from_sketch.hpp"

#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepGProp.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <BRepPrimAPI_MakePrism.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>

#include <cmath>
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

TEST(ExtrudeRoundTrip, RecogniseThenRegenerateMatchesGeometry)
{
    // GENERATE: a 10x10 boss, 4 mm tall, on the top face of a 60x60x10 stock.
    auto stock = skill::createCuboidStock(60.0, 60.0, 10.0);
    ASSERT_FALSE(stock->shape().IsNull());

    skill::extrude_boss_from_sketch::Input in;
    in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.polygon    = { { -5.0, -5.0 }, { 5.0, -5.0 }, { 5.0, 5.0 }, { -5.0, 5.0 } };
    in.height_mm  = 4.0;

    const auto out = skill::extrude_boss_from_sketch::apply(*stock, in);
    ASSERT_NE(out.workpiece, nullptr);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    const double vOrig = volumeOf(out.workpiece->shape());

    // RECOGNISE: the extrusion is found at/above the 0.7 threshold (it was
    // previously capped to 0.5 by the "feature_history" source bug and dropped).
    const auto cands = re::analyzeFiltered(*out.workpiece, 0.7);
    const skill::RecognizedFeature* found = nullptr;
    for (const auto& c : cands)
        if (c.skill_id == "extrude_boss_from_sketch") { found = &c; break; }
    ASSERT_NE(found, nullptr)
        << "a generated extrusion must be recognised at >= 0.7 confidence";
    EXPECT_NEAR(found->recovered_params.value("height_mm", 0.0), 4.0, 1e-6);
    ASSERT_TRUE(found->recovered_params.contains("polygon"));
    EXPECT_EQ(found->recovered_params["polygon"].size(), 4u)
        << "the 4-vertex profile must round-trip";

    // REGENERATE from the recovered profile + height → identical geometry.
    skill::extrude_boss_from_sketch::Input in2;
    in2.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in2.height_mm  = found->recovered_params["height_mm"].get<double>();
    for (const auto& p : found->recovered_params["polygon"])
        in2.polygon.emplace_back(p["x"].get<double>(), p["y"].get<double>());

    const auto out2 = skill::extrude_boss_from_sketch::apply(*stock, in2);
    ASSERT_NE(out2.workpiece, nullptr);
    ASSERT_FALSE(out2.workpiece->shape().IsNull());
    EXPECT_NEAR(volumeOf(out2.workpiece->shape()), vOrig, 1e-3)
        << "recovered params must regenerate the same extrusion";
}

namespace {
// Shoelace area of a closed polygon given as {x,y} json points.
double polyArea(const nlohmann::json& poly)
{
    double a = 0.0;
    const std::size_t n = poly.size();
    for (std::size_t i = 0; i < n; ++i) {
        const auto& p = poly[i];
        const auto& q = poly[(i + 1) % n];
        a += p["x"].get<double>() * q["y"].get<double>()
           - q["x"].get<double>() * p["y"].get<double>();
    }
    return std::abs(a) * 0.5;
}
}  // namespace

// PATH B (foreign geometry, NO feature history): a standalone hexagonal prism
// is recognised as an extrusion by the congruent-cap-pair detector, recovering
// the 6-vertex profile and the height from geometry alone.
TEST(ExtrudeRoundTrip, RecognisesForeignPrismFromGeometry)
{
    const double R = 10.0, height = 5.0;
    std::vector<gp_Pnt> v;
    for (int k = 0; k < 6; ++k) {
        const double a = k * M_PI / 3.0;
        v.emplace_back(R * std::cos(a), R * std::sin(a), 0.0);
    }
    BRepBuilderAPI_MakeWire wireMk;
    for (int k = 0; k < 6; ++k)
        wireMk.Add(BRepBuilderAPI_MakeEdge(v[k], v[(k + 1) % 6]).Edge());
    const TopoDS_Face face = BRepBuilderAPI_MakeFace(wireMk.Wire(), true).Face();
    const TopoDS_Shape prism =
        BRepPrimAPI_MakePrism(face, gp_Vec(0, 0, height)).Shape();

    // A fresh Workpiece carries NO feature history — only the geometric path can fire.
    skill::Workpiece wp(prism);
    const auto cands = skill::extrude_boss_from_sketch::recognize(wp);

    const skill::RecognizedFeature* g = nullptr;
    for (const auto& c : cands)
        if (c.matched_geometry.value("source", std::string{}) == "geometry") { g = &c; break; }
    ASSERT_NE(g, nullptr) << "a foreign hex prism must be recognised geometrically";
    EXPECT_NEAR(g->recovered_params.value("height_mm", 0.0), height, 1e-3);
    ASSERT_TRUE(g->recovered_params.contains("polygon"));
    EXPECT_EQ(g->recovered_params["polygon"].size(), 6u) << "6-vertex profile recovered";
    // Regular hexagon area = (3*sqrt(3)/2) R^2 ≈ 259.8.
    const double expectArea = 1.5 * std::sqrt(3.0) * R * R;
    EXPECT_NEAR(polyArea(g->recovered_params["polygon"]), expectArea, expectArea * 0.02)
        << "recovered profile area must match the hexagon";
}

// ── ARC / CIRCLE profiles ─────────────────────────────────────────────────

// APPLY: a circular boss built from an Arc-only profile (one full circle, split
// into two semicircle arcs so the wire has two edges) extrudes to the right
// solid — its volume is the circle area * height, not a chord-polygon
// approximation.
TEST(ExtrudeArc, CircularBossApplyHasCircleVolume)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 10.0);
    const double r = 8.0, h = 5.0;

    using Seg = skill::extrude_boss_from_sketch::ProfileSeg;
    skill::extrude_boss_from_sketch::Input in;
    in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.height_mm  = h;
    // Two semicircle arcs sharing centre (0,0): (r,0)->(-r,0)->(r,0), CCW.
    Seg a; a.kind = Seg::Kind::Arc; a.x = -r; a.y = 0.0; a.cx = 0.0; a.cy = 0.0; a.ccw = true;
    Seg b; b.kind = Seg::Kind::Arc; b.x =  r; b.y = 0.0; b.cx = 0.0; b.cy = 0.0; b.ccw = true;
    in.profile = { a, b };

    const auto out = skill::extrude_boss_from_sketch::apply(*stock, in);
    ASSERT_NE(out.workpiece, nullptr);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    const double vStock = 60.0 * 60.0 * 10.0;
    const double vBoss  = M_PI * r * r * h;
    EXPECT_NEAR(volumeOf(out.workpiece->shape()), vStock + vBoss, vBoss * 0.01)
        << "the circular boss must add a true cylinder, not a polygon prism";
}

// PATH B (foreign geometry): a standalone cylinder is recognised as an
// extrusion whose profile is an ARC (the circular cap boundary), recovered as a
// circle — NOT flattened to a chord polygon.
TEST(ExtrudeArc, RecognisesForeignCylinderProfileAsArc)
{
    const double r = 9.0, h = 6.0;
    const TopoDS_Shape cyl =
        BRepPrimAPI_MakeCylinder(r, h).Shape();   // axis +Z, base at origin

    skill::Workpiece wp(cyl);
    const auto cands = skill::extrude_boss_from_sketch::recognize(wp);

    const skill::RecognizedFeature* g = nullptr;
    for (const auto& c : cands)
        if (c.matched_geometry.value("source", std::string{}) == "geometry") { g = &c; break; }
    ASSERT_NE(g, nullptr) << "a foreign cylinder must be recognised geometrically";
    EXPECT_NEAR(g->recovered_params.value("height_mm", 0.0), h, 1e-3);
    ASSERT_TRUE(g->recovered_params.contains("profile"))
        << "a curved boundary must be emitted as a segment profile";
    EXPECT_TRUE(g->matched_geometry.value("has_arc", false))
        << "the cylinder boundary must be recovered as an arc, not a chord";
    // At least one profile segment must be an arc.
    bool sawArc = false;
    for (const auto& s : g->recovered_params["profile"])
        if (s.value("kind", std::string("line")) == "arc") sawArc = true;
    EXPECT_TRUE(sawArc) << "the recovered profile must contain an arc segment";
}

// ROUND-TRIP: recognise the foreign cylinder, regenerate from the recovered
// ARC profile, and get the same volume back (the circle is preserved, not
// chord-approximated).
TEST(ExtrudeArc, ArcProfileRoundTripsByVolume)
{
    const double r = 7.0, h = 4.0;
    const TopoDS_Shape cyl = BRepPrimAPI_MakeCylinder(r, h).Shape();
    const double vOrig = volumeOf(cyl);

    skill::Workpiece wp(cyl);
    const auto cands = skill::extrude_boss_from_sketch::recognize(wp);
    const skill::RecognizedFeature* g = nullptr;
    for (const auto& c : cands)
        if (c.matched_geometry.value("source", std::string{}) == "geometry") { g = &c; break; }
    ASSERT_NE(g, nullptr);

    // Rebuild the Input from the recovered arc profile and extrude onto an empty
    // half-space (a thin stock the boss sits on), checking the boss volume.
    using Seg = skill::extrude_boss_from_sketch::ProfileSeg;
    auto stock = skill::createCuboidStock(40.0, 40.0, 2.0);
    skill::extrude_boss_from_sketch::Input in2;
    in2.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in2.height_mm  = g->recovered_params["height_mm"].get<double>();
    for (const auto& s : g->recovered_params["profile"]) {
        Seg seg;
        seg.kind = (s.value("kind", std::string("line")) == "arc")
                 ? Seg::Kind::Arc : Seg::Kind::Line;
        seg.x = s.value("x", 0.0); seg.y = s.value("y", 0.0);
        seg.cx = s.value("cx", 0.0); seg.cy = s.value("cy", 0.0);
        seg.ccw = s.value("ccw", true);
        in2.profile.push_back(seg);
    }
    const auto out2 = skill::extrude_boss_from_sketch::apply(*stock, in2);
    ASSERT_NE(out2.workpiece, nullptr);
    const double vStock = 40.0 * 40.0 * 2.0;
    EXPECT_NEAR(volumeOf(out2.workpiece->shape()) - vStock, vOrig, vOrig * 0.01)
        << "the regenerated boss must reproduce the cylinder volume via arcs";
}
