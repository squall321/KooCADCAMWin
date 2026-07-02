// @lat: [[process/test-strategy#skill round-trip]]
//
// box_boss — a raised rectangular pad (watch lug / phone camera island) fused
// onto a face.  The additive feature extrude_boss_from_sketch CANNOT recover
// (the boss's inner face merges into the host, so there is no congruent
// anti-parallel cap PAIR for the prism detector); box_boss keys on the outer cap
// + 4 perpendicular side walls instead.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/box_boss.hpp"
#include "skills/extrude_boss_from_sketch.hpp"
#include "skills/mill_rect_pocket.hpp"

#include <gp_Trsf.hxx>
#include <gp_Ax1.hxx>
#include <BRepBuilderAPI_Transform.hxx>

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
}  // namespace

// ─── 1. apply adds a box of the right volume ───────────────────────────────
TEST(SkillBoxBoss, ApplyAddsBoxVolume)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 10.0);
    const double vBefore = volumeOf(stock->shape());

    skill::box_boss::Input in;
    in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1), 5.0, "largest" };
    in.length_mm = 12.0; in.width_mm = 8.0; in.height_mm = 3.0;

    const auto out = skill::box_boss::apply(*stock, in);
    ASSERT_NE(out.workpiece, nullptr);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    const double added = volumeOf(out.workpiece->shape()) - vBefore;
    EXPECT_NEAR(added, 12.0 * 8.0 * 3.0, 12.0 * 8.0 * 3.0 * 0.02)
        << "box boss adds length*width*height";
}

// ─── 2. recognize recovers a foreign box boss (geometric) ──────────────────
TEST(SkillBoxBoss, RecognizesForeignBoxBoss)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 10.0);
    skill::box_boss::Input in;
    in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1), 5.0, "largest" };
    in.length_mm = 12.0; in.width_mm = 8.0; in.height_mm = 3.0;
    const auto built = skill::box_boss::apply(*stock, in);

    skill::Workpiece foreign(built.workpiece->shape());   // NO history
    const auto cands = skill::box_boss::recognize(foreign);
    const skill::RecognizedFeature* g = nullptr;
    for (const auto& c : cands)
        if (c.matched_geometry.value("source", std::string{}) == "geometry") { g = &c; break; }
    ASSERT_NE(g, nullptr) << "a foreign box boss must be recognised geometrically";
    // L/W recovered (order may swap; check the pair).
    const double L = g->recovered_params.value("length_mm", 0.0);
    const double W = g->recovered_params.value("width_mm", 0.0);
    EXPECT_NEAR(std::max(L, W), 12.0, 0.5);
    EXPECT_NEAR(std::min(L, W), 8.0, 0.5);
    EXPECT_NEAR(g->recovered_params.value("height_mm", 0.0), 3.0, 0.5);
    // The base-plane Z the CAM field is cleared to must be the host top face
    // (Z=10), NOT the boss cap (Z=13) and NOT Z=0.  cap.c is at the top; the base
    // is height inboard.
    ASSERT_TRUE(g->recovered_params.contains("center_z_world_mm"));
    EXPECT_NEAR(g->recovered_params["center_z_world_mm"].get<double>(), 10.0, 0.5)
        << "recovered boss base plane must be the host top (10), not the cap (13)";
}

// ─── 3. round-trip by volume ───────────────────────────────────────────────
TEST(SkillBoxBoss, RoundTripsByVolume)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 10.0);
    skill::box_boss::Input in;
    in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1), 5.0, "largest" };
    in.length_mm = 10.0; in.width_mm = 6.0; in.height_mm = 2.5;
    const auto built = skill::box_boss::apply(*stock, in);
    const double vOrig = volumeOf(built.workpiece->shape());

    skill::Workpiece foreign(built.workpiece->shape());
    const auto cands = skill::box_boss::recognize(foreign);
    const skill::RecognizedFeature* g = nullptr;
    for (const auto& c : cands)
        if (c.matched_geometry.value("source", std::string{}) == "geometry") { g = &c; break; }
    ASSERT_NE(g, nullptr);

    auto fresh = skill::createCuboidStock(60.0, 60.0, 10.0);
    skill::box_boss::Input in2;
    in2.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1), 5.0, "largest" };
    in2.length_mm  = g->recovered_params["length_mm"].get<double>();
    in2.width_mm   = g->recovered_params["width_mm"].get<double>();
    in2.height_mm  = g->recovered_params["height_mm"].get<double>();
    const auto out2 = skill::box_boss::apply(*fresh, in2);
    EXPECT_NEAR(volumeOf(out2.workpiece->shape()), vOrig, vOrig * 0.02)
        << "regenerated box boss reproduces the original volume";
}

// ─── 4. extrude_boss CANNOT recover a box boss (why box_boss exists) ────────
// Documents the gap: a fused box boss has only ONE free cap (the inner face
// merges into the host), so extrude_boss's congruent-cap-pair prism detector
// finds no pair and recovers nothing.  box_boss is the additive partner.
TEST(SkillBoxBoss, ExtrudeBossDoesNotRecoverBoxBoss)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 10.0);
    skill::box_boss::Input in;
    in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1), 5.0, "largest" };
    in.length_mm = 12.0; in.width_mm = 8.0; in.height_mm = 3.0;
    const auto built = skill::box_boss::apply(*stock, in);

    skill::Workpiece foreign(built.workpiece->shape());
    const auto ex = skill::extrude_boss_from_sketch::recognize(foreign);
    bool exGeom = false;
    for (const auto& c : ex)
        if (c.matched_geometry.value("source", std::string{}) == "geometry") exGeom = true;
    EXPECT_FALSE(exGeom)
        << "extrude_boss must NOT claim a fused box boss (no congruent cap pair)";
}

// ─── 5. NO false boss on a bare stock or a plain pocket. ───────────────────
TEST(SkillBoxBoss, NoFalseBossOnBareStock)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    skill::Workpiece foreign(stock->shape());
    EXPECT_TRUE(skill::box_boss::recognize(foreign).empty())
        << "a bare cuboid has no protruding box boss";
}

// ─── 7. NO false boss on a milled POCKET (a cut, not a protrusion). ────────
//   A rect pocket has a floor + 4 walls but the floor is INBOARD, so the
//   "cap at the outer boundary" gate must reject it.
TEST(SkillBoxBoss, NoFalseBossOnPocket)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 20.0);
    skill::mill_rect_pocket::Input mp;
    mp.entry_face  = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    mp.center_x_mm = 25.0; mp.center_y_mm = 25.0;
    mp.length_mm   = 16.0; mp.width_mm = 10.0; mp.depth_mm = 4.0;
    const auto built = skill::mill_rect_pocket::apply(*stock, mp);
    skill::Workpiece foreign(built.workpiece->shape());
    EXPECT_TRUE(skill::box_boss::recognize(foreign).empty())
        << "a milled pocket (recessed floor) must not read as a box boss";
}

// ─── 8. NO false boss on a thin RIB (extreme aspect → footprint gate). ─────
TEST(SkillBoxBoss, NoFalseBossOnThinRib)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 10.0);
    skill::box_boss::Input rib;
    rib.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1), 5.0, "largest" };
    rib.length_mm = 40.0; rib.width_mm = 1.5; rib.height_mm = 3.0;   // aspect 26.7
    const auto built = skill::box_boss::apply(*stock, rib);   // a real thin rib geometry
    skill::Workpiece foreign(built.workpiece->shape());
    EXPECT_TRUE(skill::box_boss::recognize(foreign).empty())
        << "a thin rib (aspect > 12) must not read as a box boss pad";
}

// ─── 9. OFF-AXIS (45deg) boss recovers true L/W (cap-local frame, not AABB)
// and round-trips at the right size/orientation. ────────────────────────────
TEST(SkillBoxBoss, OffAxisBossRecoversTrueDims)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 10.0);
    skill::box_boss::Input in;
    in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1), 5.0, "largest" };
    in.length_mm = 12.0; in.width_mm = 8.0; in.height_mm = 3.0;
    const auto built = skill::box_boss::apply(*stock, in);

    // Rotate the whole part 45deg about Z so the boss cap is off the world axes.
    gp_Trsf rot; rot.SetRotation(gp_Ax1(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1)), M_PI / 4.0);
    const TopoDS_Shape rotated =
        BRepBuilderAPI_Transform(built.workpiece->shape(), rot, true).Shape();

    skill::Workpiece foreign(rotated);
    const auto cands = skill::box_boss::recognize(foreign);
    const skill::RecognizedFeature* g = nullptr;
    for (const auto& c : cands)
        if (c.matched_geometry.value("source", std::string{}) == "geometry") { g = &c; break; }
    ASSERT_NE(g, nullptr) << "a 45deg-rotated box boss must still be recognised";
    const double L = g->recovered_params.value("length_mm", 0.0);
    const double W = g->recovered_params.value("width_mm", 0.0);
    // TRUE 12x8 — NOT the 14.14x14.14 a world AABB would give.
    EXPECT_NEAR(std::max(L, W), 12.0, 0.6) << "true length from cap-local frame";
    EXPECT_NEAR(std::min(L, W), 8.0, 0.6)  << "true width from cap-local frame";
}

// ─── 6. DFM rejects zero dims. ─────────────────────────────────────────────
TEST(SkillBoxBoss, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    skill::box_boss::Input in;
    in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.length_mm = 0.0; in.width_mm = 8.0; in.height_mm = 3.0;
    EXPECT_THROW(skill::box_boss::apply(*stock, in), skill::SkillError);
    EXPECT_FALSE(skill::box_boss::validate(*stock, in).passed);
}
