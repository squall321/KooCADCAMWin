// @lat: [[process/test-strategy#skill round-trip]]
//
// box_pocket — a sharp-corner rectangular recess cut into a face (phone/watch
// side BUTTON, SIM tray, connector cutout).  The subtractive mirror of box_boss:
// a planar FLOOR bounded by 3-4 INWARD-facing walls, recessed below the entry
// plane.  Arbitrary axis (side faces), sharp corners — complements
// mill_rect_pocket (Z-only, keyed on corner fillets).

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/box_boss.hpp"
#include "skills/box_pocket.hpp"
#include "skills/mill_slot.hpp"

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
const skill::RecognizedFeature* geomCand(const std::vector<skill::RecognizedFeature>& cands)
{
    for (const auto& c : cands)
        if (c.matched_geometry.value("source", std::string{}) == "geometry") return &c;
    return nullptr;
}
}  // namespace

// ─── 0. PLACEMENT: an off-centre pocket must land at the requested centre, not
// be shifted (a box DY-sign error would offset it by the width). ─────────────
TEST(SkillBoxPocket, PlacementIsCentredNotShifted)
{
    // Two pockets on the +Z face, same size, at DIFFERENT y-centres.  Recognise
    // each and confirm the recovered mouth centre matches the requested centre
    // (a width-shift bug would put both floors at the wrong, size-dependent y).
    auto stock = skill::createCuboidStock(60.0, 60.0, 20.0);
    skill::box_pocket::Input in;
    in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1), 5.0, "largest" };
    in.length_mm = 12.0; in.width_mm = 8.0; in.depth_mm = 3.0;
    in.center_x_mm = 0.0; in.center_y_mm = 10.0;    // offset +10 in Y from face centre
    const auto built = skill::box_pocket::apply(*stock, in);

    // The face centre of a 60x60 top face is (30,30,20).  center_y=+10 → the
    // pocket centre is at world y = 40.  Recognise and check the recovered mouth.
    skill::Workpiece foreign(built.workpiece->shape());
    const auto cands = skill::box_pocket::recognize(foreign);
    const skill::RecognizedFeature* g = geomCand(cands);
    ASSERT_NE(g, nullptr);
    ASSERT_TRUE(g->recovered_params.contains("world_center"));
    const auto& wc = g->recovered_params["world_center"];
    // Mouth must be at (30, 40, 20) — the requested centre, NOT shifted by width.
    EXPECT_NEAR(wc[0].get<double>(), 30.0, 1.0) << "pocket X centred (face centre)";
    EXPECT_NEAR(wc[1].get<double>(), 40.0, 1.0) << "pocket Y at the requested +10 offset";
    EXPECT_NEAR(wc[2].get<double>(), 20.0, 1.0) << "mouth on the entry (top) plane";
}

// ─── 1. apply removes a box of the right volume ────────────────────────────
TEST(SkillBoxPocket, ApplyRemovesBoxVolume)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 20.0);
    const double vBefore = volumeOf(stock->shape());

    skill::box_pocket::Input in;
    in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1), 5.0, "largest" };
    in.length_mm = 12.0; in.width_mm = 8.0; in.depth_mm = 3.0;

    const auto out = skill::box_pocket::apply(*stock, in);
    ASSERT_NE(out.workpiece, nullptr);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    const double removed = vBefore - volumeOf(out.workpiece->shape());
    EXPECT_NEAR(removed, 12.0 * 8.0 * 3.0, 12.0 * 8.0 * 3.0 * 0.02)
        << "box pocket removes length*width*depth";
}

// ─── 2. recognize recovers a foreign SIDE pocket (±X axis) — the KEY test ───
// A phone/watch side button is a rect pocket on the ±X face; the recognizer must
// recover it about that radial axis, not a +Z one.
TEST(SkillBoxPocket, RecognizesForeignSidePocket)
{
    auto stock = skill::createCuboidStock(40.0, 60.0, 30.0);
    skill::box_pocket::Input in;
    in.entry_face = skill::FaceByNormal{ gp_Dir(1, 0, 0), 5.0, "largest" };  // +X side face
    in.length_mm = 10.0; in.width_mm = 5.0; in.depth_mm = 2.0;
    const auto built = skill::box_pocket::apply(*stock, in);

    skill::Workpiece foreign(built.workpiece->shape());   // NO history
    const auto cands = skill::box_pocket::recognize(foreign);
    const skill::RecognizedFeature* g = geomCand(cands);
    ASSERT_NE(g, nullptr) << "a foreign side pocket must be recognised geometrically";
    const double L = g->recovered_params.value("length_mm", 0.0);
    const double W = g->recovered_params.value("width_mm", 0.0);
    EXPECT_NEAR(std::max(L, W), 10.0, 0.5);
    EXPECT_NEAR(std::min(L, W), 5.0, 0.5);
    EXPECT_NEAR(g->recovered_params.value("depth_mm", 0.0), 2.0, 0.5);
    // The recovered axis must be ±X (the side face normal), not ±Z.
    ASSERT_TRUE(g->recovered_params.contains("face_normal"));
    const auto& n = g->recovered_params["face_normal"];
    EXPECT_NEAR(std::abs(n[0].get<double>()), 1.0, 1e-3) << "axis is ±X (side face)";
    EXPECT_NEAR(n[2].get<double>(), 0.0, 1e-3);
}

// ─── 3. round-trip by volume (side pocket, use_world replay) ────────────────
TEST(SkillBoxPocket, RoundTripsByVolume)
{
    auto stock = skill::createCuboidStock(40.0, 60.0, 30.0);
    skill::box_pocket::Input in;
    in.entry_face = skill::FaceByNormal{ gp_Dir(1, 0, 0), 5.0, "largest" };
    in.length_mm = 10.0; in.width_mm = 5.0; in.depth_mm = 2.0;
    const auto built = skill::box_pocket::apply(*stock, in);
    const double vBored = volumeOf(built.workpiece->shape());

    skill::Workpiece foreign(built.workpiece->shape());
    const auto cands = skill::box_pocket::recognize(foreign);   // bind: avoid dangling ref
    const skill::RecognizedFeature* g = geomCand(cands);
    ASSERT_NE(g, nullptr);
    ASSERT_TRUE(g->recovered_params.value("use_world", false));

    // Replay via the world path on fresh stock → same removed volume.
    auto fresh = skill::createCuboidStock(40.0, 60.0, 30.0);
    const double vFresh = volumeOf(fresh->shape());
    skill::box_pocket::Input in2;
    in2.use_world  = true;
    in2.length_mm  = g->recovered_params["length_mm"].get<double>();
    in2.width_mm   = g->recovered_params["width_mm"].get<double>();
    in2.depth_mm   = g->recovered_params["depth_mm"].get<double>();
    const auto& c = g->recovered_params["world_center"];
    in2.world_cx_mm = c[0].get<double>(); in2.world_cy_mm = c[1].get<double>(); in2.world_cz_mm = c[2].get<double>();
    const auto& n = g->recovered_params["face_normal"];
    in2.world_nx = n[0].get<double>(); in2.world_ny = n[1].get<double>(); in2.world_nz = n[2].get<double>();
    const auto& xa = g->recovered_params["face_xaxis"];
    in2.world_xx = xa[0].get<double>(); in2.world_xy = xa[1].get<double>(); in2.world_xz = xa[2].get<double>();
    const auto out2 = skill::box_pocket::apply(*fresh, in2);
    EXPECT_NEAR(vFresh - volumeOf(out2.workpiece->shape()), vFresh - vBored,
                (vFresh - vBored) * 0.03)
        << "the recovered side pocket regenerates the same removed volume in place";
}

// ─── 4. NO false on a box BOSS (outward walls) — convexity gate ─────────────
TEST(SkillBoxPocket, NoFalseOnBoss)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 10.0);
    skill::box_boss::Input in;
    in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1), 5.0, "largest" };
    in.length_mm = 12.0; in.width_mm = 8.0; in.height_mm = 3.0;
    const auto built = skill::box_boss::apply(*stock, in);

    skill::Workpiece foreign(built.workpiece->shape());
    const auto cands = skill::box_pocket::recognize(foreign);
    EXPECT_EQ(geomCand(cands), nullptr)
        << "a box boss (outward walls) must not read as a pocket";
}

// ─── 5. NO false on bare stock ─────────────────────────────────────────────
TEST(SkillBoxPocket, NoFalseOnBareStock)
{
    auto stock = skill::createCuboidStock(50.0, 40.0, 20.0);
    skill::Workpiece foreign(stock->shape());
    const auto cands = skill::box_pocket::recognize(foreign);
    EXPECT_EQ(geomCand(cands), nullptr) << "bare stock has no pocket";
}

// ─── 6. NO false on a through-slot (open both ends) ────────────────────────
TEST(SkillBoxPocket, NoFalseOnThroughSlot)
{
    auto stock = skill::createCuboidStock(60.0, 40.0, 20.0);
    skill::mill_slot::Input in;
    in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1), 5.0, "largest" };
    in.start_x_mm = 0.0;  in.start_y_mm = 20.0;    // runs off both ends (through)
    in.end_x_mm   = 60.0; in.end_y_mm   = 20.0;
    in.axis_dir = gp_Dir(0, 0, -1); in.width_mm = 8.0; in.depth_mm = 4.0;
    const auto built = skill::mill_slot::apply(*stock, in);

    skill::Workpiece foreign(built.workpiece->shape());
    const auto cands = skill::box_pocket::recognize(foreign);
    EXPECT_EQ(geomCand(cands), nullptr)
        << "an open through-slot is not a closed rectangular pocket";
}

// ─── 7. NO false on a thin high-aspect groove (footprint gate) ─────────────
TEST(SkillBoxPocket, NoFalseOnThinGroove)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 20.0);
    skill::box_pocket::Input in;   // build a valid recess, but a sliver aspect
    in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1), 5.0, "largest" };
    in.length_mm = 40.0; in.width_mm = 1.0; in.depth_mm = 3.0;   // aspect 40 > 12
    const auto built = skill::box_pocket::apply(*stock, in);

    skill::Workpiece foreign(built.workpiece->shape());
    const auto cands = skill::box_pocket::recognize(foreign);
    EXPECT_EQ(geomCand(cands), nullptr)
        << "a high-aspect groove must not read as a rectangular pocket";
}

// ─── 7b. NO false on a RABBET / open corner step: a full-width notch open on
// one side has a floor but only 2 side walls (the ends are open) — not a closed
// pocket.  Cut a box that runs off one edge of the stock. ────────────────────
TEST(SkillBoxPocket, NoFalseOnOpenRabbet)
{
    // A box_pocket cutter that spans the full Y width and runs off the +X edge
    // leaves an OPEN rabbet: a floor + 2 walls (the -X wall + the -Z floor's far
    // wall), open on +X and both Y ends.  The 3-4-wall gate must reject it.
    auto stock = skill::createCuboidStock(40.0, 30.0, 20.0);
    skill::box_pocket::Input in;
    in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1), 5.0, "largest" };
    in.length_mm = 60.0;             // longer than the 40mm X extent → runs off both X ends
    in.width_mm  = 12.0;
    in.depth_mm  = 3.0;
    const auto built = skill::box_pocket::apply(*stock, in);

    skill::Workpiece foreign(built.workpiece->shape());
    const auto cands = skill::box_pocket::recognize(foreign);
    EXPECT_EQ(geomCand(cands), nullptr)
        << "an open rabbet (floor + <3 closed walls) is not a rectangular pocket";
}

// ─── 8. OFF-AXIS L/W: a pocket in a part rotated 45° about Z recovers true
// dims from the floor-local frame, not the oversized world AABB. ────────────
TEST(SkillBoxPocket, OffAxisRecoversTrueDims)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 20.0);
    skill::box_pocket::Input in;
    in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1), 5.0, "largest" };
    in.length_mm = 12.0; in.width_mm = 8.0; in.depth_mm = 3.0;
    const auto built = skill::box_pocket::apply(*stock, in);

    // Rotate the whole part 45° about Z so the pocket edges are off the world axes.
    gp_Trsf rot; rot.SetRotation(gp_Ax1(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1)), M_PI / 4.0);
    const TopoDS_Shape rotated =
        BRepBuilderAPI_Transform(built.workpiece->shape(), rot, true).Shape();

    skill::Workpiece foreign(rotated);
    const auto cands = skill::box_pocket::recognize(foreign);   // bind: avoid dangling ref
    const skill::RecognizedFeature* g = geomCand(cands);
    ASSERT_NE(g, nullptr) << "a rotated pocket must still be recognised";
    const double L = g->recovered_params.value("length_mm", 0.0);
    const double W = g->recovered_params.value("width_mm", 0.0);
    EXPECT_NEAR(std::max(L, W), 12.0, 0.5) << "true length from floor-local frame";
    EXPECT_NEAR(std::min(L, W), 8.0, 0.5)  << "true width (not the 14.14 world AABB)";
}

// ─── 9. DFM rejects a zero dimension ───────────────────────────────────────
TEST(SkillBoxPocket, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 20.0);
    skill::box_pocket::Input in;
    in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.length_mm = 0.0; in.width_mm = 8.0; in.depth_mm = 3.0;   // zero length
    EXPECT_FALSE(skill::box_pocket::validate(*stock, in).passed);
    EXPECT_THROW(skill::box_pocket::apply(*stock, in), skill::SkillError);
}
