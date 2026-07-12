// @lat: [[process/test-strategy#skill round-trip]]
//
// annular_groove — a plain ring channel (watch bezel pocket / phone camera deco
// ring).  The geometry-recognisable counterpart to the metadata-only
// bezel_groove_assembly and the seal-specific o_ring_groove_face.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/annular_groove.hpp"
#include "skills/chamfer_edge.hpp"
#include "skills/drill_hole.hpp"
#include "skills/bore_with_shelf.hpp"
#include "skills/counterbore.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

#include <cmath>
#include <memory>

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

// ─── 1. apply cuts a ring channel of the right volume ──────────────────────
TEST(SkillAnnularGroove, ApplyRemovesRingVolume)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 10.0);
    const double vBefore = volumeOf(stock->shape());

    skill::annular_groove::Input in;
    in.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1), 5.0, "largest" };
    in.outer_dia_mm = 40.0;
    in.inner_dia_mm = 34.0;     // 3mm-wide ring
    in.depth_mm     = 2.0;

    const auto out = skill::annular_groove::apply(*stock, in);
    ASSERT_NE(out.workpiece, nullptr);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    const double removed = vBefore - volumeOf(out.workpiece->shape());
    const double expect  = M_PI * (20.0 * 20.0 - 17.0 * 17.0) * 2.0;
    EXPECT_NEAR(removed, expect, expect * 0.05)
        << "ring channel removes pi*(R^2 - r^2)*depth";
}

// ─── 2. recognize recovers a foreign bezel ring (geometric path B) ─────────
TEST(SkillAnnularGroove, RecognizesForeignBezel)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 10.0);
    skill::annular_groove::Input in;
    in.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1), 5.0, "largest" };
    in.outer_dia_mm = 40.0;
    in.inner_dia_mm = 34.0;
    in.depth_mm     = 2.0;
    const auto built = skill::annular_groove::apply(*stock, in);

    skill::Workpiece foreign(built.workpiece->shape());   // NO history
    const auto cands = skill::annular_groove::recognize(foreign);
    const skill::RecognizedFeature* g = nullptr;
    for (const auto& c : cands)
        if (c.matched_geometry.value("source", std::string{}) == "geometry") { g = &c; break; }
    ASSERT_NE(g, nullptr) << "a foreign bezel ring must be recognised geometrically";
    EXPECT_NEAR(g->recovered_params.value("outer_dia_mm", 0.0), 40.0, 0.5);
    EXPECT_NEAR(g->recovered_params.value("inner_dia_mm", 0.0), 34.0, 0.5);
    EXPECT_NEAR(g->recovered_params.value("depth_mm", 0.0), 2.0, 0.2);
}

// ─── 3. round-trip: recover -> regenerate matches volume ───────────────────
TEST(SkillAnnularGroove, RoundTripsByVolume)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 10.0);
    skill::annular_groove::Input in;
    in.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1), 5.0, "largest" };
    in.outer_dia_mm = 44.0;
    in.inner_dia_mm = 38.0;
    in.depth_mm     = 1.5;
    const auto built = skill::annular_groove::apply(*stock, in);
    const double vOrig = volumeOf(built.workpiece->shape());

    skill::Workpiece foreign(built.workpiece->shape());
    const auto cands = skill::annular_groove::recognize(foreign);
    const skill::RecognizedFeature* g = nullptr;
    for (const auto& c : cands)
        if (c.matched_geometry.value("source", std::string{}) == "geometry") { g = &c; break; }
    ASSERT_NE(g, nullptr);

    auto fresh = skill::createCuboidStock(60.0, 60.0, 10.0);
    skill::annular_groove::Input in2;
    in2.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1), 5.0, "largest" };
    in2.outer_dia_mm = g->recovered_params["outer_dia_mm"].get<double>();
    in2.inner_dia_mm = g->recovered_params["inner_dia_mm"].get<double>();
    in2.depth_mm     = g->recovered_params["depth_mm"].get<double>();
    const auto out2 = skill::annular_groove::apply(*fresh, in2);
    EXPECT_NEAR(volumeOf(out2.workpiece->shape()), vOrig, vOrig * 0.02)
        << "regenerated ring reproduces the original volume";
}

// ─── 3b. a TAPERED bezel (cone outer wall) recovers its taper angle and
// regenerates as a cone ring, not a straight wall. ───────────────────────────
TEST(SkillAnnularGroove, TaperedBezelRecoversTaperAngle)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 10.0);
    skill::annular_groove::Input in;
    in.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1), 5.0, "largest" };
    in.outer_dia_mm = 44.0;
    in.inner_dia_mm = 36.0;
    in.depth_mm     = 2.0;
    in.taper_deg    = 15.0;      // sloped outer wall (a real watch bezel)
    const auto built = skill::annular_groove::apply(*stock, in);
    const double vOrig = volumeOf(built.workpiece->shape());

    skill::Workpiece foreign(built.workpiece->shape());
    const auto cands = skill::annular_groove::recognize(foreign);
    const skill::RecognizedFeature* g = nullptr;
    for (const auto& c : cands)
        if (c.matched_geometry.value("source", std::string{}) == "geometry") { g = &c; break; }
    ASSERT_NE(g, nullptr) << "a tapered bezel must be recognised";
    // The taper angle must be recovered (not the old hard-coded 0).
    EXPECT_NEAR(g->recovered_params.value("taper_deg", 0.0), 15.0, 1.0)
        << "the cone outer wall's half-angle must be recovered";

    // Regenerate from the recovered params (incl. taper) → same volume.  Without
    // taper recovery this would rebuild a straight wall and mismatch the volume.
    auto fresh = skill::createCuboidStock(60.0, 60.0, 10.0);
    skill::annular_groove::Input in2;
    in2.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1), 5.0, "largest" };
    in2.outer_dia_mm = g->recovered_params["outer_dia_mm"].get<double>();
    in2.inner_dia_mm = g->recovered_params["inner_dia_mm"].get<double>();
    in2.depth_mm     = g->recovered_params["depth_mm"].get<double>();
    in2.taper_deg    = g->recovered_params.value("taper_deg", 0.0);
    const auto out2 = skill::annular_groove::apply(*fresh, in2);
    EXPECT_NEAR(volumeOf(out2.workpiece->shape()), vOrig, vOrig * 0.03)
        << "the tapered ring must regenerate to the same volume";
}

// ─── 4. a SHALLOW decorative ring (0.3mm) is recognised (o_ring rejects it) ─
TEST(SkillAnnularGroove, RecognizesShallowDecoRing)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    skill::annular_groove::Input in;
    in.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1), 5.0, "largest" };
    in.outer_dia_mm = 10.0;
    in.inner_dia_mm = 9.0;      // 0.5mm wide
    in.depth_mm     = 0.3;      // shallow deco ring
    const auto built = skill::annular_groove::apply(*stock, in);

    skill::Workpiece foreign(built.workpiece->shape());
    const auto cands = skill::annular_groove::recognize(foreign);
    bool found = false;
    for (const auto& c : cands)
        if (c.matched_geometry.value("source", std::string{}) == "geometry") found = true;
    EXPECT_TRUE(found) << "a 0.3mm decorative ring must still be recognised";
}

// ─── 5. NO false ring: a single drilled hole is not an annular groove. ──────
TEST(SkillAnnularGroove, NoFalseRingOnSingleHole)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    skill::drill_hole::Input h;
    h.position_x_mm = 0; h.position_y_mm = 0; h.diameter_mm = 8; h.depth_mm = 5;
    auto wp = skill::drill_hole::apply(*stock, h);
    skill::Workpiece foreign(wp.workpiece->shape());
    EXPECT_TRUE(skill::annular_groove::recognize(foreign).empty())
        << "a single cylindrical hole has no inner wall + annular floor";
}

// ─── 6. REAL WATCH BEZEL: outer wall coincides with the case exterior. ─────
//   A watch bezel is cut with outer_dia == the case's own diameter, so its
//   OUTER wall is the existing case side wall (a full-height cylinder), NOT a
//   fresh short groove wall.  Only the INNER wall + the annular floor are
//   distinct.  The recognizer must still recover the ring from the floor's
//   concentric edges, or a real bezel silently fails to round-trip.
TEST(SkillAnnularGroove, RecognizesWatchBezelOuterIsCaseWall)
{
    // Round case Ø44 × 10 thick.  Bezel: outer = 44 (= case OD), inner = 38, 1mm.
    auto stock = skill::createCylindricalStock(44.0, 10.0);
    skill::annular_groove::Input in;
    in.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1), 5.0, "largest" };
    in.outer_dia_mm = 44.0;     // == case outer diameter (wall coincides)
    in.inner_dia_mm = 38.0;
    in.depth_mm     = 1.0;
    const auto built = skill::annular_groove::apply(*stock, in);
    ASSERT_NE(built.workpiece, nullptr);

    skill::Workpiece foreign(built.workpiece->shape());   // NO history
    const auto cands = skill::annular_groove::recognize(foreign);
    const skill::RecognizedFeature* g = nullptr;
    for (const auto& c : cands)
        if (c.matched_geometry.value("source", std::string{}) == "geometry") { g = &c; break; }
    ASSERT_NE(g, nullptr)
        << "a real bezel (outer wall = case exterior) must still be recognised";
    EXPECT_NEAR(g->recovered_params.value("inner_dia_mm", 0.0), 38.0, 1.0);
    EXPECT_NEAR(g->recovered_params.value("outer_dia_mm", 0.0), 44.0, 1.0)
        << "outer dia recovered from the annular floor's outer edge";
    EXPECT_NEAR(g->recovered_params.value("depth_mm", 0.0), 1.0, 0.3);
}

// ─── 7. NO false ring on a STEPPED BORE (the coaxial-pair look-alike). ─────
//   bore_with_shelf has two coaxial cylinders + a shelf plane (an annular floor
//   look-alike), but its inner pilot continues PAST the shelf — the "solid
//   below the floor" gate must reject it even with close upper/lower depths.
TEST(SkillAnnularGroove, NoFalseRingOnSteppedBore)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 20.0);
    skill::bore_with_shelf::Input in;
    in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 30; in.position_y_mm = 30; in.axis_dir = gp_Dir(0, 0, -1);
    in.upper_dia_mm = 8;  in.upper_depth_mm = 5.0;
    in.lower_dia_mm = 16; in.lower_depth_mm = 5.5;   // close depths
    const auto built = skill::bore_with_shelf::apply(*stock, in);
    skill::Workpiece foreign(built.workpiece->shape());
    EXPECT_TRUE(skill::annular_groove::recognize(foreign).empty())
        << "a stepped bore (inner pilot continues past the shelf) is not a ring channel";
}

// ─── 8. NO false ring on a COUNTERBORE (pilot continues past the seat). ────
TEST(SkillAnnularGroove, NoFalseRingOnCounterbore)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 20.0);
    skill::counterbore::Input in;
    in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 30; in.position_y_mm = 30; in.axis_dir = gp_Dir(0, 0, -1);
    in.pilot_dia_mm = 6;  in.pilot_depth_mm = 5.0;
    in.seat_dia_mm  = 12; in.seat_depth_mm  = 4.8;   // close depths
    const auto built = skill::counterbore::apply(*stock, in);
    skill::Workpiece foreign(built.workpiece->shape());
    EXPECT_TRUE(skill::annular_groove::recognize(foreign).empty())
        << "a counterbore (pilot bore continues past the seat) is not a ring channel";
}

// ─── 9. DFM rejects inverted / zero geometry. ──────────────────────────────
TEST(SkillAnnularGroove, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    skill::annular_groove::Input in;
    in.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.outer_dia_mm = 10.0; in.inner_dia_mm = 10.0;   // zero width
    in.depth_mm     = 1.0;
    EXPECT_THROW(skill::annular_groove::apply(*stock, in), skill::SkillError);
    EXPECT_FALSE(skill::annular_groove::validate(*stock, in).passed);
}

// ─── 11. FLOOR-ANCHORED depth: a rim chamfer must not shorten the recovered
// groove depth.  The chamfer replaces the top band of the groove's inner wall
// with a cone, so a wall-extent reading came up short (0.7 for a nominal 1.0
// on the default watch) — the ring then under-cut on replay and a single-step
// transfer inherited a 30 % shallower groove.  depth = entry − floor. ───────
TEST(SkillAnnularGroove, RimChamferDoesNotShortenRecoveredDepth)
{
    auto stock = skill::createCylindricalStock(50.0, 10.0);   // Ø50 x 10 puck
    skill::annular_groove::Input in;
    in.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1), 5.0, "largest" };
    in.outer_dia_mm = 40.0;
    in.inner_dia_mm = 34.0;
    in.depth_mm     = 2.0;
    auto ringOut = skill::annular_groove::apply(*stock, in);
    ASSERT_NE(ringOut.workpiece, nullptr);

    // Chamfer the top edges (z = 10): the groove's rims lose their top band to
    // cones, exactly the geometry that made the wall-extent depth read short.
    skill::chamfer_edge::Input ch;
    ch.edge_selector   = skill::chamfer_edge::EdgesAtZBand{ 10.0, 1e-2 };
    ch.chamfer_size_mm = 0.6;
    auto chOut = skill::chamfer_edge::apply(*ringOut.workpiece, ch);
    ASSERT_NE(chOut.workpiece, nullptr);

    skill::Workpiece foreign(chOut.workpiece->shape());       // NO history
    const auto cands = skill::annular_groove::recognize(foreign);
    ASSERT_GE(cands.size(), 1u) << "the chamfered bezel ring must still be recognised";
    const auto& p = cands[0].recovered_params;
    EXPECT_NEAR(p.value("outer_dia_mm", 0.0), 40.0, 0.5);
    EXPECT_NEAR(p.value("inner_dia_mm", 0.0), 34.0, 0.5);
    EXPECT_NEAR(p.value("depth_mm", 0.0), 2.0, 0.15)
        << "depth must be FLOOR-anchored (entry − floor), not the chamfer-"
           "shortened wall extent (which would read ~1.4)";
}
