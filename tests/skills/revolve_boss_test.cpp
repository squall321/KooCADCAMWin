// @lat: [[process/test-strategy#skill round-trip]]
//
// revolve_boss — 2D profile revolved into a solid of revolution.
//
// Tests:
//   1. Apply produces a torus-like solid with the expected Pappus volume.
//   2. DFM rejects too-few profile vertices.
//   3. DFM rejects negative radius (profile crosses axis).
//   4. DFM rejects revolution_angle out of range.
//   5. Recognize finds axisymmetric cylindrical faces.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/revolve_boss.hpp"

#include <BRepAlgoAPI_Cut.hxx>
#include <BRepAlgoAPI_Fuse.hxx>
#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeCone.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <TopoDS_Shape.hxx>

#include <cmath>
#include <utility>
#include <vector>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using namespace koocadcam;

namespace {

double volumeOf(const TopoDS_Shape& s) {
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}

// Rectangle profile: r ∈ [10, 14], z ∈ [0, 2] → revolve 360° around Z
// makes a thin ring of cross-section 4 × 2 = 8 mm², centroid at r = 12.
// Expected volume (Pappus) = 2π · 12 · 8 = 192 π ≈ 603.19 mm³.
std::vector<std::pair<double,double>> ringProfile()
{
    return { {10.0, 0.0}, {14.0, 0.0}, {14.0, 2.0}, {10.0, 2.0} };
}

// Empty workpiece (start fresh — revolve directly).
std::shared_ptr<skill::Workpiece> emptyStock()
{
    BRepPrimAPI_MakeBox mk(gp_Pnt(-100.0, -100.0, -100.0), 1.0, 1.0, 1.0);
    return std::make_shared<skill::Workpiece>(mk.Shape());
}

}  // namespace

// ─── 1. Apply produces the right Pappus volume ────────────────────────────
TEST(SkillRevolveBoss, ApplyProducesRealVolume)
{
    auto stock = emptyStock();
    const double v0 = volumeOf(stock->shape());

    skill::revolve_boss::Input in;
    in.profile_polyline    = ringProfile();
    in.axis_origin         = gp_Pnt(0.0, 0.0, 0.0);
    in.axis_dir            = gp_Dir(0.0, 0.0, 1.0);
    in.revolution_angle_deg = 360.0;

    auto out = skill::revolve_boss::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    const double v1 = volumeOf(out.workpiece->shape());

    // Pappus volume = 2π · R_centroid · A
    // R_centroid = 12 (rectangle from r=10 to r=14)
    // A = 4 × 2 = 8 mm²
    // V = 2π × 12 × 8 = 192π ≈ 603.19 mm³
    const double expected = 2.0 * M_PI * 12.0 * 8.0;
    EXPECT_NEAR(v1 - v0, expected, expected * 0.05);
}

// ─── 2. DFM rejects too-few profile vertices ──────────────────────────────
TEST(SkillRevolveBoss, ValidateRejectsTooFewVerts)
{
    auto stock = emptyStock();

    skill::revolve_boss::Input in;
    in.profile_polyline = { {10.0, 0.0}, {12.0, 1.0} };
    in.revolution_angle_deg = 360.0;

    auto r = skill::revolve_boss::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_THROW(skill::revolve_boss::apply(*stock, in),
                 skill::SkillError);
}

// ─── 3. DFM rejects negative radius ───────────────────────────────────────
TEST(SkillRevolveBoss, ValidateRejectsNegativeRadius)
{
    auto stock = emptyStock();

    skill::revolve_boss::Input in;
    in.profile_polyline = { {-1.0, 0.0}, {2.0, 0.0}, {2.0, 2.0}, {-1.0, 2.0} };
    in.revolution_angle_deg = 360.0;

    auto r = skill::revolve_boss::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_THROW(skill::revolve_boss::apply(*stock, in),
                 skill::SkillError);
}

// ─── 4. DFM rejects bad revolution angle ──────────────────────────────────
TEST(SkillRevolveBoss, ValidateRejectsBadAngle)
{
    auto stock = emptyStock();

    skill::revolve_boss::Input in;
    in.profile_polyline = ringProfile();
    in.revolution_angle_deg = 400.0;  // > 360

    auto r = skill::revolve_boss::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_THROW(skill::revolve_boss::apply(*stock, in),
                 skill::SkillError);
}

// ─── 5. Recognize finds cylindrical faces ────────────────────────────────
TEST(SkillRevolveBoss, RecognizeFindsAxisymmetricFaces)
{
    auto stock = emptyStock();

    skill::revolve_boss::Input in;
    in.profile_polyline    = ringProfile();
    in.revolution_angle_deg = 360.0;

    auto out = skill::revolve_boss::apply(*stock, in);
    auto cands = skill::revolve_boss::recognize(*out.workpiece);
    ASSERT_FALSE(cands.empty());
    EXPECT_GT(cands.front().confidence, 0.3);

    // Signature pattern carries vertex count.
    EXPECT_EQ(out.signature.pattern.value("sketch_vertex_count", 0),
              static_cast<int>(in.profile_polyline.size()));
}

// ─── STEPPED meridian: a multi-segment turned part round-trips ─────────────
// Three coaxial segments (two cylinders + a cone frustum) — the single-face
// meridian paths cannot express this; the stepped assembler chains the
// segments (shoulder steps included) and the Pappus gate confirms the
// profile against the real solid before it ships.
TEST(SkillRevolveBoss, SteppedShaftMeridianRoundTrips)
{
    // A PURE stepped solid (no stock box — the cap-only specificity gate
    // rightly aborts on any prismatic walls in the shape): three coaxial
    // cylinder steps r5/r3/r1.5.  (A cone-TOPPED stack is claimed first by
    // the cone-protrusion path B-prime — the crown-knob detector — which
    // returns before the whole-part assembler; the stepped path's practical
    // target is the all-cylinder turned shaft.)
    TopoDS_Shape shaft = BRepPrimAPI_MakeCylinder(
        gp_Ax2(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1)), 5.0, 5.0).Shape();
    shaft = BRepAlgoAPI_Fuse(shaft, BRepPrimAPI_MakeCylinder(
        gp_Ax2(gp_Pnt(0, 0, 5.0), gp_Dir(0, 0, 1)), 3.0, 5.0).Shape()).Shape();
    shaft = BRepAlgoAPI_Fuse(shaft, BRepPrimAPI_MakeCylinder(
        gp_Ax2(gp_Pnt(0, 0, 10.0), gp_Dir(0, 0, 1)), 1.5, 3.0).Shape()).Shape();
    const double vOrig = volumeOf(shaft);

    // Foreign copy: NO feature history — recovery must be geometric.
    skill::Workpiece foreign(shaft);
    ASSERT_TRUE(foreign.features().empty());
    const auto cands = skill::revolve_boss::recognize(foreign);
    const skill::RecognizedFeature* g = nullptr;
    for (const auto& c : cands) {
        if (c.matched_geometry.value("source", std::string{}) != "geometry")
            continue;
        if (!c.recovered_params.contains("profile_polyline")) continue;
        if (c.recovered_params["profile_polyline"].size() >= 8) { g = &c; break; }
    }
    ASSERT_NE(g, nullptr)
        << "a stepped shaft must recover a multi-segment meridian "
           "(2 axis points + 2 per segment = 8 vertices for 3 segments)";

    // Regenerate from the RECOVERED profile on fresh stock: Pappus parity.
    skill::revolve_boss::Input rin;
    for (const auto& p : g->recovered_params["profile_polyline"])
        rin.profile_polyline.push_back(
            { p["r"].get<double>(), p["z"].get<double>() });
    const auto& ao = g->recovered_params["axis_origin"];
    const auto& ad = g->recovered_params["axis_dir"];
    rin.axis_origin = gp_Pnt(ao[0].get<double>(), ao[1].get<double>(),
                             ao[2].get<double>());
    rin.axis_dir    = gp_Dir(ad[0].get<double>(), ad[1].get<double>(),
                             ad[2].get<double>());
    rin.revolution_angle_deg = 360.0;
    auto stock2 = emptyStock();
    auto regen  = skill::revolve_boss::apply(*stock2, rin);
    const double vRegen = volumeOf(regen.workpiece->shape()) -
                          volumeOf(stock2->shape());
    EXPECT_NEAR(vRegen, vOrig, vOrig * 0.03)
        << "the recovered stepped meridian must revolve back to the same solid";
}

// ─── HOLLOW: a stepped PIPE with a THROUGH bore round-trips as an annulus ──
// Two outer wall steps (r14 z0-5, r12 z5-10) + a through bore (r10): the
// through inner wall becomes the profile's inner boundary; regeneration
// must reproduce the pipe's volume (the hole stays a hole).
TEST(SkillRevolveBoss, SteppedPipeMeridianRoundTrips)
{
    TopoDS_Shape pipe = BRepPrimAPI_MakeCylinder(
        gp_Ax2(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1)), 14.0, 5.0).Shape();
    pipe = BRepAlgoAPI_Fuse(pipe, BRepPrimAPI_MakeCylinder(
        gp_Ax2(gp_Pnt(0, 0, 5.0), gp_Dir(0, 0, 1)), 12.0, 5.0).Shape()).Shape();
    pipe = BRepAlgoAPI_Cut(pipe, BRepPrimAPI_MakeCylinder(
        gp_Ax2(gp_Pnt(0, 0, -1.0), gp_Dir(0, 0, 1)), 10.0, 12.0).Shape()).Shape();
    const double vOrig = volumeOf(pipe);

    skill::Workpiece foreign(pipe);
    ASSERT_TRUE(foreign.features().empty());
    const auto cands = skill::revolve_boss::recognize(foreign);
    const skill::RecognizedFeature* g = nullptr;
    for (const auto& c : cands) {
        if (c.matched_geometry.value("source", std::string{}) != "geometry")
            continue;
        if (!c.recovered_params.contains("profile_polyline")) continue;
        if (!c.recovered_params["profile_polyline"].empty()) { g = &c; break; }
    }
    ASSERT_NE(g, nullptr) << "a through-bored stepped pipe must recover an "
                             "annulus meridian";
    // The profile's minimum radius is the bore wall — the hole is preserved.
    double minR = 1e30;
    for (const auto& p : g->recovered_params["profile_polyline"])
        minR = std::min(minR, p["r"].get<double>());
    EXPECT_NEAR(minR, 10.0, 0.1);

    skill::revolve_boss::Input rin;
    for (const auto& p : g->recovered_params["profile_polyline"])
        rin.profile_polyline.push_back(
            { p["r"].get<double>(), p["z"].get<double>() });
    const auto& ao = g->recovered_params["axis_origin"];
    const auto& ad = g->recovered_params["axis_dir"];
    rin.axis_origin = gp_Pnt(ao[0].get<double>(), ao[1].get<double>(),
                             ao[2].get<double>());
    rin.axis_dir    = gp_Dir(ad[0].get<double>(), ad[1].get<double>(),
                             ad[2].get<double>());
    rin.revolution_angle_deg = 360.0;
    auto stock2 = emptyStock();
    auto regen  = skill::revolve_boss::apply(*stock2, rin);
    const double vRegen = volumeOf(regen.workpiece->shape()) -
                          volumeOf(stock2->shape());
    EXPECT_NEAR(vRegen, vOrig, vOrig * 0.03)
        << "the recovered annulus must revolve back to the same pipe";
}

// ─── BLIND bore: a partial-span inner wall keeps the meridian EMPTY ────────
// The bore wall does not span the whole part, so its floor would need
// modelling — the assembler must abstain instead of fabricating a profile
// that either fills the hole or drills it through.
TEST(SkillRevolveBoss, BlindBoreKeepsMeridianEmpty)
{
    TopoDS_Shape part = BRepPrimAPI_MakeCylinder(
        gp_Ax2(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1)), 14.0, 5.0).Shape();
    part = BRepAlgoAPI_Fuse(part, BRepPrimAPI_MakeCylinder(
        gp_Ax2(gp_Pnt(0, 0, 5.0), gp_Dir(0, 0, 1)), 12.0, 5.0).Shape()).Shape();
    part = BRepAlgoAPI_Cut(part, BRepPrimAPI_MakeCylinder(
        gp_Ax2(gp_Pnt(0, 0, 4.0), gp_Dir(0, 0, 1)), 10.0, 7.0).Shape()).Shape();

    skill::Workpiece foreign(part);
    for (const auto& c : skill::revolve_boss::recognize(foreign)) {
        if (c.matched_geometry.value("source", std::string{}) != "geometry")
            continue;
        if (c.recovered_params.contains("profile_polyline"))
            EXPECT_TRUE(c.recovered_params["profile_polyline"].empty())
                << "a BLIND bore's partial-span inner wall must abstain";
    }
}

// ─── ECCENTRIC journal: near-coaxial but offset — must abstain ──────────────
// The grouping tolerance (0.5 mm) would collect the offset middle step, but
// assembling it concentric about one axis would silently FLATTEN a real
// eccentric-stud class — and the Pappus gate cannot see a lateral offset of
// an axially-disjoint segment (volume is translation-invariant).
TEST(SkillRevolveBoss, EccentricJournalKeepsMeridianEmpty)
{
    TopoDS_Shape shaft = BRepPrimAPI_MakeCylinder(
        gp_Ax2(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1)), 5.0, 5.0).Shape();
    shaft = BRepAlgoAPI_Fuse(shaft, BRepPrimAPI_MakeCylinder(
        gp_Ax2(gp_Pnt(0.3, 0, 5.0), gp_Dir(0, 0, 1)), 3.0, 5.0).Shape()).Shape();
    shaft = BRepAlgoAPI_Fuse(shaft, BRepPrimAPI_MakeCylinder(
        gp_Ax2(gp_Pnt(0, 0, 10.0), gp_Dir(0, 0, 1)), 1.5, 3.0).Shape()).Shape();

    skill::Workpiece foreign(shaft);
    for (const auto& c : skill::revolve_boss::recognize(foreign)) {
        if (c.matched_geometry.value("source", std::string{}) != "geometry")
            continue;
        if (c.recovered_params.contains("profile_polyline"))
            EXPECT_TRUE(c.recovered_params["profile_polyline"].empty())
                << "a 0.3 mm eccentric step must NOT be concentricized into "
                   "a single-axis meridian";
    }
}
