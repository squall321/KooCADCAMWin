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

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
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
