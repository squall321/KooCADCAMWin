// @lat: [[process/test-strategy#skill round-trip]]
//
// revolve_cut — 2D profile revolved & SUBTRACTED from stock.
//
// Tests:
//   1. Apply removes the expected Pappus volume from a cylindrical stock.
//   2. DFM rejects too-few profile vertices.
//   3. DFM rejects negative radius.
//   4. Signature pattern carries cut volume.
//   5. Recognize returns at least one candidate.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/revolve_cut.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
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

// Profile cuts a small ring groove on the outside of a cylindrical bar.
// Cylinder OD = 40 mm (r = 20), thickness = 20 mm.  Groove: r ∈ [18, 20],
// z ∈ [5, 8] → cross-section 2 × 3 = 6 mm², centroid r = 19.
// Pappus V removed = 2π · 19 · 6 = 228π ≈ 716.28 mm³.
std::vector<std::pair<double,double>> grooveProfile()
{
    return { {18.0, 5.0}, {20.0, 5.0}, {20.0, 8.0}, {18.0, 8.0} };
}

}  // namespace

// ─── 1. Apply removes the right Pappus volume ─────────────────────────────
TEST(SkillRevolveCut, ApplyRemovesRealVolume)
{
    auto stock = skill::createCylindricalStock(40.0, 20.0);
    ASSERT_FALSE(stock->shape().IsNull());
    const double v0 = volumeOf(stock->shape());

    skill::revolve_cut::Input in;
    in.profile_polyline    = grooveProfile();
    in.axis_origin         = gp_Pnt(0.0, 0.0, 0.0);
    in.axis_dir            = gp_Dir(0.0, 0.0, 1.0);
    in.revolution_angle_deg = 360.0;

    auto out = skill::revolve_cut::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    const double v1 = volumeOf(out.workpiece->shape());

    // Pappus removed = 2π × 19 × 6 = 228π ≈ 716.28 mm³.
    const double expected = 2.0 * M_PI * 19.0 * 6.0;
    EXPECT_NEAR(v0 - v1, expected, expected * 0.05);
}

// ─── 2. DFM rejects too-few profile vertices ──────────────────────────────
TEST(SkillRevolveCut, ValidateRejectsTooFewVerts)
{
    auto stock = skill::createCylindricalStock(40.0, 20.0);

    skill::revolve_cut::Input in;
    in.profile_polyline = { {18.0, 5.0}, {20.0, 5.0} };
    in.revolution_angle_deg = 360.0;

    auto r = skill::revolve_cut::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_THROW(skill::revolve_cut::apply(*stock, in),
                 skill::SkillError);
}

// ─── 3. DFM rejects negative radius ───────────────────────────────────────
TEST(SkillRevolveCut, ValidateRejectsNegativeRadius)
{
    auto stock = skill::createCylindricalStock(40.0, 20.0);

    skill::revolve_cut::Input in;
    in.profile_polyline = { {-1.0, 5.0}, {1.0, 5.0}, {1.0, 8.0}, {-1.0, 8.0} };
    in.revolution_angle_deg = 360.0;

    auto r = skill::revolve_cut::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_THROW(skill::revolve_cut::apply(*stock, in),
                 skill::SkillError);
}

// ─── 4. Signature compound ────────────────────────────────────────────────
TEST(SkillRevolveCut, SignatureCompound)
{
    auto stock = skill::createCylindricalStock(40.0, 20.0);

    skill::revolve_cut::Input in;
    in.profile_polyline = grooveProfile();
    in.revolution_angle_deg = 360.0;

    auto out = skill::revolve_cut::apply(*stock, in);
    EXPECT_EQ(out.signature.skill_id, std::string("revolve_cut"));
    EXPECT_TRUE(out.signature.pattern.value("is_compound", false));
    EXPECT_EQ(out.signature.pattern.value("sketch_vertex_count", 0), 4);

    // Derived volume should match expected Pappus.
    const double expected = 2.0 * M_PI * 19.0 * 6.0;
    EXPECT_NEAR(out.signature.pattern.value("derived_volume_mm3", 0.0),
                expected, expected * 0.01);
}

// ─── 5. Recognize finds cylindrical inner face ────────────────────────────
TEST(SkillRevolveCut, RecognizeFindsInnerCyl)
{
    auto stock = skill::createCylindricalStock(40.0, 20.0);

    skill::revolve_cut::Input in;
    in.profile_polyline    = grooveProfile();
    in.revolution_angle_deg = 360.0;

    auto out = skill::revolve_cut::apply(*stock, in);
    auto cands = skill::revolve_cut::recognize(*out.workpiece);
    ASSERT_FALSE(cands.empty());
    EXPECT_GT(cands.front().confidence, 0.3);
}
