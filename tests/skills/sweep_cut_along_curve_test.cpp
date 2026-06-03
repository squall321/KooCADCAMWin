// @lat: [[process/test-strategy#skill round-trip]]
//
// sweep_cut_along_curve — sweep a closed profile along a 3D path and
// SUBTRACT the resulting tube from the workpiece.
//
// Tests (5):
//   1. Apply removes real volume ≈ profile area × overlap path length (±25 %).
//   2. DFM rejects path with < 2 waypoints.
//   3. DFM rejects profile with < 3 distinct vertices.
//   4. Signature pattern carries is_compound + section_count.
//   5. Recognize replays from history.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/sweep_cut_along_curve.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

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

std::vector<std::pair<double,double>> square4()
{
    return { { -2.0, -2.0 }, { 2.0, -2.0 }, { 2.0, 2.0 }, { -2.0, 2.0 } };
}

}  // namespace

// ─── 1. Apply removes real volume ─────────────────────────────────────────
TEST(SkillSweepCutAlongCurve, ApplyRemovesRealVolume)
{
    // 60 × 60 × 20 = 72,000 mm³ block.
    auto stock = skill::createCuboidStock(60.0, 60.0, 20.0);
    const double v0 = volumeOf(stock->shape());

    skill::sweep_cut_along_curve::Input in;
    in.profile_polyline = square4();   // area = 16 mm²
    // Cut path through the block along +X, fully inside.
    // Path is centered in YZ, sweeping +X from edge to edge.
    in.path_polyline_xyz = {
        gp_Pnt(-5.0, 30.0, 10.0),     // start outside −X face
        gp_Pnt(65.0, 30.0, 10.0),     // end outside +X face
    };

    auto out = skill::sweep_cut_along_curve::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    const double v1 = volumeOf(out.workpiece->shape());

    // Overlap path length inside the block ≈ 60 mm; expected removed
    // ≈ 16 × 60 = 960 mm³.
    const double expected = 16.0 * 60.0;
    EXPECT_NEAR(v0 - v1, expected, expected * 0.25);
}

// ─── 2. DFM rejects too-few path waypoints ────────────────────────────────
TEST(SkillSweepCutAlongCurve, ValidateRejectsShortPath)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 20.0);

    skill::sweep_cut_along_curve::Input in;
    in.profile_polyline  = square4();
    in.path_polyline_xyz = {};

    auto r = skill::sweep_cut_along_curve::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_THROW(skill::sweep_cut_along_curve::apply(*stock, in),
                 skill::SkillError);
}

// ─── 3. DFM rejects degenerate profile ────────────────────────────────────
TEST(SkillSweepCutAlongCurve, ValidateRejectsBadProfile)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 20.0);

    skill::sweep_cut_along_curve::Input in;
    in.profile_polyline  = { { 0.0, 0.0 } };  // 1 vertex
    in.path_polyline_xyz = {
        gp_Pnt(0.0, 0.0, 0.0), gp_Pnt(10.0, 0.0, 0.0)
    };

    auto r = skill::sweep_cut_along_curve::validate(*stock, in);
    EXPECT_FALSE(r.passed);
}

// ─── 4. Signature pattern ─────────────────────────────────────────────────
TEST(SkillSweepCutAlongCurve, SignatureCompound)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 20.0);

    skill::sweep_cut_along_curve::Input in;
    in.profile_polyline  = square4();
    in.path_polyline_xyz = {
        gp_Pnt(-5.0, 30.0, 10.0), gp_Pnt(65.0, 30.0, 10.0),
    };

    auto out = skill::sweep_cut_along_curve::apply(*stock, in);
    EXPECT_EQ(out.signature.skill_id, std::string("sweep_cut_along_curve"));
    EXPECT_TRUE(out.signature.pattern.value("is_compound", false));
    EXPECT_EQ(out.signature.pattern.value("section_count", 0), 1);
    EXPECT_NEAR(out.signature.pattern.value("path_length_mm", 0.0), 70.0, 1e-6);
    EXPECT_GT(out.signature.pattern.value("derived_volume_mm3", 0.0), 500.0);
}

// ─── 5. Recognize replays history ─────────────────────────────────────────
TEST(SkillSweepCutAlongCurve, RecognizeReplaysHistory)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 20.0);

    skill::sweep_cut_along_curve::Input in;
    in.profile_polyline  = square4();
    in.path_polyline_xyz = {
        gp_Pnt(-5.0, 30.0, 10.0), gp_Pnt(65.0, 30.0, 10.0),
    };

    auto out   = skill::sweep_cut_along_curve::apply(*stock, in);
    auto cands = skill::sweep_cut_along_curve::recognize(*out.workpiece);
    ASSERT_GE(cands.size(), 1u);
    EXPECT_EQ(cands.front().skill_id, std::string("sweep_cut_along_curve"));
    EXPECT_GT(cands.front().confidence, 0.4);
}
