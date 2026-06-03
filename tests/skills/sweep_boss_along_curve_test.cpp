// @lat: [[process/test-strategy#skill round-trip]]
//
// sweep_boss_along_curve — sweep a closed profile along a 3D path
// and FUSE the resulting tube onto the workpiece.
//
// Tests (5):
//   1. Apply adds real volume ≈ profile area × path length (±25 %).
//   2. DFM rejects path with < 2 waypoints.
//   3. DFM rejects profile with < 3 distinct vertices.
//   4. Signature pattern carries is_compound + path_length + derived_volume.
//   5. Recognize replays the feature from history.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/sweep_boss_along_curve.hpp"

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

// 6x6 mm square profile centered on origin → area = 36 mm².
std::vector<std::pair<double,double>> square6()
{
    return { { -3.0, -3.0 }, { 3.0, -3.0 }, { 3.0, 3.0 }, { -3.0, 3.0 } };
}

}  // namespace

// ─── 1. Apply adds real volume ────────────────────────────────────────────
TEST(SkillSweepBossAlongCurve, ApplyAddsRealVolume)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 10.0);
    const double v0 = volumeOf(stock->shape());

    skill::sweep_boss_along_curve::Input in;
    in.profile_polyline = square6();
    // Straight 30-mm path along +X starting outside the stock.
    in.path_polyline_xyz = {
        gp_Pnt(80.0, 30.0, 30.0),
        gp_Pnt(110.0, 30.0, 30.0),
    };

    auto out = skill::sweep_boss_along_curve::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    const double v1 = volumeOf(out.workpiece->shape());

    // Expected = 6 × 6 × 30 = 1080 mm³.
    const double expected = 36.0 * 30.0;
    EXPECT_NEAR(v1 - v0, expected, expected * 0.25);
}

// ─── 2. DFM rejects too-few path waypoints ────────────────────────────────
TEST(SkillSweepBossAlongCurve, ValidateRejectsShortPath)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 10.0);

    skill::sweep_boss_along_curve::Input in;
    in.profile_polyline  = square6();
    in.path_polyline_xyz = { gp_Pnt(0.0, 0.0, 0.0) };   // only 1 pt

    auto r = skill::sweep_boss_along_curve::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_THROW(skill::sweep_boss_along_curve::apply(*stock, in),
                 skill::SkillError);
}

// ─── 3. DFM rejects degenerate profile ────────────────────────────────────
TEST(SkillSweepBossAlongCurve, ValidateRejectsBadProfile)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 10.0);

    skill::sweep_boss_along_curve::Input in;
    in.profile_polyline  = { { 0.0, 0.0 }, { 1.0, 0.0 } };   // only 2 verts
    in.path_polyline_xyz = {
        gp_Pnt(0.0, 0.0, 0.0), gp_Pnt(10.0, 0.0, 0.0)
    };

    auto r = skill::sweep_boss_along_curve::validate(*stock, in);
    EXPECT_FALSE(r.passed);
}

// ─── 4. Signature pattern ─────────────────────────────────────────────────
TEST(SkillSweepBossAlongCurve, SignatureCompound)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 10.0);

    skill::sweep_boss_along_curve::Input in;
    in.profile_polyline  = square6();
    in.path_polyline_xyz = {
        gp_Pnt(80.0, 30.0, 30.0),
        gp_Pnt(110.0, 30.0, 30.0),
    };

    auto out = skill::sweep_boss_along_curve::apply(*stock, in);
    EXPECT_EQ(out.signature.skill_id, std::string("sweep_boss_along_curve"));
    EXPECT_TRUE(out.signature.pattern.value("is_compound", false));
    EXPECT_EQ(out.signature.pattern.value("section_count", 0), 1);
    EXPECT_NEAR(out.signature.pattern.value("path_length_mm", 0.0), 30.0, 1e-6);
    EXPECT_GT(out.signature.pattern.value("derived_volume_mm3", 0.0), 800.0);
}

// ─── 5. Recognize replays history ─────────────────────────────────────────
TEST(SkillSweepBossAlongCurve, RecognizeReplaysHistory)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 10.0);

    skill::sweep_boss_along_curve::Input in;
    in.profile_polyline  = square6();
    in.path_polyline_xyz = {
        gp_Pnt(80.0, 30.0, 30.0),
        gp_Pnt(110.0, 30.0, 30.0),
    };

    auto out   = skill::sweep_boss_along_curve::apply(*stock, in);
    auto cands = skill::sweep_boss_along_curve::recognize(*out.workpiece);
    ASSERT_GE(cands.size(), 1u);
    EXPECT_EQ(cands.front().skill_id, std::string("sweep_boss_along_curve"));
    EXPECT_GT(cands.front().confidence, 0.4);
}
