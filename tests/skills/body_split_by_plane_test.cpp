// @lat: [[process/test-strategy#skill round-trip]]
//
// body_split_by_plane — split workpiece by an arbitrary plane → compound.
//
// Cases (5):
//   1. apply produces a compound with ≥ 2 solids; total volume preserved.
//   2. DFM rejects zero-length plane_normal.
//   3. DFM rejects plane that misses the bbox.
//   4. recognize identifies the split-compound.
//   5. apply with horizontal plane (Z-cut) yields two roughly equal halves.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/body_split_by_plane.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS_Shape.hxx>

#include <cmath>

using namespace koocadcam;

namespace {
double volumeOf(const TopoDS_Shape& s)
{
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}
int countSolids(const TopoDS_Shape& s)
{
    int n = 0;
    for (TopExp_Explorer e(s, TopAbs_SOLID); e.More(); e.Next()) ++n;
    return n;
}
}  // namespace

// ─── 1. apply produces compound with 2+ solids ─────────────────────────────
TEST(SkillBodySplitByPlane, ApplyProducesCompoundOfTwoSolids)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 20.0);
    const double volBefore = volumeOf(stock->shape());

    skill::body_split_by_plane::Input in;
    in.plane_origin = { 20.0, 20.0, 10.0 };
    in.plane_normal = { 0.0, 0.0, 1.0 };

    auto out = skill::body_split_by_plane::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    EXPECT_GE(countSolids(out.workpiece->shape()), 2);

    const double volAfter = volumeOf(out.workpiece->shape());
    EXPECT_NEAR(volAfter, volBefore, volBefore * 1e-3);

    EXPECT_TRUE(out.signature.pattern["is_compound"].get<bool>());
    EXPECT_GE(out.signature.pattern["solid_count"].get<int>(), 2);
}

// ─── 2. DFM rejects zero-length normal ─────────────────────────────────────
TEST(SkillBodySplitByPlane, ValidateRejectsZeroNormal)
{
    auto stock = skill::createCuboidStock(30.0, 30.0, 10.0);
    skill::body_split_by_plane::Input in;
    in.plane_origin = { 15.0, 15.0, 5.0 };
    in.plane_normal = { 0.0, 0.0, 0.0 };
    auto r = skill::body_split_by_plane::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_THROW(skill::body_split_by_plane::apply(*stock, in), skill::SkillError);
}

// ─── 3. DFM rejects plane that does not intersect bbox ─────────────────────
TEST(SkillBodySplitByPlane, ValidateRejectsMissPlane)
{
    auto stock = skill::createCuboidStock(30.0, 30.0, 10.0);
    skill::body_split_by_plane::Input in;
    in.plane_origin = { 15.0, 15.0, 500.0 };   // way above stock
    in.plane_normal = { 0.0, 0.0, 1.0 };
    auto r = skill::body_split_by_plane::validate(*stock, in);
    EXPECT_FALSE(r.passed);
}

// ─── 4. recognize identifies split compound ────────────────────────────────
TEST(SkillBodySplitByPlane, RecognizeIdentifiesSplit)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 20.0);
    skill::body_split_by_plane::Input in;
    in.plane_origin = { 20.0, 20.0, 10.0 };
    in.plane_normal = { 0.0, 0.0, 1.0 };
    auto out   = skill::body_split_by_plane::apply(*stock, in);
    auto cands = skill::body_split_by_plane::recognize(*out.workpiece);
    ASSERT_GE(cands.size(), 1u);
    EXPECT_EQ(cands[0].skill_id, std::string("body_split_by_plane"));
    EXPECT_GT(cands[0].confidence, 0.85);
}

// ─── 5. horizontal split yields two near-equal halves ──────────────────────
TEST(SkillBodySplitByPlane, HalvesAreApproxEqual)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 20.0);
    skill::body_split_by_plane::Input in;
    in.plane_origin = { 20.0, 20.0, 10.0 };
    in.plane_normal = { 0.0, 0.0, 1.0 };
    auto out = skill::body_split_by_plane::apply(*stock, in);

    std::vector<double> vols;
    for (TopExp_Explorer e(out.workpiece->shape(), TopAbs_SOLID); e.More(); e.Next()) {
        GProp_GProps p; BRepGProp::VolumeProperties(e.Current(), p);
        vols.push_back(p.Mass());
    }
    ASSERT_GE(vols.size(), 2u);
    EXPECT_NEAR(vols[0], vols[1], vols[0] * 0.05);
}
