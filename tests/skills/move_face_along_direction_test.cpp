// @lat: [[process/test-strategy#skill round-trip]]
//
// move_face_along_direction — approximate face-move via prism fuse/cut.
//
// Cases (5):
//   1. apply (distance > 0) on a planar face INCREASES volume.
//   2. apply (distance < 0) on a planar face DECREASES volume.
//   3. DFM rejects zero-length direction.
//   4. DFM rejects zero distance.
//   5. recognize replays history.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/move_face_along_direction.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

using namespace koocadcam;

namespace {
double volumeOf(const TopoDS_Shape& s)
{
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}
// Find first planar face whose outward normal is +Z (top face of cuboid).
int findTopFaceId(const skill::Workpiece& wp)
{
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFacePlanar(i)) continue;
        gp_Dir n = wp.faceNormal(i);
        if (n.Z() > 0.95) return i;
    }
    return -1;
}
}  // namespace

// ─── 1. distance > 0 increases volume ──────────────────────────────────────
TEST(SkillMoveFaceAlongDirection, OutwardMoveIncreasesVolume)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);
    const int topId = findTopFaceId(*stock);
    ASSERT_GE(topId, 0);

    const double volBefore = volumeOf(stock->shape());

    skill::move_face_along_direction::Input in;
    in.face_id       = topId;
    in.direction_xyz = { 0.0, 0.0, 1.0 };
    in.distance_mm   = 3.0;

    auto out = skill::move_face_along_direction::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    const double volAfter = volumeOf(out.workpiece->shape());
    // Added approximately the cuboid footprint (50x50) × 3 = 7500 mm³.
    EXPECT_GT(volAfter, volBefore + 5000.0);

    EXPECT_EQ(out.signature.skill_id, std::string("move_face_along_direction"));
    EXPECT_FALSE(out.signature.pattern["is_compound"].get<bool>());
    EXPECT_TRUE(out.signature.pattern["outward"].get<bool>());
}

// ─── 2. distance < 0 decreases volume ──────────────────────────────────────
TEST(SkillMoveFaceAlongDirection, InwardMoveDecreasesVolume)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);
    const int topId = findTopFaceId(*stock);
    ASSERT_GE(topId, 0);

    const double volBefore = volumeOf(stock->shape());

    skill::move_face_along_direction::Input in;
    in.face_id       = topId;
    in.direction_xyz = { 0.0, 0.0, 1.0 };
    in.distance_mm   = -2.0;

    auto out = skill::move_face_along_direction::apply(*stock, in);
    const double volAfter = volumeOf(out.workpiece->shape());
    EXPECT_LT(volAfter, volBefore);
}

// ─── 3. DFM rejects zero-direction ─────────────────────────────────────────
TEST(SkillMoveFaceAlongDirection, ValidateRejectsZeroDirection)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 5.0);
    skill::move_face_along_direction::Input in;
    in.face_id       = 0;
    in.direction_xyz = { 0.0, 0.0, 0.0 };
    in.distance_mm   = 1.0;
    auto r = skill::move_face_along_direction::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_THROW(skill::move_face_along_direction::apply(*stock, in), skill::SkillError);
}

// ─── 4. DFM rejects zero distance ──────────────────────────────────────────
TEST(SkillMoveFaceAlongDirection, ValidateRejectsZeroDistance)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 5.0);
    skill::move_face_along_direction::Input in;
    in.face_id       = 0;
    in.direction_xyz = { 0.0, 0.0, 1.0 };
    in.distance_mm   = 0.0;
    auto r = skill::move_face_along_direction::validate(*stock, in);
    EXPECT_FALSE(r.passed);
}

// ─── 5. recognize replays history ──────────────────────────────────────────
TEST(SkillMoveFaceAlongDirection, RecognizeReplaysHistory)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 8.0);
    const int topId = findTopFaceId(*stock);
    ASSERT_GE(topId, 0);

    skill::move_face_along_direction::Input in;
    in.face_id       = topId;
    in.direction_xyz = { 0.0, 0.0, 1.0 };
    in.distance_mm   = 2.5;

    auto out   = skill::move_face_along_direction::apply(*stock, in);
    auto cands = skill::move_face_along_direction::recognize(*out.workpiece);

    ASSERT_GE(cands.size(), 1u);
    EXPECT_EQ(cands[0].skill_id, std::string("move_face_along_direction"));
    EXPECT_GT(cands[0].confidence, 0.9);
    EXPECT_NEAR(cands[0].recovered_params["distance_mm"].get<double>(), 2.5, 1e-9);
}
