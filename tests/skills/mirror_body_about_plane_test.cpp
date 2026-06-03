// @lat: [[process/test-strategy#skill round-trip]]
//
// mirror_body_about_plane — mirror the workpiece about a plane and fuse with
// the original.
//
// Cases (5):
//   1. ApplyDoublesVolume — result ≈ 2× original (within 5 %).
//   2. ValidateRejectsZeroNormal — DFM error & SkillError.
//   3. SignatureRecordsPlaneAndCompound.
//   4. RecognizeReplaysHistory.
//   5. ApplyMirrorAboutXOffsetSeparates — body mirrored to other side fuses
//      into one disjoint compound (volume still ≈ 2×).

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/mirror_body_about_plane.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

using namespace koocadcam;

namespace {
double volumeOf(const TopoDS_Shape& s)
{
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}
}  // namespace

// ─── 1. Mirror about a plane outside the body doubles the volume ──────────
TEST(SkillMirrorBodyAboutPlane, ApplyAboutOffsetPlaneDoublesVolume)
{
    // Cuboid stock from (0,0,0) → (40,30,10).
    auto stock = skill::createCuboidStock(40.0, 30.0, 10.0);
    const double volBefore = volumeOf(stock->shape());

    skill::mirror_body_about_plane::Input in;
    in.plane_origin = { 0.0, 0.0, 0.0 };     // mirror about YZ plane
    in.plane_normal = { 1.0, 0.0, 0.0 };

    auto out = skill::mirror_body_about_plane::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    const double volAfter = volumeOf(out.workpiece->shape());

    // Body sits in +X; mirror puts a copy in -X → 2 disjoint solids ≈ 2× vol.
    EXPECT_NEAR(volAfter, 2.0 * volBefore, 0.05 * volBefore);
    EXPECT_EQ(out.workpiece->features().size(), 1u);
}

// ─── 2. Validate rejects zero-length normal ───────────────────────────────
TEST(SkillMirrorBodyAboutPlane, ValidateRejectsZeroNormal)
{
    auto stock = skill::createCuboidStock(40.0, 30.0, 10.0);
    skill::mirror_body_about_plane::Input in;
    in.plane_origin = { 0.0, 0.0, 0.0 };
    in.plane_normal = { 0.0, 0.0, 0.0 };
    auto r = skill::mirror_body_about_plane::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_THROW(skill::mirror_body_about_plane::apply(*stock, in),
                 skill::SkillError);
}

// ─── 3. Signature records compound + plane parameters ─────────────────────
TEST(SkillMirrorBodyAboutPlane, SignatureRecordsPlaneAndCompound)
{
    auto stock = skill::createCuboidStock(40.0, 30.0, 10.0);
    skill::mirror_body_about_plane::Input in;
    in.plane_origin = { 0.0, 0.0, 0.0 };
    in.plane_normal = { 1.0, 0.0, 0.0 };
    auto out = skill::mirror_body_about_plane::apply(*stock, in);
    EXPECT_EQ(out.signature.skill_id,
              std::string("mirror_body_about_plane"));
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("mirror_body_about_plane"));
    EXPECT_TRUE(out.signature.pattern["is_compound"].get<bool>());
    EXPECT_TRUE(out.signature.params.contains("plane_origin"));
    EXPECT_TRUE(out.signature.params.contains("plane_normal"));
}

// ─── 4. Recognize replays history ─────────────────────────────────────────
TEST(SkillMirrorBodyAboutPlane, RecognizeReplaysHistory)
{
    auto stock = skill::createCuboidStock(40.0, 30.0, 10.0);
    skill::mirror_body_about_plane::Input in;
    in.plane_origin = { 0.0, 0.0, 0.0 };
    in.plane_normal = { 1.0, 0.0, 0.0 };
    auto out = skill::mirror_body_about_plane::apply(*stock, in);
    auto cands = skill::mirror_body_about_plane::recognize(*out.workpiece);
    ASSERT_GE(cands.size(), 1u);
    EXPECT_EQ(cands.front().skill_id,
              std::string("mirror_body_about_plane"));
    EXPECT_GE(cands.front().confidence, 0.9);
}

// ─── 5. Mirror about Y-axis with offset still doubles volume ──────────────
TEST(SkillMirrorBodyAboutPlane, ApplyAboutYZPlaneAtOriginDoubles)
{
    auto stock = skill::createCuboidStock(20.0, 15.0, 8.0);
    const double volBefore = volumeOf(stock->shape());

    skill::mirror_body_about_plane::Input in;
    in.plane_origin = { -5.0, 0.0, 0.0 };    // plane to the left of stock
    in.plane_normal = { 1.0, 0.0, 0.0 };

    auto out = skill::mirror_body_about_plane::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    const double volAfter = volumeOf(out.workpiece->shape());
    EXPECT_NEAR(volAfter, 2.0 * volBefore, 0.05 * volBefore);
}
