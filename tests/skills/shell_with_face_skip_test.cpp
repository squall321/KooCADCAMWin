// @lat: [[process/test-strategy#skill round-trip]]
//
// shell_with_face_skip — hollow the body with a uniform wall thickness while
// leaving certain faces open.
//
// Cases (5):
//   1. ApplyHollowsInteriorAndKeepsTopOpen — volume strictly less than stock.
//   2. ValidateRejectsZeroWall — DFM error & SkillError on apply.
//   3. ValidateRejectsOutOfRangeOpenFace — face id ≥ faceCount() errors.
//   4. SignatureRecordsCompoundKind — pattern.kind + is_compound=true.
//   5. RecognizeReplaysHistory — confidence ≥ 0.9.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/shell_with_face_skip.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

using namespace koocadcam;

namespace {
double volumeOf(const TopoDS_Shape& s)
{
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}

// Locate the face with the largest area whose normal is roughly +Z.  Returns
// -1 if none found.
int findTopFace(const skill::Workpiece& wp)
{
    int    bestId   = -1;
    double bestArea = 0.0;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFacePlanar(i)) continue;
        gp_Dir n; try { n = wp.faceNormal(i); } catch (...) { continue; }
        if (n.Z() < 0.9) continue;
        const double a = wp.faceArea(i);
        if (a > bestArea) { bestArea = a; bestId = i; }
    }
    return bestId;
}
}  // namespace

// ─── 1. Apply hollows the interior ────────────────────────────────────────
TEST(SkillShellWithFaceSkip, ApplyHollowsInteriorAndShrinksVolume)
{
    auto stock = skill::createCuboidStock(40.0, 30.0, 10.0);
    const double volBefore = volumeOf(stock->shape());

    const int topId = findTopFace(*stock);
    ASSERT_GE(topId, 0);

    skill::shell_with_face_skip::Input in;
    in.wall_thickness_mm = 1.5;
    in.open_face_ids     = { topId };

    auto out = skill::shell_with_face_skip::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    const double volAfter = volumeOf(out.workpiece->shape());

    EXPECT_LT(volAfter, volBefore);     // material removed → internal void
    EXPECT_GT(volBefore - volAfter, 1.0);   // non-trivial removal
    EXPECT_EQ(out.workpiece->features().size(), 1u);
}

// ─── 2. Validate rejects zero / negative wall_thickness ───────────────────
TEST(SkillShellWithFaceSkip, ValidateRejectsZeroWall)
{
    auto stock = skill::createCuboidStock(40.0, 30.0, 10.0);
    skill::shell_with_face_skip::Input in;
    in.wall_thickness_mm = 0.0;
    in.open_face_ids     = { 0 };
    auto r = skill::shell_with_face_skip::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_THROW(skill::shell_with_face_skip::apply(*stock, in),
                 skill::SkillError);
}

// ─── 3. Validate rejects out-of-range open_face_id ────────────────────────
TEST(SkillShellWithFaceSkip, ValidateRejectsOutOfRangeFaceId)
{
    auto stock = skill::createCuboidStock(40.0, 30.0, 10.0);
    skill::shell_with_face_skip::Input in;
    in.wall_thickness_mm = 1.0;
    in.open_face_ids     = { 9999 };
    auto r = skill::shell_with_face_skip::validate(*stock, in);
    EXPECT_FALSE(r.passed);
}

// ─── 4. Signature records kind + is_compound ──────────────────────────────
TEST(SkillShellWithFaceSkip, SignatureRecordsCompoundKind)
{
    auto stock = skill::createCuboidStock(40.0, 30.0, 10.0);
    const int topId = findTopFace(*stock);
    ASSERT_GE(topId, 0);

    skill::shell_with_face_skip::Input in;
    in.wall_thickness_mm = 1.5;
    in.open_face_ids     = { topId };
    auto out = skill::shell_with_face_skip::apply(*stock, in);
    EXPECT_EQ(out.signature.skill_id,
              std::string("shell_with_face_skip"));
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("shell_with_face_skip"));
    EXPECT_TRUE(out.signature.pattern["is_compound"].get<bool>());
}

// ─── 5. Recognize replays history ─────────────────────────────────────────
TEST(SkillShellWithFaceSkip, RecognizeReplaysHistory)
{
    auto stock = skill::createCuboidStock(40.0, 30.0, 10.0);
    const int topId = findTopFace(*stock);
    ASSERT_GE(topId, 0);

    skill::shell_with_face_skip::Input in;
    in.wall_thickness_mm = 1.5;
    in.open_face_ids     = { topId };
    auto out = skill::shell_with_face_skip::apply(*stock, in);
    auto cands = skill::shell_with_face_skip::recognize(*out.workpiece);
    ASSERT_GE(cands.size(), 1u);
    EXPECT_EQ(cands.front().skill_id,
              std::string("shell_with_face_skip"));
    EXPECT_GE(cands.front().confidence, 0.9);
}
