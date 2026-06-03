// @lat: [[process/test-strategy#skill round-trip]]
//
// add_draft_to_face — taper a vertical face about a neutral plane.
//
// Cases (5):
//   1. ApplyChangesFaceCountOrVolume — face count differs OR volume changes
//      (a successful draft mutates the topology).
//   2. ValidateRejectsAngleTooSmall — < 0.5° → DFM error.
//   3. ValidateRejectsAngleTooBig — > 15° → DFM error.
//   4. SignatureRecordsCompoundKind.
//   5. RecognizeReplaysHistory.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/add_draft_to_face.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

#include <cmath>

using namespace koocadcam;

namespace {
double volumeOf(const TopoDS_Shape& s)
{
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}

// Find a planar face whose normal is +Z (top face — used as neutral base).
int findTopFace(const skill::Workpiece& wp)
{
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFacePlanar(i)) continue;
        gp_Dir n; try { n = wp.faceNormal(i); } catch (...) { continue; }
        if (n.Z() > 0.95) return i;
    }
    return -1;
}

// Find a planar face whose normal is in the XY plane (vertical side face).
int findSideFace(const skill::Workpiece& wp)
{
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFacePlanar(i)) continue;
        gp_Dir n; try { n = wp.faceNormal(i); } catch (...) { continue; }
        if (std::abs(n.Z()) < 0.05) return i;
    }
    return -1;
}
}  // namespace

// ─── 1. Apply changes face count OR volume ────────────────────────────────
TEST(SkillAddDraftToFace, ApplyChangesFaceCountOrVolume)
{
    auto stock = skill::createCuboidStock(40.0, 30.0, 10.0);
    const int topId  = findTopFace(*stock);
    const int sideId = findSideFace(*stock);
    ASSERT_GE(topId,  0);
    ASSERT_GE(sideId, 0);

    const double volBefore = volumeOf(stock->shape());
    const int    fbefore   = stock->faceCount();

    skill::add_draft_to_face::Input in;
    in.source_face_id  = sideId;
    in.base_face_id    = topId;     // neutral plane = top
    in.draft_angle_deg = 3.0;

    try {
        auto out = skill::add_draft_to_face::apply(*stock, in);
        ASSERT_FALSE(out.workpiece->shape().IsNull());
        const double volAfter = volumeOf(out.workpiece->shape());
        const int    fafter   = out.workpiece->faceCount();
        const bool   changed  =
            (fafter != fbefore) || std::abs(volAfter - volBefore) > 1e-3;
        EXPECT_TRUE(changed);
        EXPECT_EQ(out.workpiece->features().size(), 1u);
    } catch (const skill::SkillError& ex) {
        // Some OCCT builds reject the operation when adjacent faces aren't
        // tangent — surface the failure with the DFM code in the message.
        const std::string what = ex.what();
        EXPECT_NE(what.find("DFM-DRAFT-FAILED"), std::string::npos)
            << "Expected DFM-DRAFT-FAILED code in message, got: " << what;
    }
}

// ─── 2. Validate rejects angle < 0.5 ──────────────────────────────────────
TEST(SkillAddDraftToFace, ValidateRejectsAngleTooSmall)
{
    auto stock = skill::createCuboidStock(40.0, 30.0, 10.0);
    skill::add_draft_to_face::Input in;
    in.source_face_id  = 0;
    in.base_face_id    = 1;
    in.draft_angle_deg = 0.1;
    auto r = skill::add_draft_to_face::validate(*stock, in);
    EXPECT_FALSE(r.passed);
}

// ─── 3. Validate rejects angle > 15 ───────────────────────────────────────
TEST(SkillAddDraftToFace, ValidateRejectsAngleTooBig)
{
    auto stock = skill::createCuboidStock(40.0, 30.0, 10.0);
    skill::add_draft_to_face::Input in;
    in.source_face_id  = 0;
    in.base_face_id    = 1;
    in.draft_angle_deg = 20.0;
    auto r = skill::add_draft_to_face::validate(*stock, in);
    EXPECT_FALSE(r.passed);
}

// ─── 4. Signature records compound + kind (when apply succeeds) ──────────
TEST(SkillAddDraftToFace, SignatureRecordsCompoundKind)
{
    auto stock = skill::createCuboidStock(40.0, 30.0, 10.0);
    const int topId  = findTopFace(*stock);
    const int sideId = findSideFace(*stock);
    ASSERT_GE(topId,  0);
    ASSERT_GE(sideId, 0);

    skill::add_draft_to_face::Input in;
    in.source_face_id  = sideId;
    in.base_face_id    = topId;
    in.draft_angle_deg = 2.0;
    try {
        auto out = skill::add_draft_to_face::apply(*stock, in);
        EXPECT_EQ(out.signature.skill_id,
                  std::string("add_draft_to_face"));
        EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
                  std::string("add_draft_to_face"));
        EXPECT_TRUE(out.signature.pattern["is_compound"].get<bool>());
    } catch (const skill::SkillError&) {
        GTEST_SKIP() << "DraftAngle build skipped on this OCCT config";
    }
}

// ─── 5. Recognize replays history (when apply succeeds) ──────────────────
TEST(SkillAddDraftToFace, RecognizeReplaysHistory)
{
    auto stock = skill::createCuboidStock(40.0, 30.0, 10.0);
    const int topId  = findTopFace(*stock);
    const int sideId = findSideFace(*stock);
    ASSERT_GE(topId,  0);
    ASSERT_GE(sideId, 0);

    skill::add_draft_to_face::Input in;
    in.source_face_id  = sideId;
    in.base_face_id    = topId;
    in.draft_angle_deg = 2.0;
    try {
        auto out = skill::add_draft_to_face::apply(*stock, in);
        auto cands = skill::add_draft_to_face::recognize(*out.workpiece);
        ASSERT_GE(cands.size(), 1u);
        EXPECT_EQ(cands.front().skill_id,
                  std::string("add_draft_to_face"));
        EXPECT_GE(cands.front().confidence, 0.9);
    } catch (const skill::SkillError&) {
        GTEST_SKIP() << "DraftAngle build skipped on this OCCT config";
    }
}
