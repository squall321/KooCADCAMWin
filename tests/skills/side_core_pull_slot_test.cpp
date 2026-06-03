// @lat: [[process/test-strategy#skill round-trip]]
//
// side_core_pull_slot — rectangular slot for side-action mold core.
//
// Cases (5):
//   1. ApplyRemovesVolume — slot cut decreases volume.
//   2. ValidateRejectsZeroDims.
//   3. ValidateRejectsTinyClearance.
//   4. SignatureRecordsCompoundKind.
//   5. RecognizeReplaysHistory.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/side_core_pull_slot.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

using namespace koocadcam;

namespace {
double volumeOf(const TopoDS_Shape& s)
{
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}
int findSideFace(const skill::Workpiece& wp)
{
    int    bestId   = -1;
    double bestArea = 0.0;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFacePlanar(i)) continue;
        gp_Dir n; try { n = wp.faceNormal(i); } catch (...) { continue; }
        // pick +X face (so retracting direction is +X).
        if (n.X() < 0.9) continue;
        const double a = wp.faceArea(i);
        if (a > bestArea) { bestArea = a; bestId = i; }
    }
    return bestId;
}
}  // namespace

// ─── 1. Slot removes volume ──────────────────────────────────────────────
TEST(SkillSideCorePullSlot, ApplyRemovesVolume)
{
    auto stock = skill::createCuboidStock(40.0, 30.0, 20.0);
    const double volBefore = volumeOf(stock->shape());
    const int fid = findSideFace(*stock);
    ASSERT_GE(fid, 0);

    skill::side_core_pull_slot::Input in;
    in.face_id              = fid;
    in.pull_dir_x           = 1.0;       // retract along +X (out of face)
    in.pull_dir_y           = 0.0;
    in.pull_dir_z           = 0.0;
    in.slot_width_mm        = 8.0;
    in.slot_height_mm       = 6.0;
    in.slot_depth_mm        = 5.0;
    in.retract_clearance_mm = 1.0;

    auto out = skill::side_core_pull_slot::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    const double volAfter = volumeOf(out.workpiece->shape());
    EXPECT_LT(volAfter, volBefore);
    EXPECT_GT(volBefore - volAfter, 1.0);
}

// ─── 2. DFM rejects zero dims ────────────────────────────────────────────
TEST(SkillSideCorePullSlot, ValidateRejectsZeroDims)
{
    auto stock = skill::createCuboidStock(40.0, 30.0, 20.0);
    const int fid = findSideFace(*stock);
    ASSERT_GE(fid, 0);

    skill::side_core_pull_slot::Input in;
    in.face_id              = fid;
    in.pull_dir_x           = 1.0;
    in.slot_width_mm        = 0.0;       // invalid
    in.slot_height_mm       = 6.0;
    in.slot_depth_mm        = 5.0;
    in.retract_clearance_mm = 1.0;

    auto r = skill::side_core_pull_slot::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_THROW(skill::side_core_pull_slot::apply(*stock, in),
                 skill::SkillError);
}

// ─── 3. DFM rejects clearance ≤ 0.5 ───────────────────────────────────────
TEST(SkillSideCorePullSlot, ValidateRejectsTinyClearance)
{
    auto stock = skill::createCuboidStock(40.0, 30.0, 20.0);
    const int fid = findSideFace(*stock);
    ASSERT_GE(fid, 0);

    skill::side_core_pull_slot::Input in;
    in.face_id              = fid;
    in.pull_dir_x           = 1.0;
    in.slot_width_mm        = 8.0;
    in.slot_height_mm       = 6.0;
    in.slot_depth_mm        = 5.0;
    in.retract_clearance_mm = 0.2;       // < 0.5

    auto r = skill::side_core_pull_slot::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool sawCode = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-SCORE-CLEARANCE") { sawCode = true; break; }
    EXPECT_TRUE(sawCode);
}

// ─── 4. Signature records compound + kind ────────────────────────────────
TEST(SkillSideCorePullSlot, SignatureRecordsCompoundKind)
{
    auto stock = skill::createCuboidStock(40.0, 30.0, 20.0);
    const int fid = findSideFace(*stock);
    ASSERT_GE(fid, 0);

    skill::side_core_pull_slot::Input in;
    in.face_id              = fid;
    in.pull_dir_x           = 1.0;
    in.slot_width_mm        = 8.0;
    in.slot_height_mm       = 6.0;
    in.slot_depth_mm        = 5.0;
    in.retract_clearance_mm = 1.0;

    auto out = skill::side_core_pull_slot::apply(*stock, in);
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("side_core_pull_slot"));
    EXPECT_TRUE(out.signature.pattern["is_compound"].get<bool>());
    EXPECT_EQ(out.signature.pattern["mold_feature_type"].get<std::string>(),
              std::string("side_core_slot"));
    EXPECT_GT(out.signature.pattern["derived_volume_removed"].get<double>(), 0.0);
}

// ─── 5. Recognize replays history ────────────────────────────────────────
TEST(SkillSideCorePullSlot, RecognizeReplaysHistory)
{
    auto stock = skill::createCuboidStock(40.0, 30.0, 20.0);
    const int fid = findSideFace(*stock);
    ASSERT_GE(fid, 0);

    skill::side_core_pull_slot::Input in;
    in.face_id              = fid;
    in.pull_dir_x           = 1.0;
    in.slot_width_mm        = 8.0;
    in.slot_height_mm       = 6.0;
    in.slot_depth_mm        = 5.0;
    in.retract_clearance_mm = 1.0;
    auto out = skill::side_core_pull_slot::apply(*stock, in);

    auto cands = skill::side_core_pull_slot::recognize(*out.workpiece);
    ASSERT_GE(cands.size(), 1u);
    EXPECT_GE(cands.front().confidence, 0.9);
    EXPECT_NEAR(cands.front().recovered_params["slot_width_mm"].get<double>(),
                8.0, 0.01);
}
