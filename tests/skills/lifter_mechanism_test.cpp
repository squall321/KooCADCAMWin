// @lat: [[process/test-strategy#skill round-trip]]
//
// lifter_mechanism — angled prismatic channel representing a lifter envelope.
//
// Cases (5):
//   1. ApplyRemovesVolume.
//   2. ValidateRejectsBadAngle.
//   3. ValidateRejectsTinyStroke.
//   4. SignatureRecordsCompoundKind.
//   5. RecognizeReplaysHistory.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/lifter_mechanism.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

using namespace koocadcam;

namespace {
double volumeOf(const TopoDS_Shape& s)
{
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}
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

// ─── 1. Lifter cut removes volume ────────────────────────────────────────
TEST(SkillLifterMechanism, ApplyRemovesVolume)
{
    auto stock = skill::createCuboidStock(40.0, 30.0, 20.0);
    const double volBefore = volumeOf(stock->shape());
    const int fid = findTopFace(*stock);
    ASSERT_GE(fid, 0);

    skill::lifter_mechanism::Input in;
    in.face_id          = fid;
    in.base_x_mm        = 20.0;
    in.base_y_mm        = 15.0;
    in.lifter_angle_deg = 8.0;
    in.lifter_width_mm  = 6.0;
    in.lifter_height_mm = 4.0;
    in.lift_stroke_mm   = 5.0;

    auto out = skill::lifter_mechanism::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    const double volAfter = volumeOf(out.workpiece->shape());
    EXPECT_LT(volAfter, volBefore);
    EXPECT_GT(volBefore - volAfter, 1.0);
}

// ─── 2. DFM rejects bad angle ────────────────────────────────────────────
TEST(SkillLifterMechanism, ValidateRejectsBadAngle)
{
    auto stock = skill::createCuboidStock(40.0, 30.0, 20.0);
    const int fid = findTopFace(*stock);
    ASSERT_GE(fid, 0);

    skill::lifter_mechanism::Input in;
    in.face_id          = fid;
    in.base_x_mm        = 20.0;
    in.base_y_mm        = 15.0;
    in.lifter_angle_deg = 20.0;          // > 15
    in.lifter_width_mm  = 6.0;
    in.lifter_height_mm = 4.0;
    in.lift_stroke_mm   = 5.0;

    auto r = skill::lifter_mechanism::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool sawCode = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-LIFTER-ANGLE") { sawCode = true; break; }
    EXPECT_TRUE(sawCode);
    EXPECT_THROW(skill::lifter_mechanism::apply(*stock, in),
                 skill::SkillError);
}

// ─── 3. DFM rejects tiny stroke ──────────────────────────────────────────
TEST(SkillLifterMechanism, ValidateRejectsTinyStroke)
{
    auto stock = skill::createCuboidStock(40.0, 30.0, 20.0);
    const int fid = findTopFace(*stock);
    ASSERT_GE(fid, 0);

    skill::lifter_mechanism::Input in;
    in.face_id          = fid;
    in.base_x_mm        = 20.0;
    in.base_y_mm        = 15.0;
    in.lifter_angle_deg = 5.0;
    in.lifter_width_mm  = 6.0;
    in.lifter_height_mm = 4.0;
    in.lift_stroke_mm   = 0.2;           // < 0.5

    auto r = skill::lifter_mechanism::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool sawCode = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-LIFTER-CLEAR") { sawCode = true; break; }
    EXPECT_TRUE(sawCode);
}

// ─── 4. Signature records compound + kind ────────────────────────────────
TEST(SkillLifterMechanism, SignatureRecordsCompoundKind)
{
    auto stock = skill::createCuboidStock(40.0, 30.0, 20.0);
    const int fid = findTopFace(*stock);
    ASSERT_GE(fid, 0);

    skill::lifter_mechanism::Input in;
    in.face_id          = fid;
    in.base_x_mm        = 20.0;
    in.base_y_mm        = 15.0;
    in.lifter_angle_deg = 5.0;
    in.lifter_width_mm  = 6.0;
    in.lifter_height_mm = 4.0;
    in.lift_stroke_mm   = 5.0;
    auto out = skill::lifter_mechanism::apply(*stock, in);
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("lifter_mechanism"));
    EXPECT_TRUE(out.signature.pattern["is_compound"].get<bool>());
    EXPECT_EQ(out.signature.pattern["mold_feature_type"].get<std::string>(),
              std::string("lifter_channel"));
    EXPECT_GT(out.signature.pattern["derived_volume_removed"].get<double>(), 0.0);
}

// ─── 5. Recognize replays history ────────────────────────────────────────
TEST(SkillLifterMechanism, RecognizeReplaysHistory)
{
    auto stock = skill::createCuboidStock(40.0, 30.0, 20.0);
    const int fid = findTopFace(*stock);
    ASSERT_GE(fid, 0);

    skill::lifter_mechanism::Input in;
    in.face_id          = fid;
    in.base_x_mm        = 20.0;
    in.base_y_mm        = 15.0;
    in.lifter_angle_deg = 7.5;
    in.lifter_width_mm  = 6.0;
    in.lifter_height_mm = 4.0;
    in.lift_stroke_mm   = 5.0;
    auto out = skill::lifter_mechanism::apply(*stock, in);

    auto cands = skill::lifter_mechanism::recognize(*out.workpiece);
    ASSERT_GE(cands.size(), 1u);
    EXPECT_GE(cands.front().confidence, 0.9);
    EXPECT_NEAR(cands.front().recovered_params["lifter_angle_deg"].get<double>(),
                7.5, 0.01);
}
