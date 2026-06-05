// @lat: [[process/test-strategy#wind yaw_motor_mount_compound]]

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/yaw_motor_mount_compound.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

using namespace koocadcam;

namespace {
double volumeOf(const TopoDS_Shape& s) {
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}

int findTopFaceId(const skill::Workpiece& wp) {
    int best = -1;
    double bestZ = -1e9;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFacePlanar(i)) continue;
        const auto n = wp.faceNormal(i);
        if (n.Z() < 0.9) continue;
        const auto c = wp.faceCenter(i);
        if (c.Z() > bestZ) { bestZ = c.Z(); best = i; }
    }
    return best;
}

skill::yaw_motor_mount_compound::Input goodInput(const skill::Workpiece& wp) {
    skill::yaw_motor_mount_compound::Input in;
    in.face_id              = findTopFaceId(wp);
    in.center_x_mm          = 75.0;
    in.center_y_mm          = 75.0;
    in.mount_pcd_mm         = 100.0;
    in.mount_bolt_count     = 6;
    in.bolt_thread_size_key = "M10";
    in.vent_count           = 6;
    in.vent_slot_width_mm   = 6.0;
    in.vent_slot_length_mm  = 20.0;
    in.vent_slot_depth_mm   = 10.0;
    return in;
}
}  // namespace

// ─── 1. Apply cuts bolts + vents ────────────────────────────────────────
TEST(SkillYawMotorMount, ApplyRemovesMaterial)
{
    auto stock = skill::createCuboidStock(150.0, 150.0, 20.0);
    auto in    = goodInput(*stock);
    ASSERT_GE(in.face_id, 0);

    const double v0 = volumeOf(stock->shape());
    auto out = skill::yaw_motor_mount_compound::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    const double v1 = volumeOf(out.workpiece->shape());

    EXPECT_GT(v0 - v1, 0.0);
    EXPECT_EQ(out.signature.pattern.at("subfeature_count").get<int>(),
              in.mount_bolt_count + in.vent_count);
}

// ─── 2. Validate rejects mount_bolt_count <= 4 ──────────────────────────
TEST(SkillYawMotorMount, ValidateRejectsTooFewBolts)
{
    auto stock = skill::createCuboidStock(150.0, 150.0, 20.0);
    auto in    = goodInput(*stock);
    in.mount_bolt_count = 3;

    auto r = skill::yaw_motor_mount_compound::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-BOLT-COUNT") { found = true; break; }
    EXPECT_TRUE(found);
}

// ─── 3. Validate rejects unknown M-thread key ───────────────────────────
TEST(SkillYawMotorMount, ValidateRejectsBadThreadKey)
{
    auto stock = skill::createCuboidStock(150.0, 150.0, 20.0);
    auto in    = goodInput(*stock);
    in.bolt_thread_size_key = "M99";

    auto r = skill::yaw_motor_mount_compound::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-THREAD") { found = true; break; }
    EXPECT_TRUE(found);
}

// ─── 4. Validate rejects vent slot length exceeding per-vent arc ────────
TEST(SkillYawMotorMount, ValidateRejectsOverlappingVents)
{
    auto stock = skill::createCuboidStock(150.0, 150.0, 20.0);
    auto in    = goodInput(*stock);
    in.vent_slot_length_mm = 100.0;  // per-vent arc ~52 mm → overlap

    auto r = skill::yaw_motor_mount_compound::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-VENT-FIT") { found = true; break; }
    EXPECT_TRUE(found);
}

// ─── 5. Signature + Recognize round-trip ────────────────────────────────
TEST(SkillYawMotorMount, SignatureAndRecognize)
{
    auto stock = skill::createCuboidStock(150.0, 150.0, 20.0);
    auto in    = goodInput(*stock);
    auto out   = skill::yaw_motor_mount_compound::apply(*stock, in);

    EXPECT_EQ(out.signature.skill_id,
              std::string("yaw_motor_mount_compound"));
    EXPECT_TRUE(out.signature.pattern.at("is_compound").get<bool>());
    EXPECT_EQ(out.signature.pattern.at("wind_feature_type").get<std::string>(),
              std::string("yaw_motor_mount"));
    EXPECT_EQ(out.signature.pattern.at("mount_bolt_count").get<int>(),
              in.mount_bolt_count);
    EXPECT_EQ(out.signature.pattern.at("vent_count").get<int>(),
              in.vent_count);

    auto cands = skill::yaw_motor_mount_compound::recognize(*out.workpiece);
    ASSERT_FALSE(cands.empty());
    EXPECT_DOUBLE_EQ(cands[0].confidence, 1.0);
}
