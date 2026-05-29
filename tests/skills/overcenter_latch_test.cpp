// @lat: [[process/test-strategy#skill compound feature]]
//
// overcenter_latch — REAL geometric test, multi-cut compound.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/overcenter_latch.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

#include <cmath>

using namespace koocadcam;

namespace {
double volumeOf(const TopoDS_Shape& s) {
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}
}  // namespace

// ─── Test 1: apply produces real multi-feature shape ─────────────────────
TEST(SkillOvercenterLatch, ApplyProducesRealGeometry)
{
    auto stock = skill::createCuboidStock(80.0, 30.0, 12.0);
    ASSERT_FALSE(stock->shape().IsNull());

    skill::overcenter_latch::Input in;
    in.entry_face          = skill::FaceByNormal{ gp_Dir(0, 0, 1), 5.0, "largest" };
    in.position_x_mm       = 15.0;
    in.position_y_mm       = 15.0;
    in.axis_dir            = gp_Dir(0, 0, -1);
    in.lever_arm_length_mm = 40.0;
    in.lever_arm_width_mm  = 8.0;
    in.lever_pocket_depth_mm = 4.0;
    in.pivot_bore_dia_mm   = 3.0;
    in.pivot_bore_depth_mm = 8.0;
    in.cam_slot_offset_mm  = 12.0;
    in.cam_slot_length_mm  = 8.0;
    in.cam_slot_width_mm   = 3.0;
    in.cam_slot_depth_mm   = 3.0;
    in.retention_hook_length_mm = 5.0;
    in.retention_hook_width_mm  = 4.0;
    in.retention_hook_depth_mm  = 2.5;
    in.grip_notch_width_mm = 2.0;
    in.grip_notch_depth_mm = 1.0;

    auto out = skill::overcenter_latch::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    // Volume delta — sum of all 5 sub-cuts (note: hook overlaps lever pocket
    // so we under-subtract; expected upper bound used).
    const double pivR = in.pivot_bore_dia_mm / 2.0;
    const double leverV = in.lever_arm_length_mm * in.lever_arm_width_mm
                          * in.lever_pocket_depth_mm;
    const double camV   = in.cam_slot_length_mm * in.cam_slot_width_mm
                          * in.cam_slot_depth_mm;
    const double pivV   = M_PI * pivR * pivR * in.pivot_bore_depth_mm;
    const double notchV = in.grip_notch_width_mm * in.lever_arm_width_mm * 2.0
                          * in.grip_notch_depth_mm;
    const double expectedUpper = leverV + camV + pivV + notchV;
    const double removed = volumeOf(stock->shape()) - volumeOf(out.workpiece->shape());
    EXPECT_GT(removed, leverV * 0.5);     // at least half lever pocket
    EXPECT_LT(removed, expectedUpper * 1.3);

    EXPECT_GT(out.workpiece->faceCount(), stock->faceCount());
}

// ─── Test 2: validate rejects bad input ──────────────────────────────────
TEST(SkillOvercenterLatch, ValidateRejectsBad)
{
    auto stock = skill::createCuboidStock(80.0, 30.0, 12.0);

    skill::overcenter_latch::Input in;
    in.entry_face          = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm       = 15.0; in.position_y_mm = 15.0;
    in.lever_arm_length_mm = 3.0;     // < 6 mm — DFM-OVERCENTER-ARM
    in.lever_arm_width_mm  = 8.0;
    in.lever_pocket_depth_mm = 4.0;
    in.pivot_bore_dia_mm   = 3.0;
    in.pivot_bore_depth_mm = 8.0;
    in.cam_slot_length_mm  = 8.0;
    in.cam_slot_width_mm   = 3.0;
    in.cam_slot_depth_mm   = 3.0;
    in.retention_hook_length_mm = 5.0;
    in.retention_hook_width_mm  = 4.0;
    in.retention_hook_depth_mm  = 2.5;
    in.grip_notch_width_mm = 2.0;
    in.grip_notch_depth_mm = 1.0;

    auto r = skill::overcenter_latch::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_THROW(skill::overcenter_latch::apply(*stock, in), skill::SkillError);
}

// ─── Test 3: signature is_compound + subfeature_count + param round-trip ─
TEST(SkillOvercenterLatch, SignatureIsCompound)
{
    auto stock = skill::createCuboidStock(80.0, 30.0, 12.0);
    skill::overcenter_latch::Input in;
    in.entry_face          = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm       = 15.0; in.position_y_mm = 15.0;
    in.lever_arm_length_mm = 30.0;
    in.lever_arm_width_mm  = 6.0;
    in.lever_pocket_depth_mm = 4.0;
    in.pivot_bore_dia_mm   = 3.0;
    in.pivot_bore_depth_mm = 8.0;
    in.cam_slot_offset_mm  = 10.0;
    in.cam_slot_length_mm  = 6.0;
    in.cam_slot_width_mm   = 2.5;
    in.cam_slot_depth_mm   = 3.0;
    in.retention_hook_length_mm = 4.0;
    in.retention_hook_width_mm  = 3.0;
    in.retention_hook_depth_mm  = 2.5;
    in.grip_notch_width_mm = 2.0;
    in.grip_notch_depth_mm = 1.0;

    auto out = skill::overcenter_latch::apply(*stock, in);
    EXPECT_TRUE(out.signature.pattern["is_compound"].get<bool>());
    EXPECT_EQ(out.signature.pattern["subfeature_count"].get<int>(), 5);
    EXPECT_GE(out.signature.pattern["subfeature_count"].get<int>(), 2);

    EXPECT_NEAR(out.signature.params["lever_arm_length_mm"].get<double>(),
                in.lever_arm_length_mm, 1e-6);
    EXPECT_NEAR(out.signature.params["pivot_bore_dia_mm"].get<double>(),
                in.pivot_bore_dia_mm, 1e-6);
}

// ─── Test 4: recognize returns >= 1 candidate ─────────────────────────────
TEST(SkillOvercenterLatch, RecognizeFinds)
{
    auto stock = skill::createCuboidStock(80.0, 30.0, 12.0);
    skill::overcenter_latch::Input in;
    in.entry_face          = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm       = 15.0; in.position_y_mm = 15.0;
    in.lever_arm_length_mm = 30.0;
    in.lever_arm_width_mm  = 6.0;
    in.lever_pocket_depth_mm = 4.0;
    in.pivot_bore_dia_mm   = 3.0;
    in.pivot_bore_depth_mm = 8.0;
    in.cam_slot_offset_mm  = 10.0;
    in.cam_slot_length_mm  = 6.0;
    in.cam_slot_width_mm   = 2.5;
    in.cam_slot_depth_mm   = 3.0;
    in.retention_hook_length_mm = 4.0;
    in.retention_hook_width_mm  = 3.0;
    in.retention_hook_depth_mm  = 2.5;
    in.grip_notch_width_mm = 2.0;
    in.grip_notch_depth_mm = 1.0;

    auto out = skill::overcenter_latch::apply(*stock, in);
    auto cands = skill::overcenter_latch::recognize(*out.workpiece);
    ASSERT_FALSE(cands.empty());
    bool sawConfident = false;
    for (const auto& c : cands) if (c.confidence > 0.5) sawConfident = true;
    EXPECT_TRUE(sawConfident);
}

// ─── Test 5: feature-specific — pivot bore dia recovered ─────────────────
TEST(SkillOvercenterLatch, PivotBoreDiaRecovered)
{
    auto stock = skill::createCuboidStock(80.0, 30.0, 12.0);
    skill::overcenter_latch::Input in;
    in.entry_face          = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm       = 20.0; in.position_y_mm = 15.0;
    in.lever_arm_length_mm = 30.0;
    in.lever_arm_width_mm  = 6.0;
    in.lever_pocket_depth_mm = 4.0;
    in.pivot_bore_dia_mm   = 4.0;        // distinct value to verify recovery
    in.pivot_bore_depth_mm = 8.0;
    in.cam_slot_offset_mm  = 10.0;
    in.cam_slot_length_mm  = 6.0;
    in.cam_slot_width_mm   = 2.5;
    in.cam_slot_depth_mm   = 3.0;
    in.retention_hook_length_mm = 4.0;
    in.retention_hook_width_mm  = 3.0;
    in.retention_hook_depth_mm  = 2.5;
    in.grip_notch_width_mm = 2.0;
    in.grip_notch_depth_mm = 1.0;

    auto out = skill::overcenter_latch::apply(*stock, in);
    auto cands = skill::overcenter_latch::recognize(*out.workpiece);

    bool checked = false;
    for (const auto& c : cands) {
        if (c.recovered_params.contains("pivot_bore_dia_mm")) {
            const double d = c.recovered_params["pivot_bore_dia_mm"].get<double>();
            EXPECT_NEAR(d, in.pivot_bore_dia_mm, in.pivot_bore_dia_mm * 0.10);
            checked = true;
            break;
        }
    }
    EXPECT_TRUE(checked);
}
