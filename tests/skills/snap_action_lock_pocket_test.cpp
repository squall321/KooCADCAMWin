// @lat: [[process/test-strategy#skill compound feature]]
//
// snap_action_lock_pocket — REAL geometric test, multi-cut compound.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/snap_action_lock_pocket.hpp"

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

// ─── Test 1: real geometry, volume delta ─────────────────────────────────
TEST(SkillSnapActionLockPocket, ApplyProducesRealGeometry)
{
    // Big stock 60 × 60 × 30 mm — needs depth for the lock body bore
    auto stock = skill::createCuboidStock(60.0, 60.0, 30.0);
    ASSERT_FALSE(stock->shape().IsNull());

    skill::snap_action_lock_pocket::Input in;
    in.entry_face              = skill::FaceByNormal{ gp_Dir(0, 0, 1), 5.0, "largest" };
    in.position_x_mm           = 30.0;
    in.position_y_mm           = 30.0;
    in.axis_dir                = gp_Dir(0, 0, -1);
    in.lock_body_dia_mm        = 19.0;
    in.lock_body_depth_mm      = 22.0;
    in.key_counterbore_dia_mm  = 24.0;
    in.key_counterbore_depth_mm = 2.0;
    in.cam_slot_length_mm      = 25.0;
    in.cam_slot_width_mm       = 4.0;
    in.cam_slot_depth_offset_mm = 18.0;
    in.spring_pocket_dia_mm    = 2.5;
    in.spring_pocket_offset_mm = 5.0;
    in.spring_pocket_depth_mm  = 6.0;

    auto out = skill::snap_action_lock_pocket::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    // Expected ≈ lock body cyl + counterbore annulus
    const double bodyR = in.lock_body_dia_mm / 2.0;
    const double kbR   = in.key_counterbore_dia_mm / 2.0;
    const double bodyV = M_PI * bodyR * bodyR * in.lock_body_depth_mm;
    const double kbV   = M_PI * (kbR * kbR - bodyR * bodyR)
                              * in.key_counterbore_depth_mm;
    const double approx = bodyV + kbV;
    const double removed = volumeOf(stock->shape())
                           - volumeOf(out.workpiece->shape());
    // Plus cam slot & spring pocket — those add more.  Allow ±20% widely.
    EXPECT_GT(removed, approx * 0.8);
    EXPECT_LT(removed, approx * 1.5);

    EXPECT_GT(out.workpiece->faceCount(), stock->faceCount() + 2);
}

// ─── Test 2: validate rejects bad input ──────────────────────────────────
TEST(SkillSnapActionLockPocket, ValidateRejects)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 30.0);
    skill::snap_action_lock_pocket::Input in;
    in.entry_face              = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm           = 30.0; in.position_y_mm = 30.0;
    in.lock_body_dia_mm        = 4.0;      // < 6 — DFM-SNAPLOCK-DIA
    in.lock_body_depth_mm      = 22.0;
    in.key_counterbore_dia_mm  = 24.0;
    in.key_counterbore_depth_mm = 2.0;
    in.cam_slot_length_mm      = 25.0;
    in.cam_slot_width_mm       = 4.0;
    in.cam_slot_depth_offset_mm = 18.0;
    in.spring_pocket_dia_mm    = 2.5;
    in.spring_pocket_offset_mm = 5.0;
    in.spring_pocket_depth_mm  = 6.0;

    auto r = skill::snap_action_lock_pocket::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_THROW(skill::snap_action_lock_pocket::apply(*stock, in), skill::SkillError);
}

// ─── Test 3: signature compound + round-trip ─────────────────────────────
TEST(SkillSnapActionLockPocket, SignatureIsCompound)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 30.0);
    skill::snap_action_lock_pocket::Input in;
    in.entry_face              = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm           = 30.0; in.position_y_mm = 30.0;
    in.lock_body_dia_mm        = 16.0;
    in.lock_body_depth_mm      = 20.0;
    in.key_counterbore_dia_mm  = 20.0;
    in.key_counterbore_depth_mm = 2.0;
    in.cam_slot_length_mm      = 22.0;
    in.cam_slot_width_mm       = 4.0;
    in.cam_slot_depth_offset_mm = 16.0;
    in.spring_pocket_dia_mm    = 2.5;
    in.spring_pocket_offset_mm = 4.0;
    in.spring_pocket_depth_mm  = 6.0;

    auto out = skill::snap_action_lock_pocket::apply(*stock, in);
    EXPECT_TRUE(out.signature.pattern["is_compound"].get<bool>());
    EXPECT_EQ(out.signature.pattern["subfeature_count"].get<int>(), 4);
    EXPECT_GE(out.signature.pattern["subfeature_count"].get<int>(), 2);
    EXPECT_NEAR(out.signature.params["lock_body_dia_mm"].get<double>(),
                in.lock_body_dia_mm, 1e-6);
    EXPECT_NEAR(out.signature.params["key_counterbore_dia_mm"].get<double>(),
                in.key_counterbore_dia_mm, 1e-6);
}

// ─── Test 4: recognize ≥ 1 candidate ─────────────────────────────────────
TEST(SkillSnapActionLockPocket, RecognizeFinds)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 30.0);
    skill::snap_action_lock_pocket::Input in;
    in.entry_face              = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm           = 30.0; in.position_y_mm = 30.0;
    in.lock_body_dia_mm        = 19.0;
    in.lock_body_depth_mm      = 22.0;
    in.key_counterbore_dia_mm  = 24.0;
    in.key_counterbore_depth_mm = 2.0;
    in.cam_slot_length_mm      = 25.0;
    in.cam_slot_width_mm       = 4.0;
    in.cam_slot_depth_offset_mm = 18.0;
    in.spring_pocket_dia_mm    = 2.5;
    in.spring_pocket_offset_mm = 6.0;
    in.spring_pocket_depth_mm  = 6.0;

    auto out = skill::snap_action_lock_pocket::apply(*stock, in);
    auto cands = skill::snap_action_lock_pocket::recognize(*out.workpiece);
    ASSERT_FALSE(cands.empty());
    bool sawConfident = false;
    for (const auto& c : cands) if (c.confidence > 0.5) sawConfident = true;
    EXPECT_TRUE(sawConfident);
}

// ─── Test 5: feature-specific — concentric pair detection ────────────────
TEST(SkillSnapActionLockPocket, GeometricRecoversBodyAndCounterbore)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 30.0);
    skill::snap_action_lock_pocket::Input in;
    in.entry_face              = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm           = 30.0; in.position_y_mm = 30.0;
    in.lock_body_dia_mm        = 19.0;
    in.lock_body_depth_mm      = 22.0;
    in.key_counterbore_dia_mm  = 24.0;
    in.key_counterbore_depth_mm = 2.0;
    in.cam_slot_length_mm      = 25.0;
    in.cam_slot_width_mm       = 4.0;
    in.cam_slot_depth_offset_mm = 18.0;
    in.spring_pocket_dia_mm    = 2.5;
    in.spring_pocket_offset_mm = 6.0;
    in.spring_pocket_depth_mm  = 6.0;

    auto out = skill::snap_action_lock_pocket::apply(*stock, in);
    auto cands = skill::snap_action_lock_pocket::recognize(*out.workpiece);

    // Find a geometric candidate (not metadata)
    bool checked = false;
    for (const auto& c : cands) {
        if (c.recovered_params.contains("lock_body_dia_mm")
            && c.recovered_params.contains("key_counterbore_dia_mm")) {
            const double bodyD = c.recovered_params["lock_body_dia_mm"].get<double>();
            const double kbD   = c.recovered_params["key_counterbore_dia_mm"].get<double>();
            EXPECT_GT(kbD, bodyD)
                << "counterbore must be larger than body";
            EXPECT_NEAR(bodyD, in.lock_body_dia_mm, 0.5);
            checked = true;
            break;
        }
    }
    EXPECT_TRUE(checked);
}
