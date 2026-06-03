// @lat: [[process/test-strategy#camera_island_step]]
//
// camera_island_step — phone camera bump compound (5 tests).
//
// Cases:
//   1. ApplyAddsNetVolume — Δvol > 0 (fuses > cut), ≈ step_vol − lens_vol ±25%
//   2. ValidateRejectsZeroDimensions
//   3. ValidateRejectsLensTooBig
//   4. SignatureCarriesChamferedStepFlag
//   5. RecognizeReplaysHistory

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/camera_island_step.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

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

skill::camera_island_step::Input good(int faceId)
{
    skill::camera_island_step::Input in;
    in.face_id              = faceId;
    in.camera_center_x_mm   = 30.0;
    in.camera_center_y_mm   = 25.0;
    in.outer_island_w_mm    = 20.0;
    in.outer_island_h_mm    = 14.0;
    in.raised_height_mm     = 1.5;
    in.inner_lens_dia_mm    = 6.0;
    in.inner_lens_depth_mm  = 0.8;
    return in;
}

}  // namespace

// ─── 1. Net additive: Δvol ≈ step_vol − lens_vol ±25% ───────────────────
TEST(SkillCameraIslandStep, ApplyAddsNetVolume)
{
    auto stock = skill::createCuboidStock(60.0, 50.0, 10.0);
    const int topId = findTopFace(*stock);
    ASSERT_GE(topId, 0);

    auto in = good(topId);
    const double v0 = volumeOf(stock->shape());
    auto out = skill::camera_island_step::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    const double v1 = volumeOf(out.workpiece->shape());
    const double net = v1 - v0;

    const double vStep = in.outer_island_w_mm * in.outer_island_h_mm *
                          in.raised_height_mm;
    const double rL = in.inner_lens_dia_mm / 2.0;
    const double vLens = M_PI * rL * rL * in.inner_lens_depth_mm;

    EXPECT_GT(net, 0.0);
    // Within ±25% of (step − lens), with ledge giving ~10% headroom.
    const double expected = vStep - vLens;
    EXPECT_NEAR(net, expected, expected * 0.30)
        << "net=" << net << " step=" << vStep << " lens=" << vLens;
}

// ─── 2. Validate rejects zero dim ────────────────────────────────────────
TEST(SkillCameraIslandStep, ValidateRejectsZeroDimensions)
{
    auto stock = skill::createCuboidStock(60.0, 50.0, 10.0);
    const int topId = findTopFace(*stock);
    auto in = good(topId);
    in.raised_height_mm = 0.0;
    auto r = skill::camera_island_step::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_THROW(skill::camera_island_step::apply(*stock, in),
                 skill::SkillError);
}

// ─── 3. Validate rejects lens ≥ island ───────────────────────────────────
TEST(SkillCameraIslandStep, ValidateRejectsLensTooBig)
{
    auto stock = skill::createCuboidStock(60.0, 50.0, 10.0);
    const int topId = findTopFace(*stock);
    auto in = good(topId);
    in.inner_lens_dia_mm = in.outer_island_w_mm + 1.0;  // lens > island
    auto r = skill::camera_island_step::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool foundCode = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-CAM-LENS") foundCode = true;
    EXPECT_TRUE(foundCode);
}

// ─── 4. Signature flags chamfered step + phone feature ───────────────────
TEST(SkillCameraIslandStep, SignatureCarriesChamferedStepFlag)
{
    auto stock = skill::createCuboidStock(60.0, 50.0, 10.0);
    const int topId = findTopFace(*stock);
    auto in = good(topId);
    auto out = skill::camera_island_step::apply(*stock, in);

    EXPECT_EQ(out.signature.skill_id, std::string("camera_island_step"));
    EXPECT_TRUE(out.signature.pattern["is_compound"].get<bool>());
    EXPECT_TRUE(out.signature.pattern["is_phone_feature"].get<bool>());
    EXPECT_TRUE(out.signature.pattern["has_chamfered_step"].get<bool>());
    EXPECT_EQ(out.signature.pattern["subfeature_count"].get<int>(), 4);
}

// ─── 5. Recognize replays history ────────────────────────────────────────
TEST(SkillCameraIslandStep, RecognizeReplaysHistory)
{
    auto stock = skill::createCuboidStock(60.0, 50.0, 10.0);
    const int topId = findTopFace(*stock);
    auto in = good(topId);
    auto out = skill::camera_island_step::apply(*stock, in);

    auto cands = skill::camera_island_step::recognize(*out.workpiece);
    ASSERT_GE(cands.size(), 1u);
    EXPECT_EQ(cands.front().skill_id, std::string("camera_island_step"));
    EXPECT_GE(cands.front().confidence, 0.9);
}
