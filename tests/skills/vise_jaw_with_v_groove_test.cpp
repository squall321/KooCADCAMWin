// @lat: [[process/test-strategy#compound vise_jaw_with_v_groove]]
//
// Verifies REAL multi-cut geometry: V-groove triangular prism + per-hole
// pilot + counterbore.  Validates DIN 6346 angle range and ASME B5.41
// spacing.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/vise_jaw_with_v_groove.hpp"

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

// ─── 1. apply: V-groove + mount holes removed (real volume) ──────────────
TEST(SkillViseJawWithVGroove, ApplyRealMultiCutVolume)
{
    // Soft jaw: 120 mm long × 25 mm thick × 30 mm tall.
    // gp_Dir Y = thickness (into vise), Z = height.
    auto stock = skill::createCuboidStock(120.0, 25.0, 30.0);
    const int    stockFaces = stock->faceCount();
    const double stockVol   = volumeOf(stock->shape());

    skill::vise_jaw_with_v_groove::Input in;
    in.working_face        = skill::FaceByNormal{ gp_Dir(0, 1, 0) };  // +Y
    in.v_groove_angle_deg  = 90.0;
    in.v_groove_depth_mm   = 5.0;
    in.v_groove_z_mm       = 15.0;
    in.pilot_dia_mm        = 6.6;
    in.counterbore_dia_mm  = 10.0;
    in.counterbore_depth_mm = 6.0;
    in.mount_holes = {
        { 30.0,  8.0 },
        { 90.0,  8.0 },
    };

    auto out = skill::vise_jaw_with_v_groove::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    // V-groove triangle area:
    //   half-base = 5 · tan(45°) = 5
    //   tri area = 0.5 · 10 · 5 = 25
    //   length = 120
    //   v vol = 25 · 120 = 3000
    // Per hole:
    //   pilot Ø6.6 × 25 = π·3.3²·25 ≈ 855.3
    //   cb annulus (Ø10 − Ø6.6) × 6 = π·(25 − 10.89) · 6 ≈ 265.9
    //   per hole ≈ 1121.2 × 2 holes = 2242.4
    const double vVol = 0.5 * (2.0 * 5.0 * std::tan(45.0 * M_PI / 180.0)) * 5.0 * 120.0;
    const double rPilot = 3.3, rCB = 5.0;
    const double pilotVol = M_PI * rPilot * rPilot * 25.0;
    const double cbVol    = M_PI * (rCB * rCB - rPilot * rPilot) * 6.0;
    const double approx   = vVol + 2.0 * (pilotVol + cbVol);
    const double removed  = stockVol - volumeOf(out.workpiece->shape());
    EXPECT_NEAR(removed, approx, approx * 0.15);
    EXPECT_GT(out.workpiece->faceCount(), stockFaces);
}

// ─── 2. validate rejects bad angle/depth + apply throws ──────────────────
TEST(SkillViseJawWithVGroove, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(120.0, 25.0, 30.0);

    // (a) angle 30° (< 60°)
    {
        skill::vise_jaw_with_v_groove::Input in;
        in.working_face       = skill::FaceByNormal{ gp_Dir(0, 1, 0) };
        in.v_groove_angle_deg = 30.0;
        in.v_groove_depth_mm  = 5.0;
        in.v_groove_z_mm      = 15.0;
        in.pilot_dia_mm       = 6.6;
        in.counterbore_dia_mm = 10.0;
        in.counterbore_depth_mm = 6.0;
        in.mount_holes = { { 30.0, 8.0 } };
        auto r = skill::vise_jaw_with_v_groove::validate(*stock, in);
        EXPECT_FALSE(r.passed);
        bool found = false;
        for (const auto& f : r.findings)
            if (f.code == "DFM-V-ANGLE") { found = true; break; }
        EXPECT_TRUE(found);
        EXPECT_THROW(skill::vise_jaw_with_v_groove::apply(*stock, in),
                     skill::SkillError);
    }
    // (b) deep groove (> 0.4 · jawHeight)
    {
        skill::vise_jaw_with_v_groove::Input in;
        in.working_face       = skill::FaceByNormal{ gp_Dir(0, 1, 0) };
        in.v_groove_angle_deg = 90.0;
        in.v_groove_depth_mm  = 15.0;        // > 0.4 · 30 = 12
        in.v_groove_z_mm      = 15.0;
        in.pilot_dia_mm       = 6.6;
        in.counterbore_dia_mm = 10.0;
        in.counterbore_depth_mm = 6.0;
        in.mount_holes = { { 30.0, 8.0 } };
        auto r = skill::vise_jaw_with_v_groove::validate(*stock, in);
        EXPECT_FALSE(r.passed);
        bool found = false;
        for (const auto& f : r.findings)
            if (f.code == "DFM-V-DEPTH") { found = true; break; }
        EXPECT_TRUE(found);
    }
    // (c) no mount holes
    {
        skill::vise_jaw_with_v_groove::Input in;
        in.working_face       = skill::FaceByNormal{ gp_Dir(0, 1, 0) };
        in.v_groove_angle_deg = 90.0;
        in.v_groove_depth_mm  = 5.0;
        in.v_groove_z_mm      = 15.0;
        in.pilot_dia_mm       = 6.6;
        in.counterbore_dia_mm = 10.0;
        in.counterbore_depth_mm = 6.0;
        auto r = skill::vise_jaw_with_v_groove::validate(*stock, in);
        EXPECT_FALSE(r.passed);
    }
}

// ─── 3. signature compound + params round-trip ───────────────────────────
TEST(SkillViseJawWithVGroove, SignatureCompoundRoundTrip)
{
    auto stock = skill::createCuboidStock(120.0, 25.0, 30.0);

    skill::vise_jaw_with_v_groove::Input in;
    in.working_face       = skill::FaceByNormal{ gp_Dir(0, 1, 0) };
    in.v_groove_angle_deg = 90.0;
    in.v_groove_depth_mm  = 5.0;
    in.v_groove_z_mm      = 15.0;
    in.pilot_dia_mm       = 6.6;
    in.counterbore_dia_mm = 10.0;
    in.counterbore_depth_mm = 6.0;
    in.mount_holes = { { 30.0, 8.0 }, { 90.0, 8.0 } };

    auto out = skill::vise_jaw_with_v_groove::apply(*stock, in);
    EXPECT_TRUE(out.signature.pattern.at("is_compound").get<bool>());
    EXPECT_GE(out.signature.pattern.at("subfeature_count").get<int>(), 2);
    // 1 V-groove + 2 holes × (pilot + cb) = 1 + 4 = 5
    EXPECT_EQ(out.signature.pattern.at("subfeature_count").get<int>(), 5);
    EXPECT_NEAR(out.signature.params.at("v_groove_angle_deg").get<double>(),
                90.0, 1e-6);
    EXPECT_EQ(out.signature.params.at("mount_holes").size(),
              in.mount_holes.size());
}

// ─── 4. recognize: metadata replay confidence 1.0 ────────────────────────
TEST(SkillViseJawWithVGroove, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(120.0, 25.0, 30.0);

    skill::vise_jaw_with_v_groove::Input in;
    in.working_face       = skill::FaceByNormal{ gp_Dir(0, 1, 0) };
    in.v_groove_angle_deg = 90.0;
    in.v_groove_depth_mm  = 5.0;
    in.v_groove_z_mm      = 15.0;
    in.pilot_dia_mm       = 6.6;
    in.counterbore_dia_mm = 10.0;
    in.counterbore_depth_mm = 6.0;
    in.mount_holes = { { 30.0, 8.0 }, { 90.0, 8.0 } };

    auto out = skill::vise_jaw_with_v_groove::apply(*stock, in);
    auto cands = skill::vise_jaw_with_v_groove::recognize(*out.workpiece);
    ASSERT_FALSE(cands.empty());
    EXPECT_DOUBLE_EQ(cands.front().confidence, 1.0);
    EXPECT_EQ(cands.front().skill_id, std::string("vise_jaw_with_v_groove"));
}

// ─── 5. feature-specific: mount hole count matches and CB > pilot ───────
TEST(SkillViseJawWithVGroove, MountHoleCountAndCBGreaterThanPilot)
{
    auto stock = skill::createCuboidStock(150.0, 25.0, 30.0);

    skill::vise_jaw_with_v_groove::Input in;
    in.working_face       = skill::FaceByNormal{ gp_Dir(0, 1, 0) };
    in.v_groove_angle_deg = 90.0;
    in.v_groove_depth_mm  = 5.0;
    in.v_groove_z_mm      = 15.0;
    in.pilot_dia_mm       = 6.6;
    in.counterbore_dia_mm = 10.0;
    in.counterbore_depth_mm = 6.0;
    in.mount_holes = { { 25.0, 8.0 }, { 75.0, 8.0 }, { 125.0, 8.0 } };

    auto out = skill::vise_jaw_with_v_groove::apply(*stock, in);
    EXPECT_EQ(out.signature.pattern.at("mount_hole_count").get<int>(), 3);
    EXPECT_GT(out.signature.pattern.at("counterbore_dia_mm").get<double>(),
              out.signature.pattern.at("pilot_dia_mm").get<double>());
    EXPECT_EQ(out.signature.pattern.at("subfeature_count").get<int>(),
              1 + 2 * 3);
}
