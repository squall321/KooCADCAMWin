// @lat: [[process/test-strategy#compound macro]]
//
// internal_water_jacket — helical jacket groove + inlet + outlet ports.
//
// Test cases:
//   1. apply produces valid shape; volume removed.
//   2. validate rejects pitch < 2 × groove_w.
//   3. signature carries is_compound + subfeature_count == 3.
//   4. recognize replays metadata at confidence 1.0.
//   5. derived turn_count >= 1 and wall_remaining > 0.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/internal_water_jacket.hpp"

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

namespace {
skill::internal_water_jacket::Input goodInput()
{
    skill::internal_water_jacket::Input in;
    in.reference_face       = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.bore_axis            = gp_Dir(0, 0, 1);
    in.bore_origin_x_mm     = 25.0;
    in.bore_origin_y_mm     = 25.0;
    in.bore_axial_start_mm  = 5.0;
    in.bore_inner_r_mm      = 8.0;
    in.outer_r_mm           = 20.0;
    in.jacket_length_mm     = 30.0;
    in.jacket_pitch_mm      = 6.0;
    in.groove_w_mm          = 2.0;
    in.groove_depth_mm      = 2.5;
    in.port_dia_mm          = 3.0;
    in.inlet_axial_pos_mm   = 3.0;
    in.outlet_axial_pos_mm  = 27.0;
    in.inlet_clock_deg      = 0.0;
    in.outlet_clock_deg     = 180.0;
    return in;
}
}  // namespace

// ─── 1. apply removes volume & changes face count ─────────────────────────
TEST(SkillInternalWaterJacket, ApplyProducesValidShapeWithMultipleSubFeatures)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 50.0);
    const double v0 = volumeOf(stock->shape());
    const int    f0 = stock->faceCount();

    auto in  = goodInput();
    auto out = skill::internal_water_jacket::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    const double v1 = volumeOf(out.workpiece->shape());
    EXPECT_GT(v0 - v1, 0.0);
    EXPECT_NE(out.workpiece->faceCount(), f0);
}

// ─── 2. validate rejects bad input (pitch too small) ──────────────────────
TEST(SkillInternalWaterJacket, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 50.0);

    auto in = goodInput();
    in.jacket_pitch_mm = 1.0;     // < 2 × groove_w (= 4.0) → overlap
    auto r = skill::internal_water_jacket::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-JACKET-PITCH") { found = true; break; }
    EXPECT_TRUE(found);

    EXPECT_THROW(skill::internal_water_jacket::apply(*stock, in), skill::SkillError);
}

// ─── 3. Signature records compound kind ────────────────────────────────────
TEST(SkillInternalWaterJacket, SignatureRecordsCompoundKind)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 50.0);

    auto out = skill::internal_water_jacket::apply(*stock, goodInput());
    EXPECT_EQ(out.signature.skill_id, std::string("internal_water_jacket"));
    EXPECT_TRUE(out.signature.pattern.at("is_compound").get<bool>());
    EXPECT_GE(out.signature.pattern.at("subfeature_count").get<int>(), 2);
    EXPECT_EQ(out.signature.pattern.at("subfeature_count").get<int>(), 3);
}

// ─── 4. Recognize is metadata replay (confidence 1.0) ─────────────────────
TEST(SkillInternalWaterJacket, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 50.0);
    auto out   = skill::internal_water_jacket::apply(*stock, goodInput());

    auto cands = skill::internal_water_jacket::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands.front().confidence, 1.0, 1e-9);
    EXPECT_NEAR(cands.front().recovered_params["jacket_pitch_mm"].get<double>(),
                6.0, 1e-6);
}

// ─── 5. Derived turn_count >= 1 and wall_remaining > 0 ────────────────────
TEST(SkillInternalWaterJacket, DerivedTurnCountAndWallRemaining)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 50.0);
    auto in    = goodInput();
    auto out   = skill::internal_water_jacket::apply(*stock, in);

    // jacket_length=30, pitch=6 → ≥ 5 turns.
    EXPECT_GE(out.signature.pattern.at("turn_count").get<int>(), 5);

    // outer_r=20, inner_r=8, depth=2.5 → wall_remaining = 12 - 2.5 = 9.5.
    EXPECT_NEAR(out.signature.pattern.at("wall_remaining_mm").get<double>(),
                9.5, 1e-6);
    EXPECT_GT(out.signature.pattern.at("wall_remaining_mm").get<double>(), 0.0);
}
