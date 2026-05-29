// @lat: [[process/test-strategy#compound bearing seat]]
//
// sealed_bearing_seat_with_shield_relief — compound: bore + outer-race seat
// + 2 shield-relief grooves + chamfer.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/sealed_bearing_seat_with_shield_relief.hpp"

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
}  // namespace

// ─── 1. Apply produces valid shape with multiple sub-features ─────────────
TEST(SkillSealedBearingSeatShieldRelief, ApplyProducesValidShapeWithMultipleSubFeatures)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 20.0);

    skill::sealed_bearing_seat_with_shield_relief::Input in;
    in.entry_face            = skill::FaceByNormal{ gp_Dir(0, 0, 1), 5.0, "largest" };
    in.position_x_mm         = 30.0;
    in.position_y_mm         = 30.0;
    in.axis_dir              = gp_Dir(0, 0, -1);
    in.outer_dia_mm          = 22.0;
    in.shield_relief_dia_mm  = 23.0;     // 0.5 mm radial relief
    in.seat_depth_mm         = 10.0;

    const double v0 = volumeOf(stock->shape());
    const int    f0 = stock->faceCount();

    auto out = skill::sealed_bearing_seat_with_shield_relief::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    const double v1 = volumeOf(out.workpiece->shape());
    const int    f1 = out.workpiece->faceCount();

    // outer bore volume ≈ π × 11² × 10 ≈ 3801
    const double boreVol = M_PI * 11.0 * 11.0 * 10.0;
    const double removed = v0 - v1;
    EXPECT_GT(removed, boreVol * 0.95);
    EXPECT_LT(removed, boreVol * 1.5);

    EXPECT_GT(f1, f0);
}

// ─── 2. Validate rejects shield_relief_dia ≤ outer_dia ────────────────────
TEST(SkillSealedBearingSeatShieldRelief, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 20.0);
    skill::sealed_bearing_seat_with_shield_relief::Input in;
    in.entry_face           = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm        = 30.0;
    in.position_y_mm        = 30.0;
    in.outer_dia_mm         = 22.0;
    in.shield_relief_dia_mm = 21.0;   // invalid (< outer_dia)
    in.seat_depth_mm        = 10.0;

    auto r = skill::sealed_bearing_seat_with_shield_relief::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-SHIELD-DIA") { found = true; break; }
    EXPECT_TRUE(found);

    EXPECT_THROW(skill::sealed_bearing_seat_with_shield_relief::apply(*stock, in),
                 skill::SkillError);
}

// ─── 3. Signature records compound kind + 5 sub-features ─────────────────
TEST(SkillSealedBearingSeatShieldRelief, SignatureRecordsCompoundKind)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 20.0);
    skill::sealed_bearing_seat_with_shield_relief::Input in;
    in.entry_face           = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm        = 30.0;
    in.position_y_mm        = 30.0;
    in.outer_dia_mm         = 22.0;
    in.shield_relief_dia_mm = 23.0;
    in.seat_depth_mm        = 10.0;
    auto out = skill::sealed_bearing_seat_with_shield_relief::apply(*stock, in);

    EXPECT_TRUE(out.signature.pattern.at("is_compound").get<bool>());
    EXPECT_EQ(out.signature.pattern.at("subfeature_count").get<int>(), 5);
    EXPECT_NEAR(out.signature.pattern.at("relief_radial_depth_mm").get<double>(),
                0.5, 1e-9);
}

// ─── 4. Recognize via metadata replay ─────────────────────────────────────
TEST(SkillSealedBearingSeatShieldRelief, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 20.0);
    skill::sealed_bearing_seat_with_shield_relief::Input in;
    in.entry_face           = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm        = 30.0;
    in.position_y_mm        = 30.0;
    in.outer_dia_mm         = 22.0;
    in.shield_relief_dia_mm = 23.0;
    in.seat_depth_mm        = 10.0;
    auto out = skill::sealed_bearing_seat_with_shield_relief::apply(*stock, in);

    auto cands = skill::sealed_bearing_seat_with_shield_relief::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-9);
    EXPECT_NEAR(cands[0].recovered_params["shield_relief_dia_mm"].get<double>(),
                23.0, 1e-6);
}

// ─── 5. Relief inner Ø < outer Ø (relief sits OUTSIDE the bore wall) ─────
TEST(SkillSealedBearingSeatShieldRelief, ReliefDiaGreaterThanBoreDia)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 20.0);
    skill::sealed_bearing_seat_with_shield_relief::Input in;
    in.entry_face           = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm        = 30.0;
    in.position_y_mm        = 30.0;
    in.outer_dia_mm         = 22.0;
    in.shield_relief_dia_mm = 23.0;
    in.seat_depth_mm        = 10.0;
    auto out = skill::sealed_bearing_seat_with_shield_relief::apply(*stock, in);

    const double relief_dia = out.signature.pattern.at("shield_relief_dia_mm").get<double>();
    const double outer_dia  = out.signature.pattern.at("outer_dia_mm").get<double>();
    EXPECT_GT(relief_dia, outer_dia);
}
