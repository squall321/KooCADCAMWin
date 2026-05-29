// @lat: [[process/test-strategy#skill compound feature]]
//
// gas_strut_hinge_compound — REAL geometric test, multi-cut.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/gas_strut_hinge_compound.hpp"

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

// ─── Test 1: apply produces real shape, volume delta ─────────────────────
TEST(SkillGasStrutHingeCompound, ApplyProducesRealGeometry)
{
    // Bracket stock 60 × 30 × 8 mm
    auto stock = skill::createCuboidStock(60.0, 30.0, 8.0);
    ASSERT_FALSE(stock->shape().IsNull());

    skill::gas_strut_hinge_compound::Input in;
    in.entry_face              = skill::FaceByNormal{ gp_Dir(0, 0, 1), 5.0, "largest" };
    in.frame_pivot_x_mm        = 12.0;
    in.frame_pivot_y_mm        = 15.0;
    in.door_pivot_x_mm         = 48.0;
    in.door_pivot_y_mm         = 15.0;
    in.bracket_thickness_mm    = 8.0;
    in.pivot_bore_dia_mm       = 6.0;
    in.clevis_pocket_length_mm = 14.0;
    in.clevis_pocket_width_mm  = 8.0;
    in.clevis_pocket_depth_mm  = 5.0;
    in.ball_groove_inner_dia_mm = 9.0;
    in.ball_groove_outer_dia_mm = 11.0;
    in.ball_groove_depth_mm     = 1.5;
    in.lightening_hole_dia_mm   = 4.0;

    auto out = skill::gas_strut_hinge_compound::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    const double pR = in.pivot_bore_dia_mm / 2.0;
    const double lR = in.lightening_hole_dia_mm / 2.0;
    const double pivotV = 2.0 * M_PI * pR * pR * in.bracket_thickness_mm;
    const double clevisV = in.clevis_pocket_length_mm * in.clevis_pocket_width_mm
                           * in.clevis_pocket_depth_mm;
    const double lightV = M_PI * lR * lR * in.bracket_thickness_mm;
    const double expected = pivotV + clevisV + lightV;
    const double removed = volumeOf(stock->shape())
                           - volumeOf(out.workpiece->shape());
    EXPECT_NEAR(removed, expected, expected * 0.25);

    EXPECT_GT(out.workpiece->faceCount(), stock->faceCount() + 4);
}

// ─── Test 2: validate rejects bad input ──────────────────────────────────
TEST(SkillGasStrutHingeCompound, ValidateRejects)
{
    auto stock = skill::createCuboidStock(60.0, 30.0, 8.0);
    skill::gas_strut_hinge_compound::Input in;
    in.entry_face              = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.frame_pivot_x_mm        = 12.0; in.frame_pivot_y_mm = 15.0;
    in.door_pivot_x_mm         = 48.0; in.door_pivot_y_mm  = 15.0;
    in.bracket_thickness_mm    = 8.0;
    in.pivot_bore_dia_mm       = 1.5;       // < 3 — DFM-GAS-PIVOT
    in.clevis_pocket_length_mm = 14.0;
    in.clevis_pocket_width_mm  = 8.0;
    in.clevis_pocket_depth_mm  = 5.0;
    in.ball_groove_inner_dia_mm = 9.0;
    in.ball_groove_outer_dia_mm = 11.0;
    in.ball_groove_depth_mm     = 1.5;
    in.lightening_hole_dia_mm   = 4.0;

    auto r = skill::gas_strut_hinge_compound::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_THROW(skill::gas_strut_hinge_compound::apply(*stock, in),
                 skill::SkillError);
}

// ─── Test 3: signature compound + round-trip ─────────────────────────────
TEST(SkillGasStrutHingeCompound, SignatureCompound)
{
    auto stock = skill::createCuboidStock(60.0, 30.0, 8.0);
    skill::gas_strut_hinge_compound::Input in;
    in.entry_face              = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.frame_pivot_x_mm        = 12.0; in.frame_pivot_y_mm = 15.0;
    in.door_pivot_x_mm         = 48.0; in.door_pivot_y_mm  = 15.0;
    in.bracket_thickness_mm    = 8.0;
    in.pivot_bore_dia_mm       = 6.0;
    in.clevis_pocket_length_mm = 14.0;
    in.clevis_pocket_width_mm  = 8.0;
    in.clevis_pocket_depth_mm  = 5.0;
    in.ball_groove_inner_dia_mm = 9.0;
    in.ball_groove_outer_dia_mm = 11.0;
    in.ball_groove_depth_mm     = 1.5;
    in.lightening_hole_dia_mm   = 4.0;

    auto out = skill::gas_strut_hinge_compound::apply(*stock, in);
    EXPECT_TRUE(out.signature.pattern["is_compound"].get<bool>());
    EXPECT_EQ(out.signature.pattern["subfeature_count"].get<int>(), 5);
    EXPECT_GE(out.signature.pattern["subfeature_count"].get<int>(), 2);

    EXPECT_NEAR(out.signature.params["pivot_bore_dia_mm"].get<double>(),
                in.pivot_bore_dia_mm, 1e-6);
    EXPECT_NEAR(out.signature.params["bracket_thickness_mm"].get<double>(),
                in.bracket_thickness_mm, 1e-6);
}

// ─── Test 4: recognize ───────────────────────────────────────────────────
TEST(SkillGasStrutHingeCompound, RecognizeFinds)
{
    auto stock = skill::createCuboidStock(60.0, 30.0, 8.0);
    skill::gas_strut_hinge_compound::Input in;
    in.entry_face              = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.frame_pivot_x_mm        = 12.0; in.frame_pivot_y_mm = 15.0;
    in.door_pivot_x_mm         = 48.0; in.door_pivot_y_mm  = 15.0;
    in.bracket_thickness_mm    = 8.0;
    in.pivot_bore_dia_mm       = 6.0;
    in.clevis_pocket_length_mm = 14.0;
    in.clevis_pocket_width_mm  = 8.0;
    in.clevis_pocket_depth_mm  = 5.0;
    in.ball_groove_inner_dia_mm = 9.0;
    in.ball_groove_outer_dia_mm = 11.0;
    in.ball_groove_depth_mm     = 1.5;
    in.lightening_hole_dia_mm   = 4.0;

    auto out = skill::gas_strut_hinge_compound::apply(*stock, in);
    auto cands = skill::gas_strut_hinge_compound::recognize(*out.workpiece);
    ASSERT_FALSE(cands.empty());
    bool sawConfident = false;
    for (const auto& c : cands) if (c.confidence > 0.5) sawConfident = true;
    EXPECT_TRUE(sawConfident);
}

// ─── Test 5: feature-specific — pivot spacing recovered ───────────────────
TEST(SkillGasStrutHingeCompound, PivotSpacingRecovered)
{
    auto stock = skill::createCuboidStock(80.0, 30.0, 8.0);
    skill::gas_strut_hinge_compound::Input in;
    in.entry_face              = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.frame_pivot_x_mm        = 10.0; in.frame_pivot_y_mm = 15.0;
    in.door_pivot_x_mm         = 65.0; in.door_pivot_y_mm  = 15.0;   // spacing = 55
    in.bracket_thickness_mm    = 8.0;
    in.pivot_bore_dia_mm       = 6.0;
    in.clevis_pocket_length_mm = 14.0;
    in.clevis_pocket_width_mm  = 8.0;
    in.clevis_pocket_depth_mm  = 5.0;
    in.ball_groove_inner_dia_mm = 9.0;
    in.ball_groove_outer_dia_mm = 11.0;
    in.ball_groove_depth_mm     = 1.5;
    in.lightening_hole_dia_mm   = 4.0;

    auto out = skill::gas_strut_hinge_compound::apply(*stock, in);
    auto cands = skill::gas_strut_hinge_compound::recognize(*out.workpiece);

    bool checked = false;
    for (const auto& c : cands) {
        if (c.matched_geometry.contains("pivot_spacing")) {
            const double sep = c.matched_geometry["pivot_spacing"].get<double>();
            EXPECT_NEAR(sep, 55.0, 1.0);
            checked = true;
            break;
        }
    }
    EXPECT_TRUE(checked);
}
