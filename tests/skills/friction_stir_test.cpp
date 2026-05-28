// @lat: [[process/test-strategy#skill round-trip]]
//
// friction_stir skill — 6-case sweep covering the unique SUBTRACTIVE
// behavior (no filler is added — volume DECREASES).

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/friction_stir.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

#include <cmath>

using namespace koocadcam;

namespace {

double volumeOf(const TopoDS_Shape& s)
{
    GProp_GProps p;
    BRepGProp::VolumeProperties(s, p);
    return p.Mass();
}

}  // namespace

// ─── 1. Apply: REMOVES a thin trace — volume decreases ────────────────────
TEST(SkillFrictionStir, ApplyRemovesShallowTrace)
{
    auto stock = skill::createCuboidStock(120.0, 80.0, 10.0);
    const double volBefore = volumeOf(stock->shape());

    skill::friction_stir::Input in;
    in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1), 5.0, "largest" };
    in.start_x_mm = 20.0; in.start_y_mm = 40.0;
    in.end_x_mm   = 100.0; in.end_y_mm  = 40.0;
    in.tool_shoulder_dia_mm    = 12.0;
    in.tool_rotation_rpm       = 1500.0;
    in.traverse_speed_mm_per_s = 5.0;
    in.depth_mm                = 0.2;

    auto out = skill::friction_stir::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    const double volAfter = volumeOf(out.workpiece->shape());
    EXPECT_LT(volAfter, volBefore)
        << "friction_stir is SUBTRACTIVE — volume must decrease";

    // Removed volume ≈ stadium area × depth
    const double r = 6.0;
    const double L = 80.0;
    const double approxRemoved = (M_PI * r * r + L * 12.0) * 0.2;
    const double removed = volBefore - volAfter;
    EXPECT_GT(removed, approxRemoved * 0.5);
    EXPECT_LT(removed, approxRemoved * 1.5);

    EXPECT_EQ(out.signature.skill_id, std::string("friction_stir"));
    EXPECT_EQ(out.signature.tooling.tool_type, std::string("fsw_pin_tool"));
    EXPECT_EQ(out.signature.pattern["additive"].get<bool>(), false);
    EXPECT_GT(out.signature.tooling.stock_removed_mm3, 0.0);
}

// ─── 2. DFM: tool shoulder < 6 mm rejected ────────────────────────────────
TEST(SkillFrictionStir, ValidateRejectsTinyShoulder)
{
    auto stock = skill::createCuboidStock(100.0, 80.0, 10.0);

    skill::friction_stir::Input tiny;
    tiny.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    tiny.start_x_mm = 20.0; tiny.start_y_mm = 40.0;
    tiny.end_x_mm   = 80.0; tiny.end_y_mm   = 40.0;
    tiny.tool_shoulder_dia_mm    = 4.0;    // < 6.0
    tiny.tool_rotation_rpm       = 1500.0;
    tiny.traverse_speed_mm_per_s = 5.0;
    tiny.depth_mm                = 0.2;
    auto r = skill::friction_stir::validate(*stock, tiny);
    EXPECT_FALSE(r.passed);
    EXPECT_THROW(skill::friction_stir::apply(*stock, tiny), skill::SkillError);
}

// ─── 3. DFM: rotation rpm outside [200, 3000] rejected ────────────────────
TEST(SkillFrictionStir, ValidateRejectsBadRpm)
{
    auto stock = skill::createCuboidStock(100.0, 80.0, 10.0);

    skill::friction_stir::Input slow;
    slow.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    slow.start_x_mm = 20.0; slow.start_y_mm = 40.0;
    slow.end_x_mm   = 80.0; slow.end_y_mm   = 40.0;
    slow.tool_shoulder_dia_mm    = 12.0;
    slow.tool_rotation_rpm       = 100.0;    // < 200
    slow.traverse_speed_mm_per_s = 5.0;
    slow.depth_mm                = 0.2;
    auto r1 = skill::friction_stir::validate(*stock, slow);
    EXPECT_FALSE(r1.passed);

    skill::friction_stir::Input fast = slow;
    fast.tool_rotation_rpm = 4000.0;   // > 3000
    auto r2 = skill::friction_stir::validate(*stock, fast);
    EXPECT_FALSE(r2.passed);
}

// ─── 4. Recognize via metadata ────────────────────────────────────────────
TEST(SkillFrictionStir, RecognizeViaMetadata)
{
    auto stock = skill::createCuboidStock(120.0, 80.0, 10.0);

    skill::friction_stir::Input in;
    in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.start_x_mm = 20.0; in.start_y_mm = 40.0;
    in.end_x_mm   = 100.0; in.end_y_mm  = 40.0;
    in.tool_shoulder_dia_mm    = 14.0;
    in.tool_rotation_rpm       = 1800.0;
    in.traverse_speed_mm_per_s = 4.0;
    in.depth_mm                = 0.25;
    in.material                = "aluminum";

    auto out = skill::friction_stir::apply(*stock, in);
    auto cands = skill::friction_stir::recognize(*out.workpiece);

    ASSERT_FALSE(cands.empty());
    EXPECT_GT(cands[0].confidence, 0.9);
    EXPECT_NEAR(cands[0].recovered_params["tool_shoulder_dia_mm"].get<double>(),
                14.0, 1e-3);
    EXPECT_NEAR(cands[0].recovered_params["depth_mm"].get<double>(), 0.25, 1e-3);
    EXPECT_NEAR(cands[0].recovered_params["tool_rotation_rpm"].get<double>(),
                1800.0, 1e-3);
}

// ─── 5. Tooling metadata: solid-state, no filler ──────────────────────────
TEST(SkillFrictionStir, ToolingMetaSolidState)
{
    auto stock = skill::createCuboidStock(120.0, 80.0, 10.0);

    skill::friction_stir::Input in;
    in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.start_x_mm = 20.0; in.start_y_mm = 40.0;
    in.end_x_mm   = 100.0; in.end_y_mm  = 40.0;
    in.tool_shoulder_dia_mm    = 10.0;
    in.tool_rotation_rpm       = 1200.0;
    in.traverse_speed_mm_per_s = 5.0;
    in.depth_mm                = 0.2;

    auto out = skill::friction_stir::apply(*stock, in);
    EXPECT_EQ(out.signature.tooling.tool_material, std::string("h13_tool_steel"));
    EXPECT_EQ(out.signature.tooling.extra["filler"], "none");
    EXPECT_EQ(out.signature.tooling.extra["solid_state"], true);
}

// ─── 6. Recognize geometric path on anonymous workpiece (shallow trace) ───
TEST(SkillFrictionStir, RecognizeGeometricShallowTrace)
{
    auto stock = skill::createCuboidStock(120.0, 80.0, 10.0);

    skill::friction_stir::Input in;
    in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.start_x_mm = 20.0; in.start_y_mm = 40.0;
    in.end_x_mm   = 100.0; in.end_y_mm  = 40.0;
    in.tool_shoulder_dia_mm    = 12.0;
    in.tool_rotation_rpm       = 1500.0;
    in.traverse_speed_mm_per_s = 5.0;
    in.depth_mm                = 0.2;

    auto out = skill::friction_stir::apply(*stock, in);

    // Strip metadata — exercise geometric path.
    skill::Workpiece raw(out.workpiece->shape());
    auto cands = skill::friction_stir::recognize(raw);
    ASSERT_FALSE(cands.empty());
    EXPECT_NEAR(cands[0].recovered_params["tool_shoulder_dia_mm"].get<double>(),
                12.0, 0.5);
    EXPECT_LT(cands[0].recovered_params["depth_mm"].get<double>(), 0.5);
}
