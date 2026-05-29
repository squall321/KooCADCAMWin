// @lat: [[process/test-strategy#skill compound feature]]
//
// spring_loaded_door_latch — REAL geometric test, multi-cut.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/spring_loaded_door_latch.hpp"

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

// ─── Test 1: apply produces real geometry ────────────────────────────────
TEST(SkillSpringLoadedDoorLatch, ApplyProducesRealGeometry)
{
    auto stock = skill::createCuboidStock(80.0, 50.0, 18.0);
    ASSERT_FALSE(stock->shape().IsNull());

    skill::spring_loaded_door_latch::Input in;
    in.entry_face            = skill::FaceByNormal{ gp_Dir(0, 0, 1), 5.0, "largest" };
    in.position_x_mm         = 40.0;
    in.position_y_mm         = 25.0;
    in.axis_dir              = gp_Dir(0, 0, -1);
    in.latch_pocket_size_mm  = 18.0;
    in.latch_pocket_depth_mm = 10.0;
    in.screw_hole_dia_mm     = 3.4;
    in.screw_hole_depth_mm   = 14.0;
    in.screw_spacing_mm      = 25.0;
    in.plunger_bore_dia_mm   = 6.0;
    in.plunger_bore_depth_mm = 10.0;
    in.receiver_slot_width_mm  = 7.0;
    in.receiver_slot_height_mm = 4.0;
    in.receiver_slot_depth_mm  = 5.0;

    auto out = skill::spring_loaded_door_latch::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    // Volume — sum of all cuts (overlaps reduce real removal):
    const double pocketV = in.latch_pocket_size_mm * in.latch_pocket_size_mm
                           * in.latch_pocket_depth_mm;
    const double screwR = in.screw_hole_dia_mm / 2.0;
    const double screwV = 2.0 * M_PI * screwR * screwR * in.screw_hole_depth_mm;
    const double plR    = in.plunger_bore_dia_mm / 2.0;
    const double plV    = M_PI * plR * plR
                          * (in.plunger_bore_depth_mm + in.latch_pocket_size_mm);
    const double recvV  = in.receiver_slot_depth_mm * in.receiver_slot_width_mm
                          * in.receiver_slot_height_mm;
    const double upper  = pocketV + screwV + plV + recvV;
    const double lower  = pocketV * 0.7;        // pocket alone is a lower bound

    const double removed = volumeOf(stock->shape())
                           - volumeOf(out.workpiece->shape());
    EXPECT_GT(removed, lower);
    EXPECT_LT(removed, upper * 1.3);

    EXPECT_GT(out.workpiece->faceCount(), stock->faceCount() + 3);
}

// ─── Test 2: validate rejects bad input ──────────────────────────────────
TEST(SkillSpringLoadedDoorLatch, ValidateRejects)
{
    auto stock = skill::createCuboidStock(80.0, 50.0, 18.0);
    skill::spring_loaded_door_latch::Input in;
    in.entry_face            = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm         = 40.0; in.position_y_mm = 25.0;
    in.latch_pocket_size_mm  = 3.0;        // < 6 — DFM-SPDL-POCKET
    in.latch_pocket_depth_mm = 10.0;
    in.screw_hole_dia_mm     = 3.4;
    in.screw_hole_depth_mm   = 14.0;
    in.screw_spacing_mm      = 25.0;
    in.plunger_bore_dia_mm   = 6.0;
    in.plunger_bore_depth_mm = 10.0;
    in.receiver_slot_width_mm  = 7.0;
    in.receiver_slot_height_mm = 4.0;
    in.receiver_slot_depth_mm  = 5.0;

    auto r = skill::spring_loaded_door_latch::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_THROW(skill::spring_loaded_door_latch::apply(*stock, in),
                 skill::SkillError);
}

// ─── Test 3: signature compound + round-trip ─────────────────────────────
TEST(SkillSpringLoadedDoorLatch, SignatureCompound)
{
    auto stock = skill::createCuboidStock(80.0, 50.0, 18.0);
    skill::spring_loaded_door_latch::Input in;
    in.entry_face            = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm         = 40.0; in.position_y_mm = 25.0;
    in.latch_pocket_size_mm  = 18.0;
    in.latch_pocket_depth_mm = 10.0;
    in.screw_hole_dia_mm     = 3.4;
    in.screw_hole_depth_mm   = 14.0;
    in.screw_spacing_mm      = 25.0;
    in.plunger_bore_dia_mm   = 6.0;
    in.plunger_bore_depth_mm = 10.0;
    in.receiver_slot_width_mm  = 7.0;
    in.receiver_slot_height_mm = 4.0;
    in.receiver_slot_depth_mm  = 5.0;

    auto out = skill::spring_loaded_door_latch::apply(*stock, in);
    EXPECT_TRUE(out.signature.pattern["is_compound"].get<bool>());
    EXPECT_EQ(out.signature.pattern["subfeature_count"].get<int>(), 5);
    EXPECT_GE(out.signature.pattern["subfeature_count"].get<int>(), 2);
    EXPECT_NEAR(out.signature.params["latch_pocket_size_mm"].get<double>(),
                in.latch_pocket_size_mm, 1e-6);
    EXPECT_NEAR(out.signature.params["plunger_bore_dia_mm"].get<double>(),
                in.plunger_bore_dia_mm, 1e-6);
}

// ─── Test 4: recognize ───────────────────────────────────────────────────
TEST(SkillSpringLoadedDoorLatch, RecognizeFinds)
{
    auto stock = skill::createCuboidStock(80.0, 50.0, 18.0);
    skill::spring_loaded_door_latch::Input in;
    in.entry_face            = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm         = 40.0; in.position_y_mm = 25.0;
    in.latch_pocket_size_mm  = 18.0;
    in.latch_pocket_depth_mm = 10.0;
    in.screw_hole_dia_mm     = 3.4;
    in.screw_hole_depth_mm   = 14.0;
    in.screw_spacing_mm      = 25.0;
    in.plunger_bore_dia_mm   = 6.0;
    in.plunger_bore_depth_mm = 10.0;
    in.receiver_slot_width_mm  = 7.0;
    in.receiver_slot_height_mm = 4.0;
    in.receiver_slot_depth_mm  = 5.0;

    auto out = skill::spring_loaded_door_latch::apply(*stock, in);
    auto cands = skill::spring_loaded_door_latch::recognize(*out.workpiece);
    ASSERT_FALSE(cands.empty());
    bool sawConfident = false;
    for (const auto& c : cands) if (c.confidence > 0.5) sawConfident = true;
    EXPECT_TRUE(sawConfident);
}

// ─── Test 5: feature-specific — plunger bore is horizontal (X-axis cyl) ─
TEST(SkillSpringLoadedDoorLatch, PlungerBoreIsHorizontal)
{
    auto stock = skill::createCuboidStock(80.0, 50.0, 18.0);
    skill::spring_loaded_door_latch::Input in;
    in.entry_face            = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm         = 40.0; in.position_y_mm = 25.0;
    in.latch_pocket_size_mm  = 18.0;
    in.latch_pocket_depth_mm = 10.0;
    in.screw_hole_dia_mm     = 3.4;
    in.screw_hole_depth_mm   = 14.0;
    in.screw_spacing_mm      = 25.0;
    in.plunger_bore_dia_mm   = 6.0;
    in.plunger_bore_depth_mm = 10.0;
    in.receiver_slot_width_mm  = 7.0;
    in.receiver_slot_height_mm = 4.0;
    in.receiver_slot_depth_mm  = 5.0;

    auto out = skill::spring_loaded_door_latch::apply(*stock, in);
    auto cands = skill::spring_loaded_door_latch::recognize(*out.workpiece);

    // Geometric path detects horizontal_cyl_count >= 1
    bool checked = false;
    for (const auto& c : cands) {
        if (c.matched_geometry.contains("x_axis_cyl_count")) {
            EXPECT_GE(c.matched_geometry["x_axis_cyl_count"].get<int>(), 1);
            checked = true;
            break;
        }
    }
    EXPECT_TRUE(checked);

    // Also verify pattern records horizontal_cyl_count
    EXPECT_EQ(out.signature.pattern["horizontal_cyl_count"].get<int>(), 1);
}
