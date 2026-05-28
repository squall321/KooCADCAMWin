// @lat: [[process/test-strategy#skill round-trip]]
//
// laser_weld skill — 6-case sweep covering keyhole aspect ratio,
// narrow-bead DFM, and metadata-driven recognition.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/laser_weld.hpp"

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

// ─── 1. Apply: narrow surface cap fused; keyhole depth in metadata ────────
TEST(SkillLaserWeld, ApplyAddsNarrowCap)
{
    auto stock = skill::createCuboidStock(80.0, 40.0, 10.0);
    const double volBefore = volumeOf(stock->shape());

    skill::laser_weld::Input in;
    in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1), 5.0, "largest" };
    in.start_x_mm = 20.0; in.start_y_mm = 20.0;
    in.end_x_mm   = 60.0; in.end_y_mm   = 20.0;
    in.weld_width_mm  = 0.6;
    in.weld_depth_mm  = 4.0;     // aspect = 4 / 0.6 ≈ 6.7 → keyhole regime
    in.weld_height_mm = 0.1;
    in.material       = "stainless";
    in.laser_power_w  = 3000.0;

    auto out = skill::laser_weld::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    // Only the surface cap is modeled; volume added must be SMALL compared
    // to declared weld_depth_mm.
    const double r = 0.3;
    const double L = 40.0;
    const double approxAdded = (M_PI * r * r + L * 0.6) * 0.1;
    const double added = volumeOf(out.workpiece->shape()) - volBefore;
    EXPECT_GT(added, approxAdded * 0.5);
    EXPECT_LT(added, approxAdded * 1.5);

    EXPECT_EQ(out.signature.skill_id, std::string("laser_weld"));
    EXPECT_EQ(out.signature.tooling.tool_type, std::string("laser_head"));
    EXPECT_TRUE(out.signature.pattern["keyhole"].get<bool>());
    EXPECT_GT(out.signature.pattern["depth_to_width"].get<double>(), 3.0);
}

// ─── 2. DFM: weld width outside [0.1, 1.5] rejected ───────────────────────
TEST(SkillLaserWeld, ValidateRejectsBadWidth)
{
    auto stock = skill::createCuboidStock(80.0, 40.0, 10.0);

    skill::laser_weld::Input narrow;
    narrow.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    narrow.start_x_mm = 20.0; narrow.start_y_mm = 20.0;
    narrow.end_x_mm   = 60.0; narrow.end_y_mm   = 20.0;
    narrow.weld_width_mm  = 0.05;   // < 0.1
    narrow.weld_depth_mm  = 2.0;
    narrow.weld_height_mm = 0.1;
    auto r1 = skill::laser_weld::validate(*stock, narrow);
    EXPECT_FALSE(r1.passed);

    skill::laser_weld::Input wide = narrow;
    wide.weld_width_mm  = 2.0;     // > 1.5
    wide.weld_depth_mm  = 8.0;     // keep aspect ≥ 3 so we hit only the width rule
    auto r2 = skill::laser_weld::validate(*stock, wide);
    EXPECT_FALSE(r2.passed);
    bool foundWidth = false;
    for (const auto& f : r2.findings)
        if (f.code == "DFM-LASER-WIDTH") { foundWidth = true; break; }
    EXPECT_TRUE(foundWidth);
}

// ─── 3. DFM: depth/width < 3 → not a keyhole → reject ─────────────────────
TEST(SkillLaserWeld, ValidateRejectsConductionWeld)
{
    auto stock = skill::createCuboidStock(80.0, 40.0, 10.0);

    skill::laser_weld::Input shallow;
    shallow.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    shallow.start_x_mm = 20.0; shallow.start_y_mm = 20.0;
    shallow.end_x_mm   = 60.0; shallow.end_y_mm   = 20.0;
    shallow.weld_width_mm  = 1.0;
    shallow.weld_depth_mm  = 2.0;    // aspect = 2 → conduction regime
    shallow.weld_height_mm = 0.1;

    auto r = skill::laser_weld::validate(*stock, shallow);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-LASER-ASPECT") { found = true; break; }
    EXPECT_TRUE(found);
    EXPECT_THROW(skill::laser_weld::apply(*stock, shallow), skill::SkillError);
}

// ─── 4. Recognize via metadata — depth recovered ──────────────────────────
TEST(SkillLaserWeld, RecognizeViaMetadata)
{
    auto stock = skill::createCuboidStock(80.0, 40.0, 10.0);

    skill::laser_weld::Input in;
    in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.start_x_mm = 20.0; in.start_y_mm = 20.0;
    in.end_x_mm   = 60.0; in.end_y_mm   = 20.0;
    in.weld_width_mm  = 0.5;
    in.weld_depth_mm  = 5.0;
    in.weld_height_mm = 0.1;
    in.material = "stainless";

    auto out = skill::laser_weld::apply(*stock, in);
    auto cands = skill::laser_weld::recognize(*out.workpiece);

    ASSERT_FALSE(cands.empty());
    EXPECT_GT(cands[0].confidence, 0.9);
    EXPECT_NEAR(cands[0].recovered_params["weld_width_mm"].get<double>(), 0.5, 1e-3);
    EXPECT_NEAR(cands[0].recovered_params["weld_depth_mm"].get<double>(), 5.0, 1e-3);
}

// ─── 5. Subsurface fusion is declared but NOT modeled in geometry ─────────
TEST(SkillLaserWeld, SubsurfaceNotModeled)
{
    auto stock = skill::createCuboidStock(80.0, 40.0, 10.0);

    skill::laser_weld::Input in;
    in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.start_x_mm = 20.0; in.start_y_mm = 20.0;
    in.end_x_mm   = 60.0; in.end_y_mm   = 20.0;
    in.weld_width_mm  = 0.4;
    in.weld_depth_mm  = 6.0;
    in.weld_height_mm = 0.1;

    auto out = skill::laser_weld::apply(*stock, in);
    EXPECT_EQ(out.signature.pattern["subsurface_fusion_modeled"].get<bool>(),
              false);
    EXPECT_NEAR(out.signature.pattern["weld_depth_mm"].get<double>(), 6.0, 1e-3);
}

// ─── 6. Tooling: stock_removed negative (cap is additive), fastest cycle ──
TEST(SkillLaserWeld, ToolingMetaFastCycle)
{
    auto stock = skill::createCuboidStock(100.0, 40.0, 10.0);

    skill::laser_weld::Input in;
    in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.start_x_mm = 10.0; in.start_y_mm = 20.0;
    in.end_x_mm   = 90.0; in.end_y_mm   = 20.0;
    in.weld_width_mm  = 0.5;
    in.weld_depth_mm  = 4.0;
    in.weld_height_mm = 0.1;

    auto out = skill::laser_weld::apply(*stock, in);
    EXPECT_LT(out.signature.tooling.stock_removed_mm3, 0.0);
    EXPECT_EQ(out.signature.tooling.tool_material, std::string("fiber_laser"));
    // Travel speed = 80 mm/s → cycle ~ 1 s for 80 mm seam.
    EXPECT_LT(out.signature.tooling.est_cycle_time_s, 2.0);
}
