// @lat: [[process/test-strategy#skill round-trip]]
//
// snap_fit — L-shaped flexible clip hook on a planar face.
//
// Cases (6):
//   1. apply on cuboid stock fuses an L-shape and ADDS volume.
//   2. DFM rejects too-thin wall (< 0.8 mm).
//   3. DFM rejects hook_depth > hook_height.
//   4. DFM warns short cantilever (length < 5 × wall_thickness).
//   5. recognize replays history.
//   6. direction_xy perpendicular (along +Y) works.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/snap_fit.hpp"

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

// ─── 1. Apply fuses L-shape ───────────────────────────────────────────────
TEST(SkillSnapFit, ApplyAddsLShapeVolume)
{
    auto stock = skill::createCuboidStock(50.0, 40.0, 10.0);
    const double stockVol = volumeOf(stock->shape());

    skill::snap_fit::Input in;
    in.entry_face        = skill::FaceByNormal{ gp_Dir(0, 0, 1), 5.0, "largest" };
    in.start_x_mm        = 10.0;
    in.start_y_mm        = 20.0;
    in.direction_xy      = gp_Dir(1.0, 0.0, 0.0);
    in.length_mm         = 15.0;
    in.hook_height_mm    = 2.0;
    in.hook_depth_mm     = 1.5;
    in.width_mm          = 4.0;
    in.wall_thickness_mm = 1.5;

    auto out = skill::snap_fit::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    const double added = volumeOf(out.workpiece->shape()) - stockVol;
    // L volume = beam (length × width × wall) + hook (hook_depth × width × hook_height)
    const double expectedAdd = 15.0 * 4.0 * 1.5 + 1.5 * 4.0 * 2.0;
    EXPECT_GT(added, 0.0);
    EXPECT_NEAR(added, expectedAdd, expectedAdd * 0.10);

    EXPECT_EQ(out.signature.skill_id, std::string("snap_fit"));
}

// ─── 2. DFM rejects sub-min wall ──────────────────────────────────────────
TEST(SkillSnapFit, ValidateRejectsThinWall)
{
    auto stock = skill::createCuboidStock(50.0, 40.0, 10.0);

    skill::snap_fit::Input in;
    in.entry_face        = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.start_x_mm        = 10.0;
    in.start_y_mm        = 20.0;
    in.direction_xy      = gp_Dir(1.0, 0.0, 0.0);
    in.length_mm         = 10.0;
    in.hook_height_mm    = 2.0;
    in.hook_depth_mm     = 1.0;
    in.width_mm          = 4.0;
    in.wall_thickness_mm = 0.5;     // < 0.8

    auto r = skill::snap_fit::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool sawCode = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-SNAP-WALL") { sawCode = true; break; }
    EXPECT_TRUE(sawCode);

    EXPECT_THROW(skill::snap_fit::apply(*stock, in), skill::SkillError);
}

// ─── 3. DFM rejects hook_depth > hook_height ──────────────────────────────
TEST(SkillSnapFit, ValidateRejectsTooDeepHook)
{
    auto stock = skill::createCuboidStock(50.0, 40.0, 10.0);

    skill::snap_fit::Input in;
    in.entry_face        = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.start_x_mm        = 10.0;
    in.start_y_mm        = 20.0;
    in.direction_xy      = gp_Dir(1.0, 0.0, 0.0);
    in.length_mm         = 12.0;
    in.hook_height_mm    = 1.0;
    in.hook_depth_mm     = 2.0;    // depth > height
    in.width_mm          = 4.0;
    in.wall_thickness_mm = 1.2;

    auto r = skill::snap_fit::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool sawCode = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-SNAP-HOOK") { sawCode = true; break; }
    EXPECT_TRUE(sawCode);
}

// ─── 4. DFM warns on short cantilever ─────────────────────────────────────
TEST(SkillSnapFit, ValidateWarnsOnShortBeam)
{
    auto stock = skill::createCuboidStock(50.0, 40.0, 10.0);

    skill::snap_fit::Input in;
    in.entry_face        = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.start_x_mm        = 10.0;
    in.start_y_mm        = 20.0;
    in.direction_xy      = gp_Dir(1.0, 0.0, 0.0);
    in.length_mm         = 4.0;        // 4 < 5 × 1.0
    in.hook_height_mm    = 2.0;
    in.hook_depth_mm     = 1.0;
    in.width_mm          = 4.0;
    in.wall_thickness_mm = 1.0;

    auto r = skill::snap_fit::validate(*stock, in);
    EXPECT_TRUE(r.passed);  // warning only
    bool sawWarn = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-SNAP-LENGTH" && f.severity == "warning") { sawWarn = true; break; }
    EXPECT_TRUE(sawWarn);
}

// ─── 5. Recognize replays history ─────────────────────────────────────────
TEST(SkillSnapFit, RecognizeReplaysHistory)
{
    auto stock = skill::createCuboidStock(50.0, 40.0, 10.0);

    skill::snap_fit::Input in;
    in.entry_face        = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.start_x_mm        = 12.0;
    in.start_y_mm        = 18.0;
    in.direction_xy      = gp_Dir(1.0, 0.0, 0.0);
    in.length_mm         = 14.0;
    in.hook_height_mm    = 2.5;
    in.hook_depth_mm     = 2.0;
    in.width_mm          = 5.0;
    in.wall_thickness_mm = 1.2;

    auto out = skill::snap_fit::apply(*stock, in);
    auto cands = skill::snap_fit::recognize(*out.workpiece);
    ASSERT_GE(cands.size(), 1u);

    const auto& c = cands.front();
    EXPECT_GT(c.confidence, 0.7);
    EXPECT_NEAR(c.recovered_params["length_mm"].get<double>(), 14.0, 0.05);
    EXPECT_NEAR(c.recovered_params["width_mm"].get<double>(), 5.0, 0.05);
    EXPECT_NEAR(c.recovered_params["wall_thickness_mm"].get<double>(), 1.2, 0.05);
}

// ─── 6. Perpendicular direction (along +Y) ────────────────────────────────
TEST(SkillSnapFit, DirectionPlusY)
{
    auto stock = skill::createCuboidStock(50.0, 40.0, 10.0);
    const double stockVol = volumeOf(stock->shape());

    skill::snap_fit::Input in;
    in.entry_face        = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.start_x_mm        = 25.0;
    in.start_y_mm        = 10.0;
    in.direction_xy      = gp_Dir(0.0, 1.0, 0.0);  // +Y
    in.length_mm         = 12.0;
    in.hook_height_mm    = 2.0;
    in.hook_depth_mm     = 1.5;
    in.width_mm          = 4.0;
    in.wall_thickness_mm = 1.2;

    auto out = skill::snap_fit::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    EXPECT_GT(volumeOf(out.workpiece->shape()), stockVol);
}
