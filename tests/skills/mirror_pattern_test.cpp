// @lat: [[process/test-strategy#skill round-trip]]
//
// mirror_pattern — original hole + mirror about X=0 (or Y=0) plane.
//
// Cases (5):
//   1. Mirror about X removes 2 holes worth of volume.
//   2. Mirror about Y also removes 2 holes worth of volume.
//   3. DFM rejects invalid mirror_axis.
//   4. DFM rejects overlap (hole at origin → 0 mirror distance).
//   5. recognize identifies the mirror pair (history replay).

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/mirror_pattern.hpp"

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
}  // namespace

// ─── 1. Mirror about X removes ~2 × hole volume ────────────────────────────
TEST(SkillMirrorPattern, MirrorAboutXRemovesTwoHoles)
{
    // The stock spans X = [0, 100], so face center is at (50, 25).
    // hole_x_mm = 10 → original at (60, 25), mirror at (40, 25).  Both inside.
    auto stock = skill::createCuboidStock(100.0, 50.0, 10.0);
    const double volBefore = volumeOf(stock->shape());

    skill::mirror_pattern::Input in;
    in.hole_x_mm     = 10.0;
    in.hole_y_mm     = 0.0;
    in.hole_dia_mm   = 5.0;
    in.hole_depth_mm = 5.0;
    in.face          = skill::FaceByNormal{ gp_Dir(0, 0, 1), 5.0, "largest" };
    in.mirror_axis   = "x";

    auto out = skill::mirror_pattern::apply(*stock, in);
    const double removed = volBefore - volumeOf(out.workpiece->shape());
    const double per      = M_PI * 2.5 * 2.5 * 5.0;
    const double expected = 2.0 * per;
    EXPECT_NEAR(removed, expected, expected * 0.15);
    EXPECT_EQ(out.signature.pattern["instance_count"].get<int>(), 2);
    EXPECT_TRUE(out.signature.pattern["is_pattern"].get<bool>());
}

// ─── 2. Mirror about Y axis ─────────────────────────────────────────────────
TEST(SkillMirrorPattern, MirrorAboutYRemovesTwoHoles)
{
    auto stock = skill::createCuboidStock(100.0, 100.0, 10.0);
    const double volBefore = volumeOf(stock->shape());

    skill::mirror_pattern::Input in;
    in.hole_x_mm     = 0.0;
    in.hole_y_mm     = 15.0;
    in.hole_dia_mm   = 4.0;
    in.hole_depth_mm = 4.0;
    in.face          = skill::FaceByNormal{ gp_Dir(0, 0, 1), 5.0, "largest" };
    in.mirror_axis   = "y";

    auto out = skill::mirror_pattern::apply(*stock, in);
    const double removed = volBefore - volumeOf(out.workpiece->shape());
    const double per      = M_PI * 2.0 * 2.0 * 4.0;
    const double expected = 2.0 * per;
    EXPECT_NEAR(removed, expected, expected * 0.15);
}

// ─── 3. DFM rejects invalid mirror_axis ────────────────────────────────────
TEST(SkillMirrorPattern, ValidateRejectsBadAxis)
{
    auto stock = skill::createCuboidStock(100.0, 50.0, 10.0);
    skill::mirror_pattern::Input in;
    in.hole_x_mm     = 10.0;
    in.hole_dia_mm   = 3.0;
    in.hole_depth_mm = 3.0;
    in.mirror_axis   = "z";          // invalid
    auto r = skill::mirror_pattern::validate(*stock, in);
    EXPECT_FALSE(r.passed);
}

// ─── 4. DFM rejects overlap (mirror distance ≤ dia) ────────────────────────
TEST(SkillMirrorPattern, ValidateRejectsOverlap)
{
    auto stock = skill::createCuboidStock(100.0, 50.0, 10.0);
    skill::mirror_pattern::Input in;
    in.hole_x_mm     = 1.0;          // mirror dist = 2 mm
    in.hole_y_mm     = 0.0;
    in.hole_dia_mm   = 5.0;          // dia 5 > 2 → overlap
    in.hole_depth_mm = 3.0;
    in.mirror_axis   = "x";
    auto r = skill::mirror_pattern::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings) if (f.code == "DFM-PATT-3") found = true;
    EXPECT_TRUE(found);
}

// ─── 5. Recognize identifies mirror pair (history replay) ──────────────────
TEST(SkillMirrorPattern, RecognizeIdentifiesMirror)
{
    auto stock = skill::createCuboidStock(100.0, 50.0, 10.0);
    skill::mirror_pattern::Input in;
    in.hole_x_mm     = 10.0;
    in.hole_y_mm     = 0.0;
    in.hole_dia_mm   = 5.0;
    in.hole_depth_mm = 5.0;
    in.face          = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.mirror_axis   = "x";

    auto out = skill::mirror_pattern::apply(*stock, in);
    auto cands = skill::mirror_pattern::recognize(*out.workpiece);
    ASSERT_GE(cands.size(), 1u);
    EXPECT_EQ(cands[0].skill_id, std::string("mirror_pattern"));
    EXPECT_GT(cands[0].confidence, 0.5);
}
