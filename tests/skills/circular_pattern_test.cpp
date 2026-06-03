// @lat: [[process/test-strategy#skill round-trip]]
//
// circular_pattern — N holes around an axis on a disc.
//
// Cases (5):
//   1. 6 holes on 360° remove ~6 × per-hole volume on a disc.
//   2. 4 holes on 180° remove ~4 × per-hole volume.
//   3. DFM rejects chord-overlap (small radial_offset + small count).
//   4. DFM rejects count = 0.
//   5. recognize finds the bolt circle (history replay).

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/circular_pattern.hpp"

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

// ─── 1. 6 holes around full circle ─────────────────────────────────────────
TEST(SkillCircularPattern, SixHolesFullCircle)
{
    auto stock = skill::createCylindricalStock(80.0, 10.0);
    const double volBefore = volumeOf(stock->shape());

    skill::circular_pattern::Input in;
    in.axis_origin_xyz   = { 0.0, 0.0, 10.0 };
    in.axis_dir_xyz      = { 0.0, 0.0, 1.0 };
    in.hole_dia_mm       = 5.0;
    in.hole_depth_mm     = 6.0;
    in.radial_offset_mm  = 30.0;
    in.count             = 6;
    in.total_angle_deg   = 360.0;

    auto out = skill::circular_pattern::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    const double removed  = volBefore - volumeOf(out.workpiece->shape());
    const double per      = M_PI * 2.5 * 2.5 * 6.0;
    const double expected = 6.0 * per;
    EXPECT_NEAR(removed, expected, expected * 0.15);
    EXPECT_EQ(out.signature.pattern["instance_count"].get<int>(), 6);
    EXPECT_TRUE(out.signature.pattern["is_pattern"].get<bool>());
}

// ─── 2. 4 holes over 180° ──────────────────────────────────────────────────
TEST(SkillCircularPattern, FourHolesHalfCircle)
{
    auto stock = skill::createCylindricalStock(80.0, 10.0);
    const double volBefore = volumeOf(stock->shape());

    skill::circular_pattern::Input in;
    in.axis_origin_xyz  = { 0.0, 0.0, 10.0 };
    in.axis_dir_xyz     = { 0.0, 0.0, 1.0 };
    in.hole_dia_mm      = 4.0;
    in.hole_depth_mm    = 5.0;
    in.radial_offset_mm = 25.0;
    in.count            = 4;
    in.total_angle_deg  = 180.0;

    auto out = skill::circular_pattern::apply(*stock, in);
    const double removed = volBefore - volumeOf(out.workpiece->shape());
    const double per      = M_PI * 2.0 * 2.0 * 5.0;
    const double expected = 4.0 * per;
    EXPECT_NEAR(removed, expected, expected * 0.15);
}

// ─── 3. DFM rejects chord-pitch overlap ────────────────────────────────────
TEST(SkillCircularPattern, ValidateRejectsChordOverlap)
{
    auto stock = skill::createCylindricalStock(80.0, 10.0);
    skill::circular_pattern::Input in;
    in.axis_origin_xyz  = { 0.0, 0.0, 10.0 };
    in.hole_dia_mm      = 20.0;
    in.hole_depth_mm    = 5.0;
    in.radial_offset_mm = 5.0;       // chord = 2*5*sin(π/12) ≈ 2.6 < 20
    in.count            = 12;
    in.total_angle_deg  = 360.0;
    auto r = skill::circular_pattern::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings) if (f.code == "DFM-PATT-2") found = true;
    EXPECT_TRUE(found);
}

// ─── 4. DFM rejects count = 0 ──────────────────────────────────────────────
TEST(SkillCircularPattern, ValidateRejectsZeroCount)
{
    auto stock = skill::createCylindricalStock(80.0, 10.0);
    skill::circular_pattern::Input in;
    in.hole_dia_mm      = 4.0;
    in.hole_depth_mm    = 5.0;
    in.radial_offset_mm = 25.0;
    in.count            = 0;
    in.total_angle_deg  = 360.0;
    auto r = skill::circular_pattern::validate(*stock, in);
    EXPECT_FALSE(r.passed);
}

// ─── 5. Recognize identifies circle (history replay) ───────────────────────
TEST(SkillCircularPattern, RecognizeIdentifiesPattern)
{
    auto stock = skill::createCylindricalStock(80.0, 10.0);
    skill::circular_pattern::Input in;
    in.axis_origin_xyz  = { 0.0, 0.0, 10.0 };
    in.axis_dir_xyz     = { 0.0, 0.0, 1.0 };
    in.hole_dia_mm      = 5.0;
    in.hole_depth_mm    = 6.0;
    in.radial_offset_mm = 30.0;
    in.count            = 6;
    in.total_angle_deg  = 360.0;
    auto out = skill::circular_pattern::apply(*stock, in);
    auto cands = skill::circular_pattern::recognize(*out.workpiece);
    ASSERT_GE(cands.size(), 1u);
    EXPECT_EQ(cands[0].skill_id, std::string("circular_pattern"));
    EXPECT_GT(cands[0].confidence, 0.5);
}
