// @lat: [[process/test-strategy#skill compound]]

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/busbar_lap_joint.hpp"

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

// 1. Apply removes real geometry: 2 bolt holes + notch + dimples.
TEST(SkillBusbarLapJoint, ApplyCutsRealGeometry)
{
    // Bar 80 × 40 × 5 mm.  end at x=0, extending +X.
    auto stock = skill::createCuboidStock(80.0, 40.0, 5.0);
    const double volBefore = volumeOf(stock->shape());
    const int    facesBefore = stock->faceCount();

    skill::busbar_lap_joint::Input in;
    in.entry_face        = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.end_x_mm          = 0.0;
    in.end_y_mm          = 20.0;     // bar centerline
    in.lap_length_mm     = 60.0;
    in.bar_width_mm      = 40.0;
    in.bar_thickness_mm  = 5.0;
    in.bolt_dia_mm       = 10.0;
    in.bolt_pitch_mm     = 40.0;
    in.notch_depth_mm    = 6.0;
    in.notch_width_mm    = 4.0;
    in.dimple_grid_nx    = 3;
    in.dimple_grid_ny    = 3;
    in.dimple_dia_mm     = 1.5;
    in.dimple_depth_mm   = 0.3;

    auto out = skill::busbar_lap_joint::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    const double rBolt = 5.0, rDim = 0.75;
    const double approxRem =
        2.0 * M_PI * rBolt * rBolt * 5.0
      + 4.0 * 6.0 * 5.0
      + 9 * M_PI * rDim * rDim * 0.3;
    const double diff = volBefore - volumeOf(out.workpiece->shape());
    EXPECT_NEAR(diff, approxRem, approxRem * 0.20);
    EXPECT_GT(out.workpiece->faceCount(), facesBefore);
}

// 2. Validate rejects bad input + apply throws.
TEST(SkillBusbarLapJoint, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(80.0, 40.0, 5.0);
    skill::busbar_lap_joint::Input bad;
    bad.entry_face        = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    bad.end_x_mm          = 0.0;
    bad.end_y_mm          = 20.0;
    bad.lap_length_mm     = 60.0;
    bad.bar_width_mm      = 40.0;
    bad.bar_thickness_mm  = 5.0;
    bad.bolt_dia_mm       = 10.0;
    bad.bolt_pitch_mm     = 15.0;   // pitch < 2.5×dia=25 → DFM-NEMA-CC1
    bad.notch_depth_mm    = 6.0;
    bad.notch_width_mm    = 4.0;
    bad.dimple_grid_nx    = 3;
    bad.dimple_grid_ny    = 3;
    bad.dimple_dia_mm     = 1.5;
    bad.dimple_depth_mm   = 0.3;

    auto r = skill::busbar_lap_joint::validate(*stock, bad);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-NEMA-CC1") { found = true; break; }
    EXPECT_TRUE(found);
    EXPECT_THROW(skill::busbar_lap_joint::apply(*stock, bad), skill::SkillError);
}

// 3. Signature compound + subfeature_count >= 2.
TEST(SkillBusbarLapJoint, SignatureIsCompound)
{
    auto stock = skill::createCuboidStock(80.0, 40.0, 5.0);
    skill::busbar_lap_joint::Input in;
    in.entry_face        = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.end_x_mm          = 0.0;
    in.end_y_mm          = 20.0;
    in.lap_length_mm     = 60.0;
    in.bar_width_mm      = 40.0;
    in.bar_thickness_mm  = 5.0;
    in.bolt_dia_mm       = 10.0;
    in.bolt_pitch_mm     = 40.0;
    in.notch_depth_mm    = 6.0;
    in.notch_width_mm    = 4.0;
    in.dimple_grid_nx    = 3;
    in.dimple_grid_ny    = 3;
    in.dimple_dia_mm     = 1.5;
    in.dimple_depth_mm   = 0.3;

    auto out = skill::busbar_lap_joint::apply(*stock, in);
    const auto& pat = out.signature.pattern;
    EXPECT_TRUE(pat["is_compound"].get<bool>());
    EXPECT_GE(pat["subfeature_count"].get<int>(), 2);
    EXPECT_EQ(pat["bolt_hole_count"].get<int>(), 2);
    EXPECT_EQ(pat["dimple_count"].get<int>(), 9);

    const auto& p = out.signature.params;
    EXPECT_DOUBLE_EQ(p["bolt_dia_mm"].get<double>(), 10.0);
    EXPECT_DOUBLE_EQ(p["bolt_pitch_mm"].get<double>(), 40.0);
}

// 4. Recognize: metadata 1.0 path present.
TEST(SkillBusbarLapJoint, RecognizeReturnsCandidate)
{
    auto stock = skill::createCuboidStock(80.0, 40.0, 5.0);
    skill::busbar_lap_joint::Input in;
    in.entry_face        = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.end_x_mm          = 0.0;
    in.end_y_mm          = 20.0;
    in.lap_length_mm     = 60.0;
    in.bar_width_mm      = 40.0;
    in.bar_thickness_mm  = 5.0;
    in.bolt_dia_mm       = 10.0;
    in.bolt_pitch_mm     = 40.0;
    in.notch_depth_mm    = 6.0;
    in.notch_width_mm    = 4.0;
    in.dimple_grid_nx    = 3;
    in.dimple_grid_ny    = 3;
    in.dimple_dia_mm     = 1.5;
    in.dimple_depth_mm   = 0.3;

    auto out = skill::busbar_lap_joint::apply(*stock, in);
    auto cands = skill::busbar_lap_joint::recognize(*out.workpiece);
    ASSERT_FALSE(cands.empty());
    bool hasMeta = false;
    for (const auto& c : cands)
        if (std::abs(c.confidence - 1.0) < 1e-6) { hasMeta = true; break; }
    EXPECT_TRUE(hasMeta);
}

// 5. Feature-specific: dimple_count = nx × ny.
TEST(SkillBusbarLapJoint, FeatureSpecific_DimpleGridCount)
{
    auto stock = skill::createCuboidStock(80.0, 40.0, 5.0);
    skill::busbar_lap_joint::Input in;
    in.entry_face        = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.end_x_mm          = 0.0;
    in.end_y_mm          = 20.0;
    in.lap_length_mm     = 60.0;
    in.bar_width_mm      = 40.0;
    in.bar_thickness_mm  = 5.0;
    in.bolt_dia_mm       = 10.0;
    in.bolt_pitch_mm     = 40.0;
    in.notch_depth_mm    = 6.0;
    in.notch_width_mm    = 4.0;
    in.dimple_grid_nx    = 4;
    in.dimple_grid_ny    = 2;
    in.dimple_dia_mm     = 1.5;
    in.dimple_depth_mm   = 0.3;

    auto out = skill::busbar_lap_joint::apply(*stock, in);
    EXPECT_EQ(out.signature.pattern["dimple_count"].get<int>(), 4 * 2);
}
