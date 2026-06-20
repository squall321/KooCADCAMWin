// @lat: [[process/test-strategy#skill round-trip]]
//
// Sketch-REVOLVE reverse-engineering round-trip — the second non-machining
// feature the recognizer handles (after extrude).  A profile revolved into a
// solid of revolution must recognise and regenerate from the recovered profile
// + axis + angle, like the extrusion does.
//
// This proves the metadata-replay path.  Geometric recovery of a revolution
// from foreign surfaces needs Workpiece surface-of-revolution detection — a
// follow-up slice.

#include <gtest/gtest.h>

#include "re/Recognizer.hpp"
#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/drill_hole.hpp"
#include "skills/revolve_boss.hpp"

#include <BRepGProp.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>
#include <gp_Ax2.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>

#include <cmath>
#include <string>
#include <utility>
#include <vector>

using namespace koocadcam;

namespace {
double volumeOf(const TopoDS_Shape& s)
{
    GProp_GProps p;
    BRepGProp::VolumeProperties(s, p);
    return p.Mass();
}
}  // namespace

TEST(RevolveRoundTrip, RecogniseThenRegenerateMatchesGeometry)
{
    // GENERATE: a ring (4x2 section at r in [10,14], z in [0,2]) revolved 360°
    // about +Z, fused onto a stock.
    auto stock = skill::createCuboidStock(60.0, 60.0, 10.0);
    ASSERT_FALSE(stock->shape().IsNull());

    skill::revolve_boss::Input in;
    in.profile_polyline     = { { 10.0, 0.0 }, { 14.0, 0.0 }, { 14.0, 2.0 }, { 10.0, 2.0 } };
    in.axis_origin          = gp_Pnt(0.0, 0.0, 0.0);
    in.axis_dir             = gp_Dir(0.0, 0.0, 1.0);
    in.revolution_angle_deg = 360.0;

    const auto out = skill::revolve_boss::apply(*stock, in);
    ASSERT_NE(out.workpiece, nullptr);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    const double vOrig = volumeOf(out.workpiece->shape());

    // RECOGNISE: the revolve is found at/above the 0.7 threshold (it was
    // previously capped to 0.5 by the "feature_history" source bug, and the
    // recognizer was never even registered).
    const auto cands = re::analyzeFiltered(*out.workpiece, 0.7);
    const skill::RecognizedFeature* found = nullptr;
    for (const auto& c : cands)
        if (c.skill_id == "revolve_boss") { found = &c; break; }
    ASSERT_NE(found, nullptr)
        << "a generated revolve must be recognised at >= 0.7 confidence";
    EXPECT_NEAR(found->recovered_params.value("revolution_angle_deg", 0.0), 360.0, 1e-6);
    ASSERT_TRUE(found->recovered_params.contains("profile_polyline"));
    EXPECT_EQ(found->recovered_params["profile_polyline"].size(), 4u)
        << "the 4-vertex profile must round-trip";

    // REGENERATE from the recovered profile + axis + angle → identical geometry.
    skill::revolve_boss::Input in2;
    in2.revolution_angle_deg = found->recovered_params["revolution_angle_deg"].get<double>();
    for (const auto& p : found->recovered_params["profile_polyline"])
        in2.profile_polyline.emplace_back(p["r"].get<double>(), p["z"].get<double>());
    const auto ao = found->recovered_params["axis_origin"];
    const auto ad = found->recovered_params["axis_dir"];
    in2.axis_origin = gp_Pnt(ao[0].get<double>(), ao[1].get<double>(), ao[2].get<double>());
    in2.axis_dir    = gp_Dir(ad[0].get<double>(), ad[1].get<double>(), ad[2].get<double>());

    const auto out2 = skill::revolve_boss::apply(*stock, in2);
    ASSERT_NE(out2.workpiece, nullptr);
    ASSERT_FALSE(out2.workpiece->shape().IsNull());
    EXPECT_NEAR(volumeOf(out2.workpiece->shape()), vOrig, 1e-3)
        << "recovered params must regenerate the same revolution";
}

// GEOMETRIC path B: a foreign solid of revolution (a plain cylinder, no feature
// history) is detected by the coaxial-revolution-face grouping, recovering the
// axis + radial/axial envelope.
TEST(RevolveRoundTrip, RecognisesForeignCylinderAxisAndEnvelope)
{
    const double R = 5.0, H = 10.0;
    const TopoDS_Shape cyl =
        BRepPrimAPI_MakeCylinder(gp_Ax2(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1)), R, H).Shape();
    skill::Workpiece wp(cyl);

    const auto cands = skill::revolve_boss::recognize(wp);
    const skill::RecognizedFeature* g = nullptr;
    for (const auto& c : cands)
        if (c.matched_geometry.value("source", std::string{}) == "geometry") { g = &c; break; }
    ASSERT_NE(g, nullptr) << "a foreign solid of revolution must be detected";
    EXPECT_NEAR(g->recovered_params.value("max_radius_mm", 0.0), R, 1e-3);
    EXPECT_NEAR(g->recovered_params.value("axial_span_mm", 0.0), H, 1e-3);
    // Axis is +Z.
    ASSERT_TRUE(g->recovered_params.contains("axis_dir"));
    EXPECT_NEAR(std::abs(g->recovered_params["axis_dir"][2].get<double>()), 1.0, 1e-6);

    // MERIDIAN: a single solid cylinder recovers the rectangle meridian
    // {(0,0),(R,0),(R,H),(0,H)} which revolves back to the same cylinder.  Verify
    // the recovered (r,z) profile directly (revolving it is exact by Pappus:
    // 2*pi * r_centroid(R/2) * area(R*H) = pi*R^2*H).
    ASSERT_TRUE(g->recovered_params.contains("profile_polyline"));
    const auto& prof = g->recovered_params["profile_polyline"];
    ASSERT_EQ(prof.size(), 4u) << "a solid cylinder's meridian is a 4-point rectangle";

    double maxR = 0.0, zLo = 1e30, zHi = -1e30, area = 0.0;
    for (std::size_t i = 0; i < prof.size(); ++i) {
        const double r = prof[i]["r"].get<double>(), z = prof[i]["z"].get<double>();
        maxR = std::max(maxR, r);
        zLo = std::min(zLo, z); zHi = std::max(zHi, z);
        const auto& q = prof[(i + 1) % prof.size()];
        area += r * q["z"].get<double>() - q["r"].get<double>() * z;   // shoelace (r,z)
    }
    area = std::abs(area) * 0.5;
    EXPECT_NEAR(maxR, R, 1e-3)        << "meridian max radius == cylinder radius";
    EXPECT_NEAR(zHi - zLo, H, 1e-3)   << "meridian z-span == cylinder height";
    EXPECT_NEAR(area, R * H, R * H * 0.01)
        << "meridian area == R*H (revolves to pi*R^2*H by Pappus)";
}

// SPECIFICITY: a drilled block has a cylinder (the hole) but also walls PARALLEL
// to the axis, so it must NOT be mis-read as a solid of revolution.
TEST(RevolveRoundTrip, DrilledBlockIsNotARevolution)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 20.0);
    skill::drill_hole::Input in;
    in.position_x_mm = 20.0; in.position_y_mm = 20.0;
    in.diameter_mm = 8.0; in.depth_mm = 12.0;
    const auto out = skill::drill_hole::apply(*stock, in);
    ASSERT_NE(out.workpiece, nullptr);

    // Rebuild a fresh Workpiece (no feature history) so only the geometric path
    // can fire — the wall gate must reject it.
    skill::Workpiece wp(out.workpiece->shape());
    const auto cands = skill::revolve_boss::recognize(wp);
    for (const auto& c : cands)
        EXPECT_NE(c.matched_geometry.value("source", std::string{}), "geometry")
            << "a drilled block (axis-parallel walls) must not read as a revolution";
}
