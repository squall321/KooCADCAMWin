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
#include "skills/revolve_boss.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

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
