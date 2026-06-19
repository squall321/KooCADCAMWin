// @lat: [[process/test-strategy#skill round-trip]]
//
// Sketch-extrusion reverse-engineering round-trip — the first NON-machining
// feature the recognizer handles.  Most real CAD is sketch->extrude, not
// drill/pocket; the RE loop must recognise an extrusion and regenerate it from
// the recovered profile + height, exactly as it does for machining features.
//
// This proves the metadata-replay path (recognize -> regenerate).  Recovering
// the profile from FOREIGN geometry (no feature history) is a follow-up slice.

#include <gtest/gtest.h>

#include "re/Recognizer.hpp"
#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/extrude_boss_from_sketch.hpp"

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

TEST(ExtrudeRoundTrip, RecogniseThenRegenerateMatchesGeometry)
{
    // GENERATE: a 10x10 boss, 4 mm tall, on the top face of a 60x60x10 stock.
    auto stock = skill::createCuboidStock(60.0, 60.0, 10.0);
    ASSERT_FALSE(stock->shape().IsNull());

    skill::extrude_boss_from_sketch::Input in;
    in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.polygon    = { { -5.0, -5.0 }, { 5.0, -5.0 }, { 5.0, 5.0 }, { -5.0, 5.0 } };
    in.height_mm  = 4.0;

    const auto out = skill::extrude_boss_from_sketch::apply(*stock, in);
    ASSERT_NE(out.workpiece, nullptr);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    const double vOrig = volumeOf(out.workpiece->shape());

    // RECOGNISE: the extrusion is found at/above the 0.7 threshold (it was
    // previously capped to 0.5 by the "feature_history" source bug and dropped).
    const auto cands = re::analyzeFiltered(*out.workpiece, 0.7);
    const skill::RecognizedFeature* found = nullptr;
    for (const auto& c : cands)
        if (c.skill_id == "extrude_boss_from_sketch") { found = &c; break; }
    ASSERT_NE(found, nullptr)
        << "a generated extrusion must be recognised at >= 0.7 confidence";
    EXPECT_NEAR(found->recovered_params.value("height_mm", 0.0), 4.0, 1e-6);
    ASSERT_TRUE(found->recovered_params.contains("polygon"));
    EXPECT_EQ(found->recovered_params["polygon"].size(), 4u)
        << "the 4-vertex profile must round-trip";

    // REGENERATE from the recovered profile + height → identical geometry.
    skill::extrude_boss_from_sketch::Input in2;
    in2.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in2.height_mm  = found->recovered_params["height_mm"].get<double>();
    for (const auto& p : found->recovered_params["polygon"])
        in2.polygon.emplace_back(p["x"].get<double>(), p["y"].get<double>());

    const auto out2 = skill::extrude_boss_from_sketch::apply(*stock, in2);
    ASSERT_NE(out2.workpiece, nullptr);
    ASSERT_FALSE(out2.workpiece->shape().IsNull());
    EXPECT_NEAR(volumeOf(out2.workpiece->shape()), vOrig, 1e-3)
        << "recovered params must regenerate the same extrusion";
}
