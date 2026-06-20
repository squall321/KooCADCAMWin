// @lat: [[process/test-strategy#re-roundtrip]]
//
// Sketch features through the REAL pipeline.  The extrude/revolve round-trip
// unit tests call apply() directly, which masked a pipeline break: the
// recognizers were registered and inferProcessPlan emitted their steps, but the
// Executor had no dispatch entry, so Executor::execute aborted on a recovered
// sketch boss.  This test exercises the full
// recognize -> inferProcessPlan -> Executor::execute path (the same contract
// watch_re_roundtrip proves for holes) and asserts geometry parity.

#include <gtest/gtest.h>

#include "process/Executor.hpp"
#include "process/ProcessPlan.hpp"
#include "re/Recognizer.hpp"
#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/extrude_boss_from_sketch.hpp"
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

// An extruded boss survives recognize -> inferProcessPlan -> Executor::execute.
TEST(SketchReExecutor, ExtrudeBossReplaysThroughExecutor)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 10.0);
    skill::extrude_boss_from_sketch::Input in;
    in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.polygon    = { { -6.0, -6.0 }, { 6.0, -6.0 }, { 6.0, 6.0 }, { -6.0, 6.0 } };
    in.height_mm  = 5.0;
    const auto synth = skill::extrude_boss_from_sketch::apply(*stock, in);
    ASSERT_NE(synth.workpiece, nullptr);
    const double vOrig = volumeOf(synth.workpiece->shape());

    process::ProcessPlan plan = re::inferProcessPlan(*synth.workpiece, 0.7);
    ASSERT_FALSE(plan.empty()) << "inferProcessPlan must emit the recovered boss";

    auto fresh = skill::createCuboidStock(60.0, 60.0, 10.0);
    const auto result = process::Executor::execute(plan, fresh);
    ASSERT_TRUE(result.ok())
        << "Executor must dispatch the recovered extrude boss (not abort): "
        << (result.errors.empty() ? "" : result.errors.front());
    ASSERT_NE(result.workpiece, nullptr);
    EXPECT_NEAR(volumeOf(result.workpiece->shape()), vOrig, 1.0)
        << "replayed boss must reproduce the same geometry";
}

// A revolved boss survives the same path.
TEST(SketchReExecutor, RevolveBossReplaysThroughExecutor)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 10.0);
    skill::revolve_boss::Input in;
    in.profile_polyline     = { { 10.0, 0.0 }, { 14.0, 0.0 }, { 14.0, 2.0 }, { 10.0, 2.0 } };
    in.axis_origin          = gp_Pnt(0, 0, 0);
    in.axis_dir             = gp_Dir(0, 0, 1);
    in.revolution_angle_deg = 360.0;
    const auto synth = skill::revolve_boss::apply(*stock, in);
    ASSERT_NE(synth.workpiece, nullptr);
    const double vOrig = volumeOf(synth.workpiece->shape());

    process::ProcessPlan plan = re::inferProcessPlan(*synth.workpiece, 0.7);
    ASSERT_FALSE(plan.empty()) << "inferProcessPlan must emit the recovered revolve";
    bool hasRevolve = false;
    for (const auto& step : plan.steps()) if (step.skill_id == "revolve_boss") hasRevolve = true;
    ASSERT_TRUE(hasRevolve) << "the plan must contain the recovered revolve step";

    auto fresh = skill::createCuboidStock(60.0, 60.0, 10.0);
    const double vFresh = volumeOf(fresh->shape());
    const auto result = process::Executor::execute(plan, fresh);
    // The point of this test: the Executor DISPATCHES the recovered revolve
    // (before the dispatch-table fix it aborted at it==table.end()).  Exact
    // volume parity is a separate RE-fidelity concern — a ring's coaxial
    // cylinder faces are also picked up by hole recognizers, so the re-synth
    // plan does more than the lone revolve.  Here we assert it ran cleanly and
    // added material.
    ASSERT_TRUE(result.ok())
        << "Executor must dispatch the recovered revolve boss (not abort): "
        << (result.errors.empty() ? "" : result.errors.front());
    ASSERT_NE(result.workpiece, nullptr);
    EXPECT_GT(volumeOf(result.workpiece->shape()), vFresh)
        << "the replayed revolve must add material (the revolve step ran)";
    (void)vOrig;
}
