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
#include "skills/drill_hole.hpp"
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
    // The Executor DISPATCHES the recovered revolve (before the dispatch-table
    // fix it aborted at it==table.end()), AND geometry parity holds: the ring's
    // coaxial inner wall used to be co-recognised as a separate bore and re-cut
    // (~ -157 mm3 drift), but the revolve subsumption now drops that inner-wall
    // hole — so the re-synth is the lone revolve and reproduces the original.
    ASSERT_TRUE(result.ok())
        << "Executor must dispatch the recovered revolve boss (not abort): "
        << (result.errors.empty() ? "" : result.errors.front());
    ASSERT_NE(result.workpiece, nullptr);
    EXPECT_GT(volumeOf(result.workpiece->shape()), vFresh)
        << "the replayed revolve must add material (the revolve step ran)";
    EXPECT_NEAR(volumeOf(result.workpiece->shape()), vOrig, 1.0)
        << "with inner-wall subsumption the re-synth reproduces the ring exactly";
}

// ORDERING: an additive boss must replay BEFORE subtractive machining — you
// build the boss on the base, then cut into it.  Before the Additive group the
// boss fell into Group::Unknown and was appended LAST (after the drill).
TEST(SketchReExecutor, AdditiveBossOrdersBeforeSubtractive)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 10.0);
    // Build a boss, then drill a hole through it.
    skill::extrude_boss_from_sketch::Input boss;
    boss.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    boss.polygon    = { { -8.0, -8.0 }, { 8.0, -8.0 }, { 8.0, 8.0 }, { -8.0, 8.0 } };
    boss.height_mm  = 6.0;
    auto wp1 = skill::extrude_boss_from_sketch::apply(*stock, boss);
    ASSERT_NE(wp1.workpiece, nullptr);
    skill::drill_hole::Input hole;
    hole.position_x_mm = 0.0; hole.position_y_mm = 0.0;
    hole.diameter_mm = 4.0; hole.depth_mm = 8.0;
    auto wp2 = skill::drill_hole::apply(*wp1.workpiece, hole);
    ASSERT_NE(wp2.workpiece, nullptr);

    const process::ProcessPlan plan = re::inferProcessPlan(*wp2.workpiece, 0.7);
    int idxBoss = -1, idxDrill = -1;
    for (int i = 0; i < static_cast<int>(plan.steps().size()); ++i) {
        const auto& id = plan.steps()[i].skill_id;
        if (id == "extrude_boss_from_sketch" && idxBoss < 0) idxBoss = i;
        if (id == "drill_hole" && idxDrill < 0)              idxDrill = i;
    }
    ASSERT_GE(idxBoss, 0)  << "the boss must be in the plan";
    ASSERT_GE(idxDrill, 0) << "the drill must be in the plan";
    EXPECT_LT(idxBoss, idxDrill)
        << "the additive boss must replay BEFORE the subtractive drill";
}
