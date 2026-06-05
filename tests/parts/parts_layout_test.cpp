// @lat: [[engine/skills#Layer 6 Parts layout]]
//
// PartsLayout + DatumGraph tests:
//
//   1. AddAndQuery               — add 3 Parts; verify count + findById.
//   2. DiffEmpty                 — identical layouts → empty diff.
//   3. DiffMoved                 — same part at different positions →
//                                  exactly one "moved" entry.
//   4. DiffAddedAndRemoved       — A={x,y}, B={y,z} → 1 added + 1 removed.
//   5. HeuristicMatchesByPosition — drill_hole at (10, 20, 0) matches part
//                                   centred near (10, 20) → DatumGraph edge.

#include <gtest/gtest.h>

#include "parts/DatumGraph.hpp"
#include "parts/Part.hpp"
#include "parts/PartsLayout.hpp"

#include "process/ProcessPlan.hpp"
#include "process/StepInvocation.hpp"

#include <gp_Ax1.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>
#include <gp_Trsf.hxx>

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using namespace koocadcam::parts;
using koocadcam::process::ProcessPlan;
using koocadcam::process::StepInvocation;
using nlohmann::json;

namespace {

// Build a Part with a pre-computed AABB; no OCCT shape needed because the
// AABB drives every downstream calculation we care about in these tests.
Part makeAabbPart(const std::string& id,
                  double cx, double cy, double cz,
                  double sx = 4.0, double sy = 4.0, double sz = 2.0)
{
    Part p;
    p.id   = id;
    p.xMin = cx - sx * 0.5;  p.xMax = cx + sx * 0.5;
    p.yMin = cy - sy * 0.5;  p.yMax = cy + sy * 0.5;
    p.zMin = cz - sz * 0.5;  p.zMax = cz + sz * 0.5;
    return p;
}

}  // namespace

// ─── 1. AddAndQuery ──────────────────────────────────────────────────────
TEST(PartsLayout, AddAndQuery)
{
    PartsLayout layout;
    layout.addPart(makeAabbPart("pcb_main",    0.0, 0.0, 0.0));
    layout.addPart(makeAabbPart("battery",   20.0, 0.0, 0.0));
    layout.addPart(makeAabbPart("display",   10.0, 5.0, 1.0));

    EXPECT_EQ(layout.parts().size(), 3u);

    ASSERT_NE(layout.findById("pcb_main"), nullptr);
    EXPECT_EQ(layout.findById("pcb_main")->id, "pcb_main");

    ASSERT_NE(layout.findById("battery"), nullptr);
    EXPECT_DOUBLE_EQ(layout.findById("battery")->centerX(), 20.0);

    ASSERT_NE(layout.findById("display"), nullptr);
    EXPECT_DOUBLE_EQ(layout.findById("display")->centerY(), 5.0);

    EXPECT_EQ(layout.findById("no_such_part"), nullptr);
}

// ─── 2. DiffEmpty ────────────────────────────────────────────────────────
TEST(PartsLayout, DiffEmpty)
{
    PartsLayout a, b;
    a.addPart(makeAabbPart("pcb",   0.0,  0.0, 0.0));
    a.addPart(makeAabbPart("batt", 20.0,  0.0, 0.0));
    b.addPart(makeAabbPart("pcb",   0.0,  0.0, 0.0));
    b.addPart(makeAabbPart("batt", 20.0,  0.0, 0.0));

    auto diff = a.diff(b);
    EXPECT_TRUE(diff.empty()) << "Identical layouts must produce empty diff "
                                 "(got " << diff.size() << " entries)";
}

// ─── 3. DiffMoved ────────────────────────────────────────────────────────
TEST(PartsLayout, DiffMoved)
{
    PartsLayout a, b;
    a.addPart(makeAabbPart("pcb", 0.0, 0.0, 0.0));
    // Same id, same size, different position (shift +5 mm in X).
    b.addPart(makeAabbPart("pcb", 5.0, 0.0, 0.0));

    auto diff = a.diff(b);
    ASSERT_EQ(diff.size(), 1u);
    EXPECT_EQ(diff[0].part_id, "pcb");
    EXPECT_EQ(diff[0].kind,    "moved");
    ASSERT_TRUE(diff[0].details.contains("delta"));
    EXPECT_DOUBLE_EQ(diff[0].details["delta"][0].get<double>(), 5.0);
    EXPECT_DOUBLE_EQ(diff[0].details["delta"][1].get<double>(), 0.0);
    EXPECT_DOUBLE_EQ(diff[0].details["delta"][2].get<double>(), 0.0);
    EXPECT_NEAR(diff[0].details["magnitude"].get<double>(), 5.0, 1e-9);
}

// ─── 4. DiffAddedAndRemoved ──────────────────────────────────────────────
TEST(PartsLayout, DiffAddedAndRemoved)
{
    PartsLayout a, b;
    a.addPart(makeAabbPart("x", 0.0,  0.0, 0.0));
    a.addPart(makeAabbPart("y", 10.0, 0.0, 0.0));
    b.addPart(makeAabbPart("y", 10.0, 0.0, 0.0));
    b.addPart(makeAabbPart("z", 20.0, 0.0, 0.0));

    auto diff = a.diff(b);
    ASSERT_EQ(diff.size(), 2u);

    // Order: removed entries come from this's iteration first, then added.
    int removedCount = 0, addedCount = 0;
    for (const auto& d : diff) {
        if (d.kind == "removed") {
            ++removedCount;
            EXPECT_EQ(d.part_id, "x");
        }
        else if (d.kind == "added") {
            ++addedCount;
            EXPECT_EQ(d.part_id, "z");
        } else {
            ADD_FAILURE() << "unexpected diff kind: " << d.kind;
        }
    }
    EXPECT_EQ(removedCount, 1);
    EXPECT_EQ(addedCount,   1);
}

// ─── 5. HeuristicMatchesByPosition ───────────────────────────────────────
TEST(DatumGraph, HeuristicMatchesByPosition)
{
    // Layout: one part (pcb_main) centred near (10, 20, 0).
    PartsLayout layout;
    layout.addPart(makeAabbPart("pcb_main",  10.0, 20.0, 0.0, 8.0, 8.0, 2.0));
    layout.addPart(makeAabbPart("battery",   50.0, 50.0, 0.0));   // far away

    // Plan: one drill_hole at (10, 20, 0).
    ProcessPlan plan;
    StepInvocation drill;
    drill.skill_id = "drill_hole";
    drill.params   = {
        { "entry_face",    "top" },
        { "position_x_mm", 10.0 },
        { "position_y_mm", 20.0 },
        { "diameter_mm",    3.0 },
        { "depth_mm",       5.0 },
    };
    plan.append(drill);

    DatumGraph graph = extractHeuristicDependencies(plan, layout, /*tol=*/5.0);

    // Step 0 must have an edge to "pcb_main", not to "battery".
    const auto parts0 = graph.partsForStep(0);
    ASSERT_EQ(parts0.size(), 1u);
    EXPECT_EQ(parts0[0], "pcb_main");

    // Reverse lookup.
    const auto deps = graph.stepsDependentOn("pcb_main");
    ASSERT_EQ(deps.size(), 1u);
    EXPECT_EQ(deps[0], 0);

    EXPECT_TRUE(graph.partsForStep(99).empty());
    EXPECT_TRUE(graph.stepsDependentOn("nope").empty());

    // Now: move pcb_main 7 mm in X.  invalidatedSteps must return {0}.
    PartsLayout shifted;
    shifted.addPart(makeAabbPart("pcb_main", 17.0, 20.0, 0.0, 8.0, 8.0, 2.0));
    shifted.addPart(makeAabbPart("battery",  50.0, 50.0, 0.0));

    auto diff = layout.diff(shifted);
    ASSERT_GE(diff.size(), 1u);
    auto invalid = graph.invalidatedSteps(diff);
    ASSERT_EQ(invalid.size(), 1u);
    EXPECT_EQ(invalid[0], 0);
}

// ─── 6. (bonus) DatumGraph JSON round-trip ───────────────────────────────
TEST(DatumGraph, JsonRoundTrip)
{
    DatumGraph g;
    g.addDependency(0, "pcb_main");
    g.addDependency(0, "battery");      // multi-part dep
    g.addDependency(0, "pcb_main");     // dedupe — should remain size 2
    g.addDependency(3, "display");

    auto j = g.toJson();
    auto h = DatumGraph::fromJson(j);

    EXPECT_EQ(h.partsForStep(0).size(), 2u);
    EXPECT_EQ(h.partsForStep(3).size(), 1u);
    EXPECT_EQ(h.partsForStep(3)[0], "display");
    EXPECT_EQ(h.stepsDependentOn("pcb_main").size(), 1u);
}

// ─── 7. (bonus) PartsLayout JSON round-trip ──────────────────────────────
TEST(PartsLayout, JsonRoundTrip)
{
    PartsLayout layout;
    layout.addPart(makeAabbPart("pcb",  0.0, 0.0, 0.0));
    layout.addPart(makeAabbPart("batt", 20.0, 5.0, 1.0, 10.0, 6.0, 3.0));

    auto j = layout.toJson();
    auto restored = PartsLayout::fromJson(j);
    ASSERT_EQ(restored.parts().size(), 2u);
    EXPECT_EQ(restored.findById("pcb")->id, "pcb");
    EXPECT_DOUBLE_EQ(restored.findById("batt")->centerX(), 20.0);
    EXPECT_DOUBLE_EQ(restored.findById("batt")->sizeX(), 10.0);
}

// ─── 8. reframePlanForMoves shifts ONLY the steps tied to a moved part ────
//
// The deterministic adapt core: a step anchored to a part that moves follows
// the part's centre delta; a step anchored to a stationary part is untouched.
TEST(PartsLayout, ReframePlanForMovesShiftsDependentSteps)
{
    ProcessPlan plan;
    {   StepInvocation s; s.skill_id = "drill_hole";
        s.params = { { "position_x_mm", 10.0 }, { "position_y_mm", 10.0 },
                     { "diameter_mm", 4.0 } };
        plan.append(s); }
    {   StepInvocation s; s.skill_id = "drill_hole";
        s.params = { { "position_x_mm", 40.0 }, { "position_y_mm", 10.0 },
                     { "diameter_mm", 4.0 } };
        plan.append(s); }

    PartsLayout before;
    before.addPart(makeAabbPart("pcb",  10.0, 10.0, 0.0));
    before.addPart(makeAabbPart("batt", 40.0, 10.0, 0.0));

    // Only the PCB moves: +7 mm X, −3 mm Y.  Battery stays put.
    PartsLayout after;
    after.addPart(makeAabbPart("pcb",  17.0,  7.0, 0.0));
    after.addPart(makeAabbPart("batt", 40.0, 10.0, 0.0));

    const ProcessPlan out = reframePlanForMoves(plan, before, after, 1.0);
    ASSERT_EQ(out.size(), 2);

    // Step 0 anchored to the PCB → shifted by (+7, −3).
    EXPECT_DOUBLE_EQ(out.steps()[0].params["position_x_mm"].get<double>(), 17.0);
    EXPECT_DOUBLE_EQ(out.steps()[0].params["position_y_mm"].get<double>(),  7.0);
    // Step 1 anchored to the (stationary) battery → unchanged.
    EXPECT_DOUBLE_EQ(out.steps()[1].params["position_x_mm"].get<double>(), 40.0);
    EXPECT_DOUBLE_EQ(out.steps()[1].params["position_y_mm"].get<double>(), 10.0);
}

// ─── 9. reframePlanForMoves carries features through a part ROTATION ───────
//
// Not just translation: a part that rotates about its centre drags its
// anchored features around with it.  Plate centred at origin, rotated +90°
// about Z via its placement → each hole (x,y) maps to (−y, x).
TEST(PartsLayout, ReframeRotatesDependentSteps)
{
    ProcessPlan plan;
    {   StepInvocation s; s.skill_id = "drill_hole";
        s.params = { { "position_x_mm", 10.0 }, { "position_y_mm", 0.0 },
                     { "diameter_mm", 3.0 } };
        plan.append(s); }
    {   StepInvocation s; s.skill_id = "drill_hole";
        s.params = { { "position_x_mm", 0.0 }, { "position_y_mm", 5.0 },
                     { "diameter_mm", 3.0 } };
        plan.append(s); }

    PartsLayout before;
    before.addPart(makeAabbPart("plate", 0.0, 0.0, 0.0, 40.0, 40.0, 4.0));

    // Same centre, but rotated +90° about Z (placement orientation change).
    PartsLayout after;
    {   Part p = makeAabbPart("plate", 0.0, 0.0, 0.0, 40.0, 40.0, 4.0);
        gp_Trsf t;
        t.SetRotation(gp_Ax1(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1)), M_PI / 2.0);
        p.placement = t;
        after.addPart(p); }

    const ProcessPlan out = reframePlanForMoves(plan, before, after, 1.0);
    ASSERT_EQ(out.size(), 2);

    // +90° about Z: (x,y) -> (-y, x).  A(10,0)->(0,10); B(0,5)->(-5,0).
    EXPECT_NEAR(out.steps()[0].params["position_x_mm"].get<double>(),  0.0, 1e-6);
    EXPECT_NEAR(out.steps()[0].params["position_y_mm"].get<double>(), 10.0, 1e-6);
    EXPECT_NEAR(out.steps()[1].params["position_x_mm"].get<double>(), -5.0, 1e-6);
    EXPECT_NEAR(out.steps()[1].params["position_y_mm"].get<double>(),  0.0, 1e-6);
}
