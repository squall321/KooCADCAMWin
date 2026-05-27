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

#include <nlohmann/json.hpp>

#include <algorithm>
#include <string>
#include <vector>

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
