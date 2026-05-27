// @lat: [[engine/skills#Layer 5 LLM adapter]]
//
// Layer-5 PlanEditor — five-case slice-1 spec.
//
//   1. AppendOpAddsStep            — empty plan + AppendStep → 1-step plan.
//   2. ReplaceParamsChangesValues  — plan w/ drill_hole; ReplaceParams op
//                                    with new diameter; reflects in plan.
//   3. RemoveOpDeletesStep         — plan w/ 3 steps; RemoveStep idx 1;
//                                    plan has 2 steps with correct identity.
//   4. ValidatePlanSurfacesDFM     — plan w/ drill_hole(dia=0.5) (violates
//                                    DFM-002); validatePlan returns FAIL
//                                    with code "DFM-002".
//   5. DiffShowsChanges            — drill_hole-only vs mill_pocket-only;
//                                    diff returns ≥ 1 line mentioning the
//                                    change.

#include <gtest/gtest.h>

#include "adapt/EditOp.hpp"
#include "adapt/PlanEditor.hpp"

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"

#include <memory>
#include <string>
#include <vector>

using namespace koocadcam;
using nlohmann::json;

namespace {

// Build a fully-formed StepInvocation for a drill_hole at the given
// position / dimension.  Used by multiple test cases.
process::StepInvocation makeDrillStep(double x, double y,
                                       double dia, double depth)
{
    process::StepInvocation s;
    s.skill_id = "drill_hole";
    s.params = {
        { "position_x_mm", x },
        { "position_y_mm", y },
        { "diameter_mm",   dia },
        { "depth_mm",      depth },
        { "through_hole",  false },
        { "axis_dir",      json::array({ 0.0, 0.0, -1.0 }) },
    };
    return s;
}

process::StepInvocation makeMillPocketStep(double cx, double cy,
                                            double len, double wid,
                                            double depth)
{
    process::StepInvocation s;
    s.skill_id = "mill_rect_pocket";
    s.params = {
        { "center_x_mm",  cx },
        { "center_y_mm",  cy },
        { "length_mm",    len },
        { "width_mm",     wid },
        { "depth_mm",     depth },
        { "corner_r_mm",  1.0 },
        { "axis_dir",     json::array({ 0.0, 0.0, -1.0 }) },
    };
    return s;
}

}  // namespace

// ── 1. AppendOpAddsStep ──────────────────────────────────────────────────
TEST(LayerFivePlanEditor, AppendOpAddsStep)
{
    process::ProcessPlan empty;
    ASSERT_EQ(empty.size(), 0);

    const auto step = makeDrillStep(10.0, 10.0, 3.0, 5.0);
    const adapt::EditOp op = adapt::EditOp::appendStep(step);

    auto result = adapt::PlanEditor::apply(empty, op);

    ASSERT_EQ(result.size(), 1);
    EXPECT_EQ(result.steps()[0].skill_id, std::string("drill_hole"));
    EXPECT_DOUBLE_EQ(result.steps()[0].params["diameter_mm"].get<double>(), 3.0);

    // Source plan must remain untouched (functional API contract).
    EXPECT_EQ(empty.size(), 0);
}

// ── 2. ReplaceParamsChangesValues ───────────────────────────────────────
TEST(LayerFivePlanEditor, ReplaceParamsChangesValues)
{
    process::ProcessPlan plan;
    plan.append(makeDrillStep(25.0, 25.0, 3.0, 6.0));
    ASSERT_EQ(plan.size(), 1);

    // Build new params: diameter doubled, rest unchanged.
    json newParams = plan.steps()[0].params;
    newParams["diameter_mm"] = 6.0;

    auto edited = adapt::PlanEditor::apply(
        plan, adapt::EditOp::replaceParams(0, newParams));

    ASSERT_EQ(edited.size(), 1);
    EXPECT_DOUBLE_EQ(edited.steps()[0].params["diameter_mm"].get<double>(), 6.0);
    EXPECT_DOUBLE_EQ(edited.steps()[0].params["depth_mm"].get<double>(),   6.0)
        << "non-targeted params must survive ReplaceParams unchanged";

    // Source plan still has the original value.
    EXPECT_DOUBLE_EQ(plan.steps()[0].params["diameter_mm"].get<double>(), 3.0);
}

// ── 3. RemoveOpDeletesStep ──────────────────────────────────────────────
TEST(LayerFivePlanEditor, RemoveOpDeletesStep)
{
    process::ProcessPlan plan;
    plan.append(makeDrillStep(10.0, 10.0, 3.0, 5.0));        // 0 — kept
    plan.append(makeDrillStep(20.0, 20.0, 4.0, 5.0));        // 1 — removed
    plan.append(makeMillPocketStep(30.0, 30.0, 10.0, 10.0, 2.0)); // 2 — kept

    ASSERT_EQ(plan.size(), 3);

    auto edited = adapt::PlanEditor::apply(plan, adapt::EditOp::removeStep(1));

    ASSERT_EQ(edited.size(), 2);
    // Step 0 of the edited plan should still be the drill_hole at (10,10).
    EXPECT_EQ(edited.steps()[0].skill_id, std::string("drill_hole"));
    EXPECT_DOUBLE_EQ(edited.steps()[0].params["position_x_mm"].get<double>(), 10.0);
    // Step 1 of the edited plan should be the mill_rect_pocket
    // (i.e. the dropped middle drill_hole is gone).
    EXPECT_EQ(edited.steps()[1].skill_id, std::string("mill_rect_pocket"));
    EXPECT_DOUBLE_EQ(edited.steps()[1].params["center_x_mm"].get<double>(), 30.0);

    // Original is unchanged.
    EXPECT_EQ(plan.size(), 3);
}

// ── 4. ValidatePlanSurfacesDFM ──────────────────────────────────────────
TEST(LayerFivePlanEditor, ValidatePlanSurfacesDFM)
{
    process::ProcessPlan plan;
    // diameter 0.5 mm < min 0.8 mm → DFM-002 must fire.
    plan.append(makeDrillStep(25.0, 25.0, 0.5, 3.0));

    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);
    ASSERT_TRUE(stock != nullptr);

    auto report = adapt::PlanEditor::validatePlan(plan, stock);

    EXPECT_FALSE(report.passed) << "DFM-002 violation must turn passed=false";

    bool foundDfm002 = false;
    bool taggedWithStepIndex = false;
    for (const auto& f : report.findings) {
        if (f.code == "DFM-002") {
            foundDfm002 = true;
            if (f.message.find("[step 0]") != std::string::npos) {
                taggedWithStepIndex = true;
            }
        }
    }
    EXPECT_TRUE(foundDfm002) << "validatePlan must surface DFM-002 for sub-min diameter";
    EXPECT_TRUE(taggedWithStepIndex)
        << "findings must be tagged with their step index for traceability";
}

// ── 5. DiffShowsChanges ─────────────────────────────────────────────────
TEST(LayerFivePlanEditor, DiffShowsChanges)
{
    process::ProcessPlan planA;
    planA.append(makeDrillStep(20.0, 20.0, 3.0, 5.0));

    process::ProcessPlan planB;
    planB.append(makeMillPocketStep(20.0, 20.0, 10.0, 10.0, 3.0));

    const auto changes = adapt::PlanEditor::diff(planA, planB);
    ASSERT_FALSE(changes.empty()) << "diff must report at least one change";

    // The change should mention the skill switch in at least one line.
    bool mentionsSkillChange = false;
    for (const auto& line : changes) {
        if (line.find("drill_hole") != std::string::npos &&
            line.find("mill_rect_pocket") != std::string::npos)
        {
            mentionsSkillChange = true;
            break;
        }
    }
    if (!mentionsSkillChange) {
        // Dump all diff lines so the failure is debuggable.
        std::string joined;
        for (const auto& l : changes) joined += "  " + l + "\n";
        ADD_FAILURE() << "diff should name the old + new skill_id when "
                         "step 0's skill changes; got:\n" << joined;
    }
}
