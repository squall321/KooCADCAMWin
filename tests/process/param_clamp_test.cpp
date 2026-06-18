// @lat: [[engine/skills#param-clamp]]
//
// B3.4 — recovered-parameter clamp.  Proves that an over-specified RE candidate
// (a hollow_cavity whose recovered wall_thickness exceeds the stock) crashes
// apply() when dispatched raw, but is clamped to a valid value and succeeds when
// it flows through the Executor's clamp hook.

#include <gtest/gtest.h>

#include "process/ParamClamp.hpp"
#include "process/Executor.hpp"
#include "process/ProcessPlan.hpp"
#include "process/StepInvocation.hpp"
#include "skills/Skill.hpp"
#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/hollow_cavity.hpp"

using namespace koocadcam;
using json = nlohmann::json;

namespace {

// An over-specified hollow_cavity: wall 60 mm and depth 80 mm are both larger
// than a 100x100x50 stock can hold (half the XY extent is 50, the Z span 50).
json overspecCavity()
{
    return json{
        { "entry_face", "top" },
        { "wall_thickness_mm", 60.0 },
        { "depth_mm", 80.0 },
    };
}

}  // namespace

// clampParams rewrites the impossible values to the nearest valid ones.
TEST(ParamClamp, ClampsOverspecHollowCavity)
{
    auto stock = skill::createCuboidStock(100.0, 100.0, 50.0);
    const json c = process::clampParams("hollow_cavity", overspecCavity(), *stock);

    EXPECT_NEAR(c.value("wall_thickness_mm", 0.0), 0.45 * 100.0, 1e-6);   // 45
    EXPECT_NEAR(c.value("depth_mm", 0.0), 50.0 - 0.4, 1e-6);              // 49.6
    // The clamped cavity is now positive: 100 - 2*45 = 10 mm > 0.
}

// A thin-but-VALID wall (watch-grade 0.35mm, only a DFM warning) is preserved —
// the clamp fixes crash-class input only, not DFM warnings.
TEST(ParamClamp, PreservesValidThinWall)
{
    auto stock = skill::createCuboidStock(100.0, 100.0, 50.0);
    const json in = json{
        { "entry_face", "top" }, { "wall_thickness_mm", 0.35 }, { "depth_mm", 5.0 } };
    const json c = process::clampParams("hollow_cavity", in, *stock);
    EXPECT_NEAR(c.value("wall_thickness_mm", 0.0), 0.35, 1e-9)
        << "a valid thin wall must NOT be floored up";
}

// A skill with no clamp rule is a pass-through (params returned unchanged).
TEST(ParamClamp, NoRuleIsPassThrough)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 20.0);
    const json in = json{ { "diameter_mm", 8.0 }, { "position_x_mm", 25.0 } };
    EXPECT_EQ(process::clampParams("drill_hole", in, *stock), in);
}

// Below the clamp ceiling, params pass through untouched (no over-clamping).
TEST(ParamClamp, ValidCavityUnchanged)
{
    auto stock = skill::createCuboidStock(100.0, 100.0, 50.0);
    const json in = json{
        { "entry_face", "top" }, { "wall_thickness_mm", 3.0 }, { "depth_mm", 10.0 } };
    const json c = process::clampParams("hollow_cavity", in, *stock);
    EXPECT_NEAR(c.value("wall_thickness_mm", 0.0), 3.0, 1e-6);
    EXPECT_NEAR(c.value("depth_mm", 0.0), 10.0, 1e-6);
}

// Raw apply() of the over-spec cavity THROWS — establishing the hazard the
// clamp removes.
TEST(ParamClamp, RawOverspecApplyThrows)
{
    auto stock = skill::createCuboidStock(100.0, 100.0, 50.0);
    skill::hollow_cavity::Input in;
    in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.wall_thickness_mm = 60.0;
    in.depth_mm = 80.0;
    EXPECT_THROW(skill::hollow_cavity::apply(*stock, in), skill::SkillError);
}

// End-to-end: the SAME over-spec step, executed through the Executor, is clamped
// and applies successfully (no crash, no skipped step).
TEST(ParamClamp, ExecutorClampsAndSucceeds)
{
    auto stock = skill::createCuboidStock(100.0, 100.0, 50.0);
    process::ProcessPlan plan;
    process::StepInvocation step;
    step.skill_id = "hollow_cavity";
    step.params   = overspecCavity();
    plan.append(step);

    const auto result = process::Executor::execute(plan, stock);
    EXPECT_TRUE(result.ok())
        << "Executor should clamp the over-spec cavity and apply it; errors="
        << (result.errors.empty() ? "" : result.errors.front());
    ASSERT_NE(result.workpiece, nullptr);
    // The cavity actually cut material: a bare cuboid has 6 faces; a cavity
    // adds its floor + walls, so the step ran (was not skipped).
    EXPECT_GT(result.workpiece->faceCount(), 6);
}
