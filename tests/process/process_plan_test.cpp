// @lat: [[engine/skills#Layer 3 ProcessPlan tests]]
//
// Layer-3 ProcessPlan + Executor tests (slice-1 scope):
//
//   1. AppendAndRetrieve  — build a 3-step plan, verify size + step access.
//   2. JsonRoundTrip      — toJson → fromJson reproduces the plan.
//   3. FileRoundTrip      — writeFile → readFile reproduces the plan.
//   4. ExecuteSimple      — single drill_hole step removes ~π r² h volume.
//   5. ExecuteCompound    — drill_hole + counterbore on the same stock
//                           produce 2 FeatureSignatures in result.signatures.

#include <gtest/gtest.h>

#include "process/Executor.hpp"
#include "process/ProcessPlan.hpp"
#include "process/StepInvocation.hpp"

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

#include <cmath>
#include <filesystem>

namespace fs = std::filesystem;
using namespace koocadcam;
using nlohmann::json;

namespace {

double volumeOf(const TopoDS_Shape& s)
{
    GProp_GProps p;
    BRepGProp::VolumeProperties(s, p);
    return p.Mass();
}

// Build a typical drill_hole step (top-face, params shape mirrors
// FeatureSignature.params).
process::StepInvocation makeDrillStep(double x, double y, double dia, double depth,
                                      bool through = false,
                                      const std::string& note = "")
{
    process::StepInvocation s;
    s.skill_id = "drill_hole";
    s.params   = {
        { "entry_face",     "top" },
        { "position_x_mm",  x },
        { "position_y_mm",  y },
        { "diameter_mm",    dia },
        { "depth_mm",       depth },
        { "through_hole",   through },
    };
    s.note = note;
    return s;
}

}  // namespace

// ─── 1. AppendAndRetrieve ────────────────────────────────────────────────
TEST(ProcessPlan, AppendAndRetrieve)
{
    process::ProcessPlan plan;

    EXPECT_EQ(plan.size(), 0);
    EXPECT_TRUE(plan.empty());

    const int i0 = plan.append(makeDrillStep(10.0, 10.0, 3.0, 5.0, false, "first"));
    const int i1 = plan.append(makeDrillStep(20.0, 20.0, 4.0, 6.0));
    process::StepInvocation cs;
    cs.skill_id = "counterbore";
    cs.params = {
        { "entry_face",     "top" },
        { "position_x_mm",  30.0 },
        { "position_y_mm",  30.0 },
        { "pilot_dia_mm",   3.0 },
        { "pilot_depth_mm", 5.0 },
        { "seat_dia_mm",    6.0 },
        { "seat_depth_mm",  2.0 },
    };
    cs.depends_on = { 0 };
    const int i2 = plan.append(cs);

    EXPECT_EQ(i0, 0);
    EXPECT_EQ(i1, 1);
    EXPECT_EQ(i2, 2);
    EXPECT_EQ(plan.size(), 3);
    EXPECT_FALSE(plan.empty());

    const auto& steps = plan.steps();
    EXPECT_EQ(steps[0].skill_id, "drill_hole");
    EXPECT_EQ(steps[0].note,     "first");
    EXPECT_EQ(steps[2].skill_id, "counterbore");
    ASSERT_EQ(steps[2].depends_on.size(), 1u);
    EXPECT_EQ(steps[2].depends_on[0], 0);

    // replaceParams happy path + out-of-range.
    json newParams = steps[1].params;
    newParams["diameter_mm"] = 5.0;
    EXPECT_TRUE(plan.replaceParams(1, newParams));
    EXPECT_DOUBLE_EQ(plan.steps()[1].params["diameter_mm"].get<double>(), 5.0);

    EXPECT_FALSE(plan.replaceParams(99,  json::object()));
    EXPECT_FALSE(plan.replaceParams(-1,  json::object()));
}

// ─── 2. JsonRoundTrip ────────────────────────────────────────────────────
TEST(ProcessPlan, JsonRoundTrip)
{
    process::ProcessPlan plan;
    plan.append(makeDrillStep(10.0, 10.0, 3.0, 5.0, false, "drill1"));
    plan.append(makeDrillStep(20.0, 20.0, 4.0, 7.0, true,  "through"));

    process::StepInvocation pocket;
    pocket.skill_id   = "mill_rect_pocket";
    pocket.params     = {
        { "entry_face",  "top" },
        { "center_x_mm", 25.0 },
        { "center_y_mm", 25.0 },
        { "length_mm",   10.0 },
        { "width_mm",    6.0 },
        { "depth_mm",    3.0 },
        { "corner_r_mm", 1.0 },
    };
    pocket.depends_on = { 0, 1 };
    pocket.note       = "finishing pocket";
    plan.append(pocket);

    const json j = plan.toJson();
    ASSERT_TRUE(j.contains("steps"));
    ASSERT_TRUE(j.contains("version"));
    EXPECT_EQ(j["steps"].size(), 3u);

    process::ProcessPlan restored = process::ProcessPlan::fromJson(j);
    ASSERT_EQ(restored.size(), 3);

    EXPECT_EQ(restored.steps()[0].skill_id,            "drill_hole");
    EXPECT_EQ(restored.steps()[0].note,                "drill1");
    EXPECT_DOUBLE_EQ(restored.steps()[0].params["diameter_mm"].get<double>(), 3.0);

    EXPECT_EQ(restored.steps()[1].params["through_hole"].get<bool>(), true);

    EXPECT_EQ(restored.steps()[2].skill_id,            "mill_rect_pocket");
    EXPECT_EQ(restored.steps()[2].note,                "finishing pocket");
    ASSERT_EQ(restored.steps()[2].depends_on.size(), 2u);
    EXPECT_EQ(restored.steps()[2].depends_on[0], 0);
    EXPECT_EQ(restored.steps()[2].depends_on[1], 1);

    // toJson(fromJson(toJson(plan))) should be identical to the first toJson.
    EXPECT_EQ(restored.toJson(), j);
}

// ─── 3. FileRoundTrip ────────────────────────────────────────────────────
TEST(ProcessPlan, FileRoundTrip)
{
    process::ProcessPlan plan;
    plan.append(makeDrillStep(15.0, 15.0, 3.0, 5.0));
    plan.append(makeDrillStep(35.0, 35.0, 4.0, 8.0));

    const fs::path path = fs::temp_directory_path() / "koo_process_plan_test.json";
    if (fs::exists(path)) fs::remove(path);

    ASSERT_TRUE(plan.writeFile(path));
    ASSERT_TRUE(fs::exists(path));

    auto loadedOpt = process::ProcessPlan::readFile(path);
    ASSERT_TRUE(loadedOpt.has_value());

    const auto& loaded = *loadedOpt;
    ASSERT_EQ(loaded.size(), 2);
    EXPECT_EQ(loaded.steps()[0].skill_id, "drill_hole");
    EXPECT_DOUBLE_EQ(loaded.steps()[0].params["position_x_mm"].get<double>(), 15.0);
    EXPECT_DOUBLE_EQ(loaded.steps()[1].params["diameter_mm"].get<double>(),    4.0);
    EXPECT_EQ(loaded.toJson(), plan.toJson());

    // readFile on a missing file → nullopt.
    const fs::path missing = fs::temp_directory_path() / "koo_no_such_plan_xyz.json";
    if (fs::exists(missing)) fs::remove(missing);
    auto miss = process::ProcessPlan::readFile(missing);
    EXPECT_FALSE(miss.has_value());

    fs::remove(path);
}

// ─── 4. ExecuteSimple ────────────────────────────────────────────────────
//
// Spec scenario: drill_hole(top, 25, 25, dia=3, depth=5) on a 50×50×10 cuboid.
TEST(ProcessExecutor, ExecuteSimple)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);
    ASSERT_TRUE(stock != nullptr);
    const double v0 = volumeOf(stock->shape());

    process::ProcessPlan plan;
    plan.append(makeDrillStep(25.0, 25.0, 3.0, 5.0));

    auto result = process::Executor::execute(plan, stock);
    ASSERT_TRUE(result.ok()) << "errors=" << (result.errors.empty() ? "<none>" : result.errors[0]);
    ASSERT_TRUE(result.workpiece != nullptr);
    EXPECT_EQ(result.failedAtStep, -1);
    EXPECT_EQ(result.signatures.size(), 1u);
    EXPECT_EQ(result.signatures.front().skill_id, "drill_hole");

    // Volume removed ≈ π × 1.5² × 5.0 ≈ 35.34.
    const double expectedRemoved = M_PI * 1.5 * 1.5 * 5.0;
    const double actualRemoved = v0 - volumeOf(result.workpiece->shape());
    EXPECT_NEAR(actualRemoved, expectedRemoved, expectedRemoved * 0.05);

    // Feature history on the resulting workpiece records the step.
    EXPECT_EQ(result.workpiece->features().size(), 1u);
}

// ─── 5. ExecuteCompound ──────────────────────────────────────────────────
//
// drill_hole at (15,15) then counterbore at (35,35).  Two independent
// features so neither perturbs the other's geometry; the executor's job is
// just to chain them through the same plan.
TEST(ProcessExecutor, ExecuteCompound)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    process::ProcessPlan plan;
    plan.append(makeDrillStep(15.0, 15.0, 3.0, 5.0, false, "pilot hole"));

    process::StepInvocation cb;
    cb.skill_id = "counterbore";
    cb.params = {
        { "entry_face",     "top" },
        { "position_x_mm",  35.0 },
        { "position_y_mm",  35.0 },
        { "pilot_dia_mm",   3.0 },
        { "pilot_depth_mm", 6.0 },
        { "seat_dia_mm",    6.0 },
        { "seat_depth_mm",  2.0 },
    };
    cb.note = "screw seat";
    plan.append(cb);

    auto result = process::Executor::execute(plan, stock);
    ASSERT_TRUE(result.ok()) << "errors="
        << (result.errors.empty() ? "<none>" : result.errors[0]);
    ASSERT_TRUE(result.workpiece != nullptr);
    EXPECT_EQ(result.failedAtStep, -1);

    ASSERT_EQ(result.signatures.size(), 2u);
    EXPECT_EQ(result.signatures[0].skill_id, "drill_hole");
    EXPECT_EQ(result.signatures[1].skill_id, "counterbore");

    // The Executor mirrors its signatures accumulator into each step's
    // output Workpiece via setFeatures(), so wp.features() reflects the full
    // chain history.
    EXPECT_EQ(result.workpiece->features().size(), 2u);
}

// ─── 6. (bonus) Unknown skill_id → failedAtStep set, partial wp preserved ─
TEST(ProcessExecutor, UnknownSkillFailsCleanly)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    process::ProcessPlan plan;
    plan.append(makeDrillStep(25.0, 25.0, 3.0, 5.0));
    process::StepInvocation bogus;
    bogus.skill_id = "no_such_skill_xyz";
    bogus.params   = json::object();
    plan.append(bogus);

    auto result = process::Executor::execute(plan, stock);
    EXPECT_FALSE(result.ok());
    EXPECT_EQ(result.failedAtStep, 1);
    EXPECT_EQ(result.signatures.size(), 1u);   // step 0 succeeded
    ASSERT_TRUE(result.workpiece != nullptr);  // partial workpiece preserved
}

// ─── 7. ExecuteAllSkillCategories ────────────────────────────────────────
//
// Smoke test for the expanded dispatch table (slice-2): exercise FIVE distinct
// skill categories in a single plan to confirm each is wired up correctly and
// that the full chain history propagates via Workpiece::setFeatures().
//
// Categories covered:
//   1. face_milling             (subtractive — planar skim, first so all
//                                later operations work off the trimmed top)
//   2. chamfer_edge             (additive bevel on the bottom rim, Z = 0)
//   3. fillet_edge              (additive blend on the post-skim top rim)
//   4. drill_hole               (subtractive — drill after rim is filleted)
//   5. mill_circular_pocket     (subtractive — pocket on far side)
//
// We intentionally use simple, well-separated geometry so no skill rejects
// its input (entry_face resolves, fillet edges exist, etc.).
TEST(ProcessExecutor, ExecuteAllSkillCategories)
{
    auto stock = skill::createCuboidStock(80.0, 80.0, 15.0);
    ASSERT_TRUE(stock != nullptr);

    process::ProcessPlan plan;

    // 1) face_milling — skim 0.5 mm off the top.  Done FIRST so subsequent
    //    drill / pocket / fillet work on a clean planar top at Z = 14.5.
    process::StepInvocation fm;
    fm.skill_id = "face_milling";
    fm.params = {
        { "entry_face", "top" },
        { "depth_mm",   0.5 },
    };
    plan.append(fm);

    // 2) chamfer_edge — bevel the bottom rim at Z = 0 BEFORE any drilling
    //    or pocketing so we know the 4 bottom edges are unmodified.
    process::StepInvocation ce;
    ce.skill_id = "chamfer_edge";
    ce.params = {
        { "edges_at_z_mm",  0.0 },
        { "tolerance_mm",   0.05 },
        { "chamfer_size_mm", 0.5 },
        { "angle_deg",      45.0 },
    };
    plan.append(ce);

    // 3) fillet_edge — round the top-rim outline at the post-skim Z = 14.5
    //    BEFORE drilling, so only the outer 4 rim edges are filleted (no
    //    interior drill rim to confuse the fillet builder).
    process::StepInvocation fe;
    fe.skill_id = "fillet_edge";
    fe.params = {
        { "edges_at_z_mm", 14.5 },
        { "tolerance_mm",  0.05 },
        { "radius_mm",     0.5 },
    };
    plan.append(fe);

    // 4) drill_hole — small hole, well clear of the rim, after fillet to
    //    keep the rim edges out of the way.
    process::StepInvocation dh;
    dh.skill_id = "drill_hole";
    dh.params = {
        { "entry_face",     "top" },
        { "position_x_mm",  20.0 },
        { "position_y_mm",  20.0 },
        { "diameter_mm",    4.0 },
        { "depth_mm",       6.0 },
        { "through_hole",   false },
    };
    plan.append(dh);

    // 5) mill_circular_pocket — wide pocket on the far side.
    process::StepInvocation mcp;
    mcp.skill_id = "mill_circular_pocket";
    mcp.params = {
        { "entry_face",    "top" },
        { "position_x_mm", 60.0 },
        { "position_y_mm", 60.0 },
        { "diameter_mm",   8.0 },
        { "depth_mm",      3.0 },
    };
    plan.append(mcp);

    auto result = process::Executor::execute(plan, stock);
    ASSERT_TRUE(result.ok())
        << "errors=" << (result.errors.empty() ? "<none>" : result.errors[0]);
    EXPECT_EQ(result.failedAtStep, -1);
    ASSERT_EQ(result.signatures.size(), 5u);
    ASSERT_TRUE(result.workpiece != nullptr);

    // Full history is mirrored into the final workpiece's feature list.
    EXPECT_EQ(result.workpiece->features().size(), 5u);

    EXPECT_EQ(result.signatures[0].skill_id, "face_milling");
    EXPECT_EQ(result.signatures[1].skill_id, "chamfer_edge");
    EXPECT_EQ(result.signatures[2].skill_id, "fillet_edge");
    EXPECT_EQ(result.signatures[3].skill_id, "drill_hole");
    EXPECT_EQ(result.signatures[4].skill_id, "mill_circular_pocket");
}
