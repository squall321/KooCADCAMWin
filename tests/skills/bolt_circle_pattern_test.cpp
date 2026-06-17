// @lat: [[engine/skills#bolt_circle_pattern]]
//
// The generative side of the B1.2 grammar: a recognised bolt circle becomes a
// replayable, EDITABLE step.  These tests close the recognise -> edit ->
// regenerate loop: apply drills the ring; editing hole_count regenerates a
// different ring; a STEP round-trip re-recognises it; and the Executor can
// dispatch it from a plain ProcessPlan.

#include <gtest/gtest.h>

#include "skills/bolt_circle_pattern.hpp"
#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "re/Recognizer.hpp"
#include "io/StepIO.hpp"
#include "process/Executor.hpp"
#include "process/ProcessPlan.hpp"
#include "process/StepInvocation.hpp"

#include <BRepCheck_Analyzer.hxx>
#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>
#include <gp_Dir.hxx>

#include <nlohmann/json.hpp>

#include <filesystem>
#include <string>

using namespace koocadcam;
using json = nlohmann::json;

namespace {

double volumeOf(const TopoDS_Shape& s)
{
    GProp_GProps p;
    BRepGProp::VolumeProperties(s, p);
    return p.Mass();
}

skill::bolt_circle_pattern::Input ring(int n)
{
    skill::bolt_circle_pattern::Input in;
    in.entry_face         = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.hole_count         = n;
    in.bolt_circle_dia_mm = 40.0;
    in.hole_dia_mm        = 5.0;
    in.center_x_mm        = 40.0;
    in.center_y_mm        = 40.0;
    in.axis_dir           = gp_Dir(0, 0, -1);
    in.through_hole       = true;
    return in;
}

}  // namespace

// 1. apply() drills a valid ring that removes material.
TEST(BoltCirclePattern, GeneratesValidRing)
{
    auto stock = skill::createCuboidStock(80.0, 80.0, 15.0);
    const double v0 = volumeOf(stock->shape());

    auto out = skill::bolt_circle_pattern::apply(*stock, ring(4));
    ASSERT_TRUE(out.workpiece);
    EXPECT_TRUE(BRepCheck_Analyzer(out.workpiece->shape()).IsValid());
    EXPECT_LT(volumeOf(out.workpiece->shape()), v0) << "ring must remove material";
}

// 2. EDIT: changing hole_count regenerates a different ring (more holes →
// more material removed).  This is the parametric edit the grammar enables.
TEST(BoltCirclePattern, EditHoleCountRegenerates)
{
    auto stock = skill::createCuboidStock(80.0, 80.0, 15.0);
    const double v4 = volumeOf(skill::bolt_circle_pattern::apply(*stock, ring(4)).workpiece->shape());
    const double v6 = volumeOf(skill::bolt_circle_pattern::apply(*stock, ring(6)).workpiece->shape());
    EXPECT_LT(v6, v4) << "6 holes must remove more than 4 of the same diameter";
}

// 3. validate() rejects a degenerate ring (< 3 holes).
TEST(BoltCirclePattern, ValidateRejectsTooFewHoles)
{
    auto stock = skill::createCuboidStock(80.0, 80.0, 15.0);
    EXPECT_FALSE(skill::bolt_circle_pattern::validate(*stock, ring(2)).passed);
    EXPECT_TRUE(skill::bolt_circle_pattern::validate(*stock, ring(4)).passed);
}

// 4. RECOGNISE: the generated ring round-trips through STEP and the grammar
// re-recognises it as a bolt_circle_pattern with the right hole count.
TEST(BoltCirclePattern, RoundTripIsRecognised)
{
    namespace fs = std::filesystem;
    auto stock = skill::createCuboidStock(80.0, 80.0, 15.0);
    auto out = skill::bolt_circle_pattern::apply(*stock, ring(4));

    const fs::path p = fs::temp_directory_path() /
                       ("koo_bcp_" + std::to_string(::rand()) + ".step");
    std::string err;
    io::StepIO::write(out.workpiece->shape(), p, err);
    auto reim = io::StepIO::read(p, err);
    std::error_code ec; fs::remove(p, ec);
    ASSERT_TRUE(reim);
    skill::Workpiece foreign(*reim);

    const auto cands = re::analyze(foreign, /*applyCap=*/true);
    const skill::RecognizedFeature* bc = nullptr;
    for (const auto& c : cands)
        if (c.skill_id == "bolt_circle_pattern") { bc = &c; break; }
    ASSERT_NE(bc, nullptr) << "generated ring was not re-recognised";
    EXPECT_EQ(bc->recovered_params.value("hole_count", 0), 4);
}

// 6. SUBSUME: inferProcessPlan replaces the constituent drills with the single
// bolt_circle_pattern step, so the recovered plan is a clean editable unit.
TEST(BoltCirclePattern, SubsumesConstituentDrillsInPlan)
{
    namespace fs = std::filesystem;
    auto stock = skill::createCuboidStock(80.0, 80.0, 15.0);
    auto out = skill::bolt_circle_pattern::apply(*stock, ring(4));

    const fs::path p = fs::temp_directory_path() /
                       ("koo_bcps_" + std::to_string(::rand()) + ".step");
    std::string err;
    io::StepIO::write(out.workpiece->shape(), p, err);
    auto reim = io::StepIO::read(p, err);
    std::error_code ec; fs::remove(p, ec);
    ASSERT_TRUE(reim);
    skill::Workpiece foreign(*reim);

    const process::ProcessPlan plan = re::inferProcessPlan(foreign, 0.7);
    bool hasBolt = false;
    int  drills  = 0;
    for (const auto& s : plan.steps()) {
        if (s.skill_id == "bolt_circle_pattern") hasBolt = true;
        if (s.skill_id == "drill_hole" || s.skill_id == "drill_through_hole") ++drills;
    }
    EXPECT_TRUE(hasBolt) << "recovered plan must carry the bolt circle";
    EXPECT_EQ(drills, 0) << "constituent drills must be subsumed by the compound";
}

// 7. CAPSTONE — the whole loop: recognise a foreign bolt circle, edit the
// recovered compound step (4 → 6 holes), re-execute, and confirm the
// regenerated part is a 6-hole bolt circle.  This is generalised design
// automation demonstrated end to end.
TEST(BoltCirclePattern, EndToEndRecogniseEditRegenerate)
{
    namespace fs = std::filesystem;

    // (a) A foreign part containing a 4-hole bolt circle.
    auto stock = skill::createCuboidStock(80.0, 80.0, 15.0);
    auto built = skill::bolt_circle_pattern::apply(*stock, ring(4));
    auto roundTrip = [](const TopoDS_Shape& s) {
        const fs::path p = fs::temp_directory_path() /
                           ("koo_bce2e_" + std::to_string(::rand()) + ".step");
        std::string err;
        io::StepIO::write(s, p, err);
        auto reim = io::StepIO::read(p, err);
        std::error_code ec; fs::remove(p, ec);
        return reim;
    };
    auto reim = roundTrip(built.workpiece->shape());
    ASSERT_TRUE(reim);
    skill::Workpiece foreign(*reim);

    // (b) Recognise → a clean recovered plan (compound, drills subsumed).
    process::ProcessPlan plan = re::inferProcessPlan(foreign, 0.7);
    int idx = -1;
    for (int i = 0; i < static_cast<int>(plan.steps().size()); ++i)
        if (plan.steps()[i].skill_id == "bolt_circle_pattern") { idx = i; break; }
    ASSERT_GE(idx, 0) << "recovered plan has no bolt_circle_pattern step";
    ASSERT_EQ(plan.steps()[idx].params.value("hole_count", 0), 4);

    // (c) EDIT the recovered compound in place: 4 → 6 holes.
    nlohmann::json edited = plan.steps()[idx].params;
    edited["hole_count"] = 6;
    ASSERT_TRUE(plan.replaceParams(idx, edited));

    // (d) Re-execute the edited plan on fresh stock.
    auto fresh = skill::createCuboidStock(80.0, 80.0, 15.0);
    const auto result = process::Executor::execute(plan, fresh);
    ASSERT_TRUE(result.ok())
        << "edited plan failed; errors="
        << (result.errors.empty() ? "" : result.errors.front());
    ASSERT_NE(result.workpiece, nullptr);

    // (e) The regenerated part is now a 6-hole bolt circle.
    auto reim2 = roundTrip(result.workpiece->shape());
    ASSERT_TRUE(reim2);
    skill::Workpiece regen(*reim2);
    const auto cands = re::analyze(regen, /*applyCap=*/true);
    const skill::RecognizedFeature* bc = nullptr;
    for (const auto& c : cands)
        if (c.skill_id == "bolt_circle_pattern") { bc = &c; break; }
    ASSERT_NE(bc, nullptr) << "regenerated part not recognised as a bolt circle";
    EXPECT_EQ(bc->recovered_params.value("hole_count", 0), 6)
        << "the edit (4→6) did not regenerate a 6-hole bolt circle";
}

// 5. DISPATCH: a ProcessPlan with a bolt_circle_pattern step executes via the
// Executor (the step is now replayable, not dropped as un-dispatchable).
TEST(BoltCirclePattern, DispatchableViaExecutor)
{
    auto stock = skill::createCuboidStock(80.0, 80.0, 15.0);
    process::ProcessPlan plan;
    process::StepInvocation step;
    step.skill_id = "bolt_circle_pattern";
    step.params = {
        { "hole_count", 4 }, { "bolt_circle_dia_mm", 40.0 }, { "hole_dia_mm", 5.0 },
        { "center_x_mm", 40.0 }, { "center_y_mm", 40.0 },
        { "axis_dir", { 0.0, 0.0, -1.0 } }, { "through_hole", true },
    };
    plan.append(step);

    const auto result = process::Executor::execute(plan, stock);
    EXPECT_TRUE(result.ok())
        << "bolt_circle_pattern must dispatch; errors="
        << (result.errors.empty() ? "" : result.errors.front());
    ASSERT_NE(result.workpiece, nullptr);
    EXPECT_TRUE(BRepCheck_Analyzer(result.workpiece->shape()).IsValid());
}
