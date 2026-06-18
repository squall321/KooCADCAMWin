// @lat: [[engine/skills#coaxial_step_bore]]
//
// Generative side of the B1.2 grammar step-bore pattern: recognise -> edit ->
// regenerate -> dispatch.  Mirrors the other generative-compound tests; relies
// on the grammar enrichment that records per-step depth + through.

#include <gtest/gtest.h>

#include "skills/coaxial_step_bore.hpp"
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

// A 3-step bore at the centre; `seat` sets the widest diameter.
skill::coaxial_step_bore::Input stepBore(double seat = 16.0)
{
    skill::coaxial_step_bore::Input in;
    in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.center_x_mm = 40.0;
    in.center_y_mm = 40.0;
    in.axis_dir    = gp_Dir(0, 0, -1);
    in.steps = {
        { seat, 4.0,  false },   // wide seat, shallow
        { 10.0, 10.0, false },   // mid bore
        { 5.0,  0.0,  true  },   // through drill
    };
    return in;
}

}  // namespace

TEST(CoaxialStepBore, GeneratesValidStack)
{
    auto stock = skill::createCuboidStock(80.0, 80.0, 15.0);
    const double v0 = volumeOf(stock->shape());
    auto out = skill::coaxial_step_bore::apply(*stock, stepBore());
    ASSERT_TRUE(out.workpiece);
    EXPECT_TRUE(BRepCheck_Analyzer(out.workpiece->shape()).IsValid());
    EXPECT_LT(volumeOf(out.workpiece->shape()), v0);
}

TEST(CoaxialStepBore, EditSeatRegenerates)
{
    auto stock = skill::createCuboidStock(80.0, 80.0, 15.0);
    const double v16 = volumeOf(skill::coaxial_step_bore::apply(*stock, stepBore(16.0)).workpiece->shape());
    const double v22 = volumeOf(skill::coaxial_step_bore::apply(*stock, stepBore(22.0)).workpiece->shape());
    EXPECT_LT(v22, v16) << "a wider seat must remove more material";
}

TEST(CoaxialStepBore, ValidateRejectsTooFewSteps)
{
    auto stock = skill::createCuboidStock(80.0, 80.0, 15.0);
    skill::coaxial_step_bore::Input two = stepBore();
    two.steps.pop_back();   // 2 steps
    EXPECT_FALSE(skill::coaxial_step_bore::validate(*stock, two).passed);
    EXPECT_TRUE(skill::coaxial_step_bore::validate(*stock, stepBore()).passed);
}

TEST(CoaxialStepBore, RoundTripIsRecognised)
{
    namespace fs = std::filesystem;
    auto stock = skill::createCuboidStock(80.0, 80.0, 15.0);
    auto out = skill::coaxial_step_bore::apply(*stock, stepBore());

    const fs::path p = fs::temp_directory_path() /
                       ("koo_csb_" + std::to_string(::rand()) + ".step");
    std::string err;
    io::StepIO::write(out.workpiece->shape(), p, err);
    auto reim = io::StepIO::read(p, err);
    std::error_code ec; fs::remove(p, ec);
    ASSERT_TRUE(reim);
    skill::Workpiece foreign(*reim);

    const auto cands = re::analyze(foreign, /*applyCap=*/true);
    const skill::RecognizedFeature* sb = nullptr;
    for (const auto& c : cands)
        if (c.skill_id == "coaxial_step_bore") { sb = &c; break; }
    ASSERT_NE(sb, nullptr) << "generated step bore was not re-recognised";
    EXPECT_EQ(sb->recovered_params.value("step_count", 0), 3);
    EXPECT_NEAR(sb->recovered_params.value("max_dia_mm", 0.0), 16.0, 0.5);
}

// inferProcessPlan replaces the coaxial drills/bores with the single step-bore
// step, so the recovered plan is a clean editable unit.
TEST(CoaxialStepBore, SubsumesConstituentBoresInPlan)
{
    namespace fs = std::filesystem;
    auto stock = skill::createCuboidStock(80.0, 80.0, 15.0);
    auto out = skill::coaxial_step_bore::apply(*stock, stepBore());

    const fs::path p = fs::temp_directory_path() /
                       ("koo_csbs_" + std::to_string(::rand()) + ".step");
    std::string err;
    io::StepIO::write(out.workpiece->shape(), p, err);
    auto reim = io::StepIO::read(p, err);
    std::error_code ec; fs::remove(p, ec);
    ASSERT_TRUE(reim);
    skill::Workpiece foreign(*reim);

    const process::ProcessPlan plan = re::inferProcessPlan(foreign, 0.7);
    bool hasStep = false;
    int  atoms   = 0;
    for (const auto& s : plan.steps()) {
        if (s.skill_id == "coaxial_step_bore") hasStep = true;
        if (s.skill_id == "drill_hole" || s.skill_id == "drill_through_hole" ||
            s.skill_id == "bore_cylindrical") ++atoms;
    }
    EXPECT_TRUE(hasStep) << "recovered plan must carry the step bore";
    EXPECT_EQ(atoms, 0) << "coaxial drills/bores must be subsumed by the step bore";
}

TEST(CoaxialStepBore, DispatchableViaExecutor)
{
    auto stock = skill::createCuboidStock(80.0, 80.0, 15.0);
    process::ProcessPlan plan;
    process::StepInvocation step;
    step.skill_id = "coaxial_step_bore";
    step.params = {
        { "position_x_mm", 40.0 }, { "position_y_mm", 40.0 },
        { "axis_dir", { 0.0, 0.0, -1.0 } },
        { "steps", json::array({
            json{ { "diameter_mm", 16.0 }, { "depth_mm", 4.0 },  { "through", false } },
            json{ { "diameter_mm", 10.0 }, { "depth_mm", 10.0 }, { "through", false } },
            json{ { "diameter_mm", 5.0 },  { "depth_mm", 0.0 },  { "through", true  } },
        }) },
    };
    plan.append(step);

    const auto result = process::Executor::execute(plan, stock);
    EXPECT_TRUE(result.ok())
        << "coaxial_step_bore must dispatch; errors="
        << (result.errors.empty() ? "" : result.errors.front());
    ASSERT_NE(result.workpiece, nullptr);
    EXPECT_TRUE(BRepCheck_Analyzer(result.workpiece->shape()).IsValid());
}
