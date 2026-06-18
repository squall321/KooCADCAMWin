// @lat: [[engine/skills#linear_hole_array]]
//
// Generative side of the B1.2 grammar linear pattern: recognise -> edit ->
// regenerate -> dispatch, mirroring bolt_circle_pattern_test.

#include <gtest/gtest.h>

#include "skills/linear_hole_array.hpp"
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

skill::linear_hole_array::Input row(int n)
{
    skill::linear_hole_array::Input in;
    in.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.hole_count   = n;
    in.hole_dia_mm  = 5.0;
    in.start_x_mm   = 15.0;
    in.start_y_mm   = 40.0;
    in.dir_x        = 1.0;
    in.dir_y        = 0.0;
    in.pitch_mm     = 12.0;
    in.axis_dir     = gp_Dir(0, 0, -1);
    in.through_hole = true;
    return in;
}

}  // namespace

TEST(LinearHoleArray, GeneratesValidRow)
{
    auto stock = skill::createCuboidStock(80.0, 80.0, 15.0);
    const double v0 = volumeOf(stock->shape());
    auto out = skill::linear_hole_array::apply(*stock, row(5));
    ASSERT_TRUE(out.workpiece);
    EXPECT_TRUE(BRepCheck_Analyzer(out.workpiece->shape()).IsValid());
    EXPECT_LT(volumeOf(out.workpiece->shape()), v0);
}

TEST(LinearHoleArray, EditHoleCountRegenerates)
{
    auto stock = skill::createCuboidStock(80.0, 80.0, 15.0);
    const double v4 = volumeOf(skill::linear_hole_array::apply(*stock, row(4)).workpiece->shape());
    const double v6 = volumeOf(skill::linear_hole_array::apply(*stock, row(6)).workpiece->shape());
    EXPECT_LT(v6, v4) << "6 holes must remove more than 4";
}

TEST(LinearHoleArray, ValidateRejectsTooFewHoles)
{
    auto stock = skill::createCuboidStock(80.0, 80.0, 15.0);
    EXPECT_FALSE(skill::linear_hole_array::validate(*stock, row(2)).passed);
    EXPECT_TRUE(skill::linear_hole_array::validate(*stock, row(5)).passed);
}

// A BLIND row must round-trip as blind, not silently become through.
TEST(LinearHoleArray, BlindRowRoundTripsAsBlind)
{
    namespace fs = std::filesystem;
    auto stock = skill::createCuboidStock(80.0, 80.0, 15.0);
    auto in = row(5);
    in.through_hole = false;
    in.depth_mm     = 8.0;
    auto out = skill::linear_hole_array::apply(*stock, in);

    const fs::path p = fs::temp_directory_path() /
                       ("koo_lhab_" + std::to_string(::rand()) + ".step");
    std::string err;
    io::StepIO::write(out.workpiece->shape(), p, err);
    auto reim = io::StepIO::read(p, err);
    std::error_code ec; fs::remove(p, ec);
    ASSERT_TRUE(reim);
    skill::Workpiece foreign(*reim);

    const auto cands = re::analyze(foreign, /*applyCap=*/true);
    const skill::RecognizedFeature* la = nullptr;
    for (const auto& c : cands)
        if (c.skill_id == "linear_hole_array") { la = &c; break; }
    ASSERT_NE(la, nullptr);
    EXPECT_FALSE(la->recovered_params.value("through_hole", true))
        << "a blind row must NOT regenerate as a through row";
    EXPECT_NEAR(la->recovered_params.value("depth_mm", 0.0), 8.0, 0.5);
}

TEST(LinearHoleArray, RoundTripIsRecognised)
{
    namespace fs = std::filesystem;
    auto stock = skill::createCuboidStock(80.0, 80.0, 15.0);
    auto out = skill::linear_hole_array::apply(*stock, row(5));

    const fs::path p = fs::temp_directory_path() /
                       ("koo_lha_" + std::to_string(::rand()) + ".step");
    std::string err;
    io::StepIO::write(out.workpiece->shape(), p, err);
    auto reim = io::StepIO::read(p, err);
    std::error_code ec; fs::remove(p, ec);
    ASSERT_TRUE(reim);
    skill::Workpiece foreign(*reim);

    const auto cands = re::analyze(foreign, /*applyCap=*/true);
    const skill::RecognizedFeature* la = nullptr;
    for (const auto& c : cands)
        if (c.skill_id == "linear_hole_array") { la = &c; break; }
    ASSERT_NE(la, nullptr) << "generated row was not re-recognised";
    EXPECT_EQ(la->recovered_params.value("hole_count", 0), 5);
}

TEST(LinearHoleArray, DispatchableViaExecutor)
{
    auto stock = skill::createCuboidStock(80.0, 80.0, 15.0);
    process::ProcessPlan plan;
    process::StepInvocation step;
    step.skill_id = "linear_hole_array";
    step.params = {
        { "hole_count", 5 }, { "hole_dia_mm", 5.0 },
        { "start_x_mm", 15.0 }, { "start_y_mm", 40.0 },
        { "direction", { 1.0, 0.0, 0.0 } }, { "pitch_mm", 12.0 },
        { "axis_dir", { 0.0, 0.0, -1.0 } }, { "through_hole", true },
    };
    plan.append(step);

    const auto result = process::Executor::execute(plan, stock);
    EXPECT_TRUE(result.ok())
        << "linear_hole_array must dispatch; errors="
        << (result.errors.empty() ? "" : result.errors.front());
    ASSERT_NE(result.workpiece, nullptr);
    EXPECT_TRUE(BRepCheck_Analyzer(result.workpiece->shape()).IsValid());
}
