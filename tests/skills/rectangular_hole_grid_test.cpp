// @lat: [[engine/skills#rectangular_hole_grid]]
//
// Generative side of the B1.2 grammar grid pattern (e.g. a speaker grille):
// recognise -> edit -> regenerate -> dispatch.  Mirrors the bolt_circle /
// linear tests; relies on the grammar enrichment that records origin + basis.

#include <gtest/gtest.h>

#include "skills/rectangular_hole_grid.hpp"
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

skill::rectangular_hole_grid::Input grid(int cols, int rows)
{
    skill::rectangular_hole_grid::Input in;
    in.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.cols         = cols;
    in.rows         = rows;
    in.hole_dia_mm  = 4.0;
    in.origin_x_mm  = 20.0;
    in.origin_y_mm  = 20.0;
    in.u_dx = 1.0; in.u_dy = 0.0;
    in.v_dx = 0.0; in.v_dy = 1.0;
    in.pitch_u_mm   = 10.0;
    in.pitch_v_mm   = 8.0;
    in.axis_dir     = gp_Dir(0, 0, -1);
    in.through_hole = true;
    return in;
}

}  // namespace

TEST(RectangularHoleGrid, GeneratesValidGrid)
{
    auto stock = skill::createCuboidStock(80.0, 80.0, 15.0);
    const double v0 = volumeOf(stock->shape());
    auto out = skill::rectangular_hole_grid::apply(*stock, grid(2, 3));
    ASSERT_TRUE(out.workpiece);
    EXPECT_TRUE(BRepCheck_Analyzer(out.workpiece->shape()).IsValid());
    EXPECT_LT(volumeOf(out.workpiece->shape()), v0);
}

TEST(RectangularHoleGrid, EditDimsRegenerates)
{
    auto stock = skill::createCuboidStock(80.0, 80.0, 15.0);
    const double v6 = volumeOf(skill::rectangular_hole_grid::apply(*stock, grid(2, 3)).workpiece->shape());
    const double v9 = volumeOf(skill::rectangular_hole_grid::apply(*stock, grid(3, 3)).workpiece->shape());
    EXPECT_LT(v9, v6) << "3x3 must remove more than 2x3";
}

TEST(RectangularHoleGrid, ValidateRejectsTooSmall)
{
    auto stock = skill::createCuboidStock(80.0, 80.0, 15.0);
    EXPECT_FALSE(skill::rectangular_hole_grid::validate(*stock, grid(2, 2)).passed);  // 4 < 6
    EXPECT_TRUE(skill::rectangular_hole_grid::validate(*stock, grid(2, 3)).passed);
}

TEST(RectangularHoleGrid, RoundTripIsRecognised)
{
    namespace fs = std::filesystem;
    auto stock = skill::createCuboidStock(80.0, 80.0, 15.0);
    auto out = skill::rectangular_hole_grid::apply(*stock, grid(2, 3));

    const fs::path p = fs::temp_directory_path() /
                       ("koo_grid_" + std::to_string(::rand()) + ".step");
    std::string err;
    io::StepIO::write(out.workpiece->shape(), p, err);
    auto reim = io::StepIO::read(p, err);
    std::error_code ec; fs::remove(p, ec);
    ASSERT_TRUE(reim);
    skill::Workpiece foreign(*reim);

    const auto cands = re::analyze(foreign, /*applyCap=*/true);
    const skill::RecognizedFeature* g = nullptr;
    for (const auto& c : cands)
        if (c.skill_id == "rectangular_hole_grid") { g = &c; break; }
    ASSERT_NE(g, nullptr) << "generated grid was not re-recognised";
    EXPECT_EQ(g->recovered_params.value("hole_count", 0), 6);
    EXPECT_EQ(g->recovered_params.value("cols", 0) * g->recovered_params.value("rows", 0), 6);
}

TEST(RectangularHoleGrid, DispatchableViaExecutor)
{
    auto stock = skill::createCuboidStock(80.0, 80.0, 15.0);
    process::ProcessPlan plan;
    process::StepInvocation step;
    step.skill_id = "rectangular_hole_grid";
    step.params = {
        { "cols", 2 }, { "rows", 3 }, { "hole_dia_mm", 4.0 },
        { "origin_x_mm", 20.0 }, { "origin_y_mm", 20.0 },
        { "u_dir", { 1.0, 0.0, 0.0 } }, { "v_dir", { 0.0, 1.0, 0.0 } },
        { "pitch_u_mm", 10.0 }, { "pitch_v_mm", 8.0 },
        { "axis_dir", { 0.0, 0.0, -1.0 } }, { "through_hole", true },
    };
    plan.append(step);

    const auto result = process::Executor::execute(plan, stock);
    EXPECT_TRUE(result.ok())
        << "rectangular_hole_grid must dispatch; errors="
        << (result.errors.empty() ? "" : result.errors.front());
    ASSERT_NE(result.workpiece, nullptr);
    EXPECT_TRUE(BRepCheck_Analyzer(result.workpiece->shape()).IsValid());
}
