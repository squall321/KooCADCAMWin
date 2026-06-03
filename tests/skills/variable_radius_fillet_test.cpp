// @lat: [[process/test-strategy#skill round-trip]]
//
// variable_radius_fillet — basic apply + DFM + recognize sanity checks.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/variable_radius_fillet.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

using namespace koocadcam;

namespace {

double volumeOf(const TopoDS_Shape& s)
{
    GProp_GProps p;
    BRepGProp::VolumeProperties(s, p);
    return p.Mass();
}

}  // namespace

// ─── 1. Apply: variable-radius fillet on the top rim of a cuboid ──────────
TEST(SkillVariableRadiusFillet, ApplyTopRimSucceeds)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);
    const double volBefore = volumeOf(stock->shape());

    skill::variable_radius_fillet::Input in;
    in.edge_z_band_min = 9.9;
    in.edge_z_band_max = 10.1;
    in.radius_start_mm = 0.5;
    in.radius_end_mm   = 2.0;

    auto out = skill::variable_radius_fillet::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    // edge_count_filleted > 0 — material was actually removed.
    const int n = out.signature.pattern["edge_count_filleted"].get<int>();
    EXPECT_GT(n, 0);

    // Volume change negative (material removed by the fillet).
    const double volAfter = volumeOf(out.workpiece->shape());
    EXPECT_LT(volAfter, volBefore);

    EXPECT_EQ(out.signature.skill_id, std::string("variable_radius_fillet"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
}

// ─── 2. Validate rejects radius <= 0 ─────────────────────────────────────
TEST(SkillVariableRadiusFillet, ValidateRejectsBadRadii)
{
    auto stock = skill::createCuboidStock(20.0, 20.0, 5.0);

    skill::variable_radius_fillet::Input in;
    in.edge_z_band_min = 4.9;
    in.edge_z_band_max = 5.1;
    in.radius_start_mm = -1.0;
    in.radius_end_mm   = 0.5;

    auto r = skill::variable_radius_fillet::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_THROW(skill::variable_radius_fillet::apply(*stock, in),
                 skill::SkillError);
}

// ─── 3. No matching edges → throws ────────────────────────────────────────
TEST(SkillVariableRadiusFillet, NoMatchingEdgesThrows)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::variable_radius_fillet::Input in;
    in.edge_z_band_min = 100.0;
    in.edge_z_band_max = 110.0;
    in.radius_start_mm = 0.5;
    in.radius_end_mm   = 1.0;

    EXPECT_THROW(skill::variable_radius_fillet::apply(*stock, in),
                 skill::SkillError);
}

// ─── 4. Recognize: history replay returns full-confidence match ───────────
TEST(SkillVariableRadiusFillet, RecognizeHistoryReplay)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::variable_radius_fillet::Input in;
    in.edge_z_band_min = 9.9;
    in.edge_z_band_max = 10.1;
    in.radius_start_mm = 0.5;
    in.radius_end_mm   = 1.5;

    auto out  = skill::variable_radius_fillet::apply(*stock, in);
    auto cands = skill::variable_radius_fillet::recognize(*out.workpiece);

    ASSERT_FALSE(cands.empty());
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_NEAR(cands[0].recovered_params["radius_start_mm"].get<double>(),
                in.radius_start_mm, 1e-6);
    EXPECT_NEAR(cands[0].recovered_params["radius_end_mm"].get<double>(),
                in.radius_end_mm, 1e-6);
}
