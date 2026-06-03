// @lat: [[process/test-strategy#skill round-trip]]
//
// multi_radius_fillet_table — apply + DFM + recognize sanity checks.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/multi_radius_fillet_table.hpp"

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

// ─── 1. Apply: two bands, two different radii ────────────────────────────
TEST(SkillMultiRadiusFilletTable, ApplyTwoBandsSucceeds)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 20.0);
    const double volBefore = volumeOf(stock->shape());

    skill::multi_radius_fillet_table::Input in;
    // Band 1 — top rim, radius 1 mm
    in.edge_specs.push_back({ 19.9, 20.1, 1.0 });
    // Band 2 — bottom rim, radius 2 mm
    in.edge_specs.push_back({ -0.1,  0.1, 2.0 });

    auto out = skill::multi_radius_fillet_table::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    const int filleted =
        out.signature.pattern["edge_count_filleted"].get<int>();
    EXPECT_GT(filleted, 0);

    // Material removed.
    const double volAfter = volumeOf(out.workpiece->shape());
    EXPECT_LT(volAfter, volBefore);

    EXPECT_EQ(out.signature.skill_id,
              std::string("multi_radius_fillet_table"));
    EXPECT_EQ(out.signature.pattern["spec_count"].get<size_t>(), 2u);
}

// ─── 2. Validate rejects empty spec list ─────────────────────────────────
TEST(SkillMultiRadiusFilletTable, ValidateRejectsEmptySpecs)
{
    auto stock = skill::createCuboidStock(20.0, 20.0, 10.0);

    skill::multi_radius_fillet_table::Input in;   // edge_specs empty

    auto r = skill::multi_radius_fillet_table::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_THROW(skill::multi_radius_fillet_table::apply(*stock, in),
                 skill::SkillError);
}

// ─── 3. Validate rejects per-spec bad radius or band ──────────────────────
TEST(SkillMultiRadiusFilletTable, ValidateRejectsBadSpec)
{
    auto stock = skill::createCuboidStock(20.0, 20.0, 10.0);

    skill::multi_radius_fillet_table::Input in;
    in.edge_specs.push_back({ 9.9, 10.1, -1.0 });    // bad radius
    in.edge_specs.push_back({ 5.0,  4.0,  1.0 });    // bad band (min > max)

    auto r = skill::multi_radius_fillet_table::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_THROW(skill::multi_radius_fillet_table::apply(*stock, in),
                 skill::SkillError);
}

// ─── 4. No edges match any spec → throws ─────────────────────────────────
TEST(SkillMultiRadiusFilletTable, NoMatchingEdgesThrows)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::multi_radius_fillet_table::Input in;
    in.edge_specs.push_back({ 100.0, 110.0, 1.0 });

    EXPECT_THROW(skill::multi_radius_fillet_table::apply(*stock, in),
                 skill::SkillError);
}
