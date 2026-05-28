// @lat: [[process/test-strategy#skill round-trip]]
//
// hem skill — 180° fold-back edge.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/hem.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

#include <cmath>

using namespace koocadcam;

namespace {
double volumeOf(const TopoDS_Shape& s) {
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}
}  // namespace

// ── 1. Flat hem on x_max edge adds expected slab volume ────────────────
TEST(SkillHem, ApplyFlatHemAddsSlab)
{
    auto stock = skill::createCuboidStock(100.0, 80.0, 1.5);

    skill::hem::Input in;
    in.edge_selector = skill::flange::EdgeSide::XMax;
    in.hem_width_mm  = 4.0;        // ≥ 2 × 1.5 = 3.0
    in.flat_hem      = true;

    auto out = skill::hem::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    // Added volume ≈ hem_width × edge_length × thickness = 4 × 80 × 1.5
    const double expected = 4.0 * 80.0 * 1.5;
    const double added = volumeOf(out.workpiece->shape()) - volumeOf(stock->shape());
    EXPECT_NEAR(added, expected, expected * 0.05);

    EXPECT_EQ(out.signature.skill_id, std::string("hem"));
    EXPECT_TRUE(out.signature.pattern["flat_hem"].get<bool>());
}

// ── 2. Open hem leaves a gap (separation ≈ 2 × thickness) ─────────────
TEST(SkillHem, OpenHemSeparation)
{
    auto stock = skill::createCuboidStock(100.0, 80.0, 1.5);

    skill::hem::Input in;
    in.edge_selector = skill::flange::EdgeSide::YMax;
    in.hem_width_mm  = 5.0;
    in.flat_hem      = false;

    auto out = skill::hem::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    // Open hem: separation = thickness + thickness = 2 × thickness.
    EXPECT_NEAR(out.signature.pattern["separation_mm"].get<double>(), 3.0, 1e-6);
}

// ── 3. DFM-H-MIN-W: hem_width too small ────────────────────────────────
TEST(SkillHem, ValidateRejectsTooSmallWidth)
{
    auto stock = skill::createCuboidStock(100.0, 80.0, 1.5);

    skill::hem::Input in;
    in.edge_selector = skill::flange::EdgeSide::XMax;
    in.hem_width_mm  = 2.0;     // < 2 × 1.5 = 3.0
    in.flat_hem      = true;

    auto r = skill::hem::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-H-MIN-W") { found = true; break; }
    EXPECT_TRUE(found);
    EXPECT_THROW(skill::hem::apply(*stock, in), skill::SkillError);
}

// ── 4. DFM-H-SHEET: too-thick stock rejected ──────────────────────────
TEST(SkillHem, ValidateRejectsThickStock)
{
    auto thick = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::hem::Input in;
    in.edge_selector = skill::flange::EdgeSide::XMax;
    in.hem_width_mm  = 25.0;
    in.flat_hem      = true;

    auto r = skill::hem::validate(*thick, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-H-SHEET") { found = true; break; }
    EXPECT_TRUE(found);
}

// ── 5. Recognize finds the hem layer ───────────────────────────────────
TEST(SkillHem, RecognizeFindsHem)
{
    auto stock = skill::createCuboidStock(100.0, 80.0, 1.5);

    skill::hem::Input in;
    in.edge_selector = skill::flange::EdgeSide::XMax;
    in.hem_width_mm  = 4.0;
    in.flat_hem      = true;

    auto out = skill::hem::apply(*stock, in);
    auto cands = skill::hem::recognize(*out.workpiece);

    ASSERT_FALSE(cands.empty());
    const auto& c = cands.front();
    EXPECT_EQ(c.skill_id, std::string("hem"));
    // The recovered hem width should match input within tolerance.
    EXPECT_NEAR(c.recovered_params["hem_width_mm"].get<double>(), 4.0, 0.2);
    EXPECT_TRUE(c.recovered_params["flat_hem"].get<bool>());
}
