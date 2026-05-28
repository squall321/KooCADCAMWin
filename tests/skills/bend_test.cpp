// @lat: [[process/test-strategy#skill round-trip]]
//
// bend skill — fold sheet along a line.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/bend.hpp"

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

// ── 1. Apply: 90° upward bend on a 100×80×1.5 sheet ────────────────────
TEST(SkillBend, ApplyBuildsValidSolid)
{
    auto stock = skill::createCuboidStock(100.0, 80.0, 1.5);

    skill::bend::Input in;
    in.bend_line_start_xy = { 50.0,  0.0 };
    in.bend_line_end_xy   = { 50.0, 80.0 };  // bend along y, at x=50
    in.angle_deg          = 90.0;
    in.bend_radius_mm     = 1.0;            // ≥ 0.5 × 1.5 = 0.75

    auto out = skill::bend::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    // Bending should not destroy material (some addition from bend wedge OK).
    EXPECT_GT(volumeOf(out.workpiece->shape()), 0.0);

    EXPECT_EQ(out.signature.skill_id, std::string("bend"));
    EXPECT_NEAR(out.signature.pattern["bend_radius_mm"].get<double>(), 1.0, 1e-6);
    EXPECT_NEAR(out.signature.pattern["angle_deg"].get<double>(), 90.0, 1e-6);
}

// ── 2. DFM-B-MIN-R: bend radius too tight ──────────────────────────────
TEST(SkillBend, ValidateRejectsTooTightRadius)
{
    auto stock = skill::createCuboidStock(100.0, 80.0, 1.5);

    skill::bend::Input in;
    in.bend_line_start_xy = { 50.0,  0.0 };
    in.bend_line_end_xy   = { 50.0, 80.0 };
    in.angle_deg          = 45.0;
    in.bend_radius_mm     = 0.3;            // 0.3 < 0.5 × 1.5 = 0.75 → fail

    auto r = skill::bend::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-B-MIN-R") { found = true; break; }
    EXPECT_TRUE(found);
    EXPECT_THROW(skill::bend::apply(*stock, in), skill::SkillError);
}

// ── 3. DFM-B-ANGLE: out-of-range angle rejected ────────────────────────
TEST(SkillBend, ValidateRejectsBadAngle)
{
    auto stock = skill::createCuboidStock(100.0, 80.0, 1.5);

    skill::bend::Input in;
    in.bend_line_start_xy = { 50.0,  0.0 };
    in.bend_line_end_xy   = { 50.0, 80.0 };
    in.angle_deg          = 200.0;          // > 180 → fail
    in.bend_radius_mm     = 1.0;

    auto r = skill::bend::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-B-ANGLE") { found = true; break; }
    EXPECT_TRUE(found);
}

// ── 4. DFM-INPUT: collapsed bend_line rejected ─────────────────────────
TEST(SkillBend, ValidateRejectsDegenerateBendLine)
{
    auto stock = skill::createCuboidStock(100.0, 80.0, 1.5);

    skill::bend::Input in;
    in.bend_line_start_xy = { 50.0, 40.0 };
    in.bend_line_end_xy   = { 50.0, 40.0 };  // same point
    in.angle_deg          = 90.0;
    in.bend_radius_mm     = 1.0;

    auto r = skill::bend::validate(*stock, in);
    EXPECT_FALSE(r.passed);
}

// ── 5. Recognize: after bend, find the cylindrical bend face ───────────
TEST(SkillBend, RecognizeFindsCylindricalBendFace)
{
    auto stock = skill::createCuboidStock(100.0, 80.0, 1.5);

    skill::bend::Input in;
    in.bend_line_start_xy = { 50.0,  0.0 };
    in.bend_line_end_xy   = { 50.0, 80.0 };
    in.angle_deg          = 90.0;
    in.bend_radius_mm     = 1.0;

    auto out = skill::bend::apply(*stock, in);
    auto cands = skill::bend::recognize(*out.workpiece);

    // At least one candidate expected — the cylindrical bend zone.
    ASSERT_GE(cands.size(), 1u);
    const auto& c = cands.front();
    EXPECT_EQ(c.skill_id, std::string("bend"));
    EXPECT_NEAR(c.recovered_params["bend_radius_mm"].get<double>(), 1.0, 0.05);
}
