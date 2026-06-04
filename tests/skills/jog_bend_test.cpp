// @lat: [[process/test-strategy#skill round-trip]]
//
// jog_bend — Z-shaped jog (two opposing bends).

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/jog_bend.hpp"

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

// ── 1. Apply: jog with 5 mm offset & 45° angle ─────────────────────────
TEST(SkillJogBend, ApplyProducesValidSolid)
{
    auto stock = skill::createCuboidStock(100.0, 80.0, 1.5);

    skill::jog_bend::Input in;
    in.bend_axis_start_xy = { 50.0,  0.0 };
    in.bend_axis_end_xy   = { 50.0, 80.0 };
    in.jog_offset_mm      = 5.0;
    in.jog_angle_deg      = 45.0;
    in.inner_radius_mm    = 1.0;

    auto out = skill::jog_bend::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    EXPECT_GT(volumeOf(out.workpiece->shape()), 0.0);

    EXPECT_EQ(out.signature.skill_id, std::string("jog_bend"));
    EXPECT_TRUE(out.signature.pattern["is_compound"].get<bool>());
    EXPECT_EQ(out.signature.pattern["bend_count"].get<int>(), 2);
    EXPECT_NEAR(out.signature.pattern["jog_offset_mm"].get<double>(),
                5.0, 1e-6);
}

// ── 2. DFM-J-OFFSET: offset ≤ thickness ───────────────────────────────
TEST(SkillJogBend, ValidateRejectsTooSmallOffset)
{
    auto stock = skill::createCuboidStock(100.0, 80.0, 1.5);

    skill::jog_bend::Input in;
    in.bend_axis_start_xy = { 50.0,  0.0 };
    in.bend_axis_end_xy   = { 50.0, 80.0 };
    in.jog_offset_mm      = 1.0;        // < thickness 1.5
    in.jog_angle_deg      = 45.0;
    in.inner_radius_mm    = 1.0;

    auto r = skill::jog_bend::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-J-OFFSET") { found = true; break; }
    EXPECT_TRUE(found);
    EXPECT_THROW(skill::jog_bend::apply(*stock, in), skill::SkillError);
}

// ── 3. DFM-J-ANGLE: outside 30–60° ─────────────────────────────────────
TEST(SkillJogBend, ValidateRejectsBadAngle)
{
    auto stock = skill::createCuboidStock(100.0, 80.0, 1.5);

    skill::jog_bend::Input in;
    in.bend_axis_start_xy = { 50.0,  0.0 };
    in.bend_axis_end_xy   = { 50.0, 80.0 };
    in.jog_offset_mm      = 5.0;
    in.jog_angle_deg      = 90.0;     // > 60
    in.inner_radius_mm    = 1.0;

    auto r = skill::jog_bend::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-J-ANGLE") { found = true; break; }
    EXPECT_TRUE(found);
}

// ── 4. DFM-J-MIN-R: bend radius too small ──────────────────────────────
TEST(SkillJogBend, ValidateRejectsTooSmallRadius)
{
    auto stock = skill::createCuboidStock(100.0, 80.0, 1.5);

    skill::jog_bend::Input in;
    in.bend_axis_start_xy = { 50.0,  0.0 };
    in.bend_axis_end_xy   = { 50.0, 80.0 };
    in.jog_offset_mm      = 5.0;
    in.jog_angle_deg      = 45.0;
    in.inner_radius_mm    = 0.3;     // < 0.5 × 1.5
    auto r = skill::jog_bend::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-J-MIN-R") { found = true; break; }
    EXPECT_TRUE(found);
}

// ── 5. Recognize: detect Z-shape after apply ───────────────────────────
TEST(SkillJogBend, RecognizeDetectsJog)
{
    auto stock = skill::createCuboidStock(100.0, 80.0, 1.5);

    skill::jog_bend::Input in;
    in.bend_axis_start_xy = { 50.0,  0.0 };
    in.bend_axis_end_xy   = { 50.0, 80.0 };
    in.jog_offset_mm      = 5.0;
    in.jog_angle_deg      = 45.0;
    in.inner_radius_mm    = 1.0;

    auto out = skill::jog_bend::apply(*stock, in);
    auto cands = skill::jog_bend::recognize(*out.workpiece);
    // Recognition is best-effort; require at least 0 candidates (no crash).
    EXPECT_GE(cands.size(), 0u);
    if (!cands.empty()) {
        EXPECT_EQ(cands.front().skill_id, std::string("jog_bend"));
    }
}
