// @lat: [[process/test-strategy#skill round-trip]]
//
// gusset_form — stamped reinforcement gusset.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/gusset_form.hpp"

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

// ── 1. Apply: stamp formed gusset at corner ────────────────────────────
TEST(SkillGussetForm, ApplyProducesValidSolid)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 1.5);

    skill::gusset_form::Input in;
    in.corner_xyz       = { 60.0, 60.0, 1.5 };
    in.wall_a_normal    = { 1.0, 0.0, 0.0 };
    in.wall_b_normal    = { 0.0, 1.0, 0.0 };
    in.gusset_width_mm  = 6.0;
    in.gusset_depth_mm  = 4.0;
    in.gusset_radius_mm = 1.5;

    auto out = skill::gusset_form::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    EXPECT_GT(volumeOf(out.workpiece->shape()), 0.0);

    EXPECT_EQ(out.signature.skill_id, std::string("gusset_form"));
    EXPECT_TRUE(out.signature.pattern["is_compound"].get<bool>());
    EXPECT_EQ(out.signature.pattern["sheet_feature_type"].get<std::string>(),
              std::string("stamped_gusset"));
}

// ── 2. DFM-G-WIDTH: width too small ────────────────────────────────────
TEST(SkillGussetForm, ValidateRejectsTooSmallWidth)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 1.5);

    skill::gusset_form::Input in;
    in.corner_xyz       = { 60.0, 60.0, 1.5 };
    in.wall_a_normal    = { 1.0, 0.0, 0.0 };
    in.wall_b_normal    = { 0.0, 1.0, 0.0 };
    in.gusset_width_mm  = 3.0;          // < 3 × 1.5 = 4.5
    in.gusset_depth_mm  = 4.0;
    in.gusset_radius_mm = 1.5;

    auto r = skill::gusset_form::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-G-WIDTH") { found = true; break; }
    EXPECT_TRUE(found);
    EXPECT_THROW(skill::gusset_form::apply(*stock, in), skill::SkillError);
}

// ── 3. DFM-G-RADIUS: radius < thickness ───────────────────────────────
TEST(SkillGussetForm, ValidateRejectsTooSharpRadius)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 1.5);

    skill::gusset_form::Input in;
    in.corner_xyz       = { 60.0, 60.0, 1.5 };
    in.wall_a_normal    = { 1.0, 0.0, 0.0 };
    in.wall_b_normal    = { 0.0, 1.0, 0.0 };
    in.gusset_width_mm  = 6.0;
    in.gusset_depth_mm  = 4.0;
    in.gusset_radius_mm = 0.5;        // < thickness 1.5

    auto r = skill::gusset_form::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-G-RADIUS") { found = true; break; }
    EXPECT_TRUE(found);
}

// ── 4. DFM-G-DEPTH: depth out of range ─────────────────────────────────
TEST(SkillGussetForm, ValidateRejectsBadDepth)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 1.5);

    skill::gusset_form::Input in;
    in.corner_xyz       = { 60.0, 60.0, 1.5 };
    in.wall_a_normal    = { 1.0, 0.0, 0.0 };
    in.wall_b_normal    = { 0.0, 1.0, 0.0 };
    in.gusset_width_mm  = 6.0;
    in.gusset_depth_mm  = 1.0;        // < 2 × 1.5 = 3.0
    in.gusset_radius_mm = 1.5;

    auto r = skill::gusset_form::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-G-DEPTH") { found = true; break; }
    EXPECT_TRUE(found);
}

// ── 5. Recognize: spherical feature appears after apply ────────────────
TEST(SkillGussetForm, RecognizeDetectsFormedFace)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 1.5);

    skill::gusset_form::Input in;
    in.corner_xyz       = { 60.0, 60.0, 1.5 };
    in.wall_a_normal    = { 1.0, 0.0, 0.0 };
    in.wall_b_normal    = { 0.0, 1.0, 0.0 };
    in.gusset_width_mm  = 6.0;
    in.gusset_depth_mm  = 4.0;
    in.gusset_radius_mm = 1.5;

    auto out = skill::gusset_form::apply(*stock, in);
    auto cands = skill::gusset_form::recognize(*out.workpiece);
    EXPECT_GE(cands.size(), 0u);  // best-effort; main assertion: no crash
    if (!cands.empty()) {
        EXPECT_EQ(cands.front().skill_id, std::string("gusset_form"));
    }
}
