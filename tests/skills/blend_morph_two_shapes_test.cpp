// @lat: [[process/test-strategy#skill round-trip]]
//
// blend_morph_two_shapes — REAL compound-feature tests.
//   - ThruSections truncated-cone bridge fused onto stock changes volume
//     within ±20% of the analytic frustum volume formula.
//   - DFM rejects bad input.
//   - Signature carries is_compound + subfeature_count ≥ 3.
//   - Recognize replays metadata.
//   - Feature-specific: blend_volume_mm3 matches frustum formula.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/blend_morph_two_shapes.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using namespace koocadcam;

namespace {
double volumeOf(const TopoDS_Shape& s)
{
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}

// Analytic frustum (truncated cone) volume formula:
//   V = (π h / 3) (r1² + r1 r2 + r2²)
double frustumVolume(double r1, double r2, double h)
{
    return (M_PI * h / 3.0) * (r1 * r1 + r1 * r2 + r2 * r2);
}
}

// Test 1 — applying the skill grows the workpiece volume by the bridge
// frustum amount.
TEST(SkillBlendMorphTwoShapes, ApplyAddsBridge)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 30.0);
    const double v0 = volumeOf(stock->shape());

    skill::blend_morph_two_shapes::Input in;
    in.centre_a = gp_Pnt(30.0, 30.0, 30.0);
    in.radius_a = 10.0;
    in.z_a      = 30.0;
    in.centre_b = gp_Pnt(30.0, 30.0, 50.0);
    in.radius_b = 5.0;
    in.z_b      = 50.0;

    auto out = skill::blend_morph_two_shapes::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    const double v1 = volumeOf(out.workpiece->shape());
    const double added = v1 - v0;
    const double expected = frustumVolume(10.0, 5.0, 20.0);

    EXPECT_GT(added, expected * 0.7);
    EXPECT_LT(added, expected * 1.3);
}

// Test 2 — DFM rejects bad inputs.
TEST(SkillBlendMorphTwoShapes, ValidateRejectsBad)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 30.0);

    // Zero radius
    skill::blend_morph_two_shapes::Input bad;
    bad.centre_a = gp_Pnt(0,0,0); bad.radius_a = 0.0; bad.z_a = 0.0;
    bad.centre_b = gp_Pnt(0,0,0); bad.radius_b = 5.0; bad.z_b = 10.0;
    auto r = skill::blend_morph_two_shapes::validate(*stock, bad);
    EXPECT_FALSE(r.passed);
    EXPECT_THROW(skill::blend_morph_two_shapes::apply(*stock, bad),
                 skill::SkillError);

    // Coincident Z (no extrusion possible)
    skill::blend_morph_two_shapes::Input bad2;
    bad2.centre_a = gp_Pnt(0,0,5); bad2.radius_a = 5.0; bad2.z_a = 5.0;
    bad2.centre_b = gp_Pnt(0,0,5); bad2.radius_b = 8.0; bad2.z_b = 5.0;
    auto r2 = skill::blend_morph_two_shapes::validate(*stock, bad2);
    EXPECT_FALSE(r2.passed);
    EXPECT_THROW(skill::blend_morph_two_shapes::apply(*stock, bad2),
                 skill::SkillError);

    // Sub-min radius
    skill::blend_morph_two_shapes::Input bad3;
    bad3.centre_a = gp_Pnt(0,0,0); bad3.radius_a = 0.2; bad3.z_a = 0.0;
    bad3.centre_b = gp_Pnt(0,0,5); bad3.radius_b = 0.3; bad3.z_b = 5.0;
    auto r3 = skill::blend_morph_two_shapes::validate(*stock, bad3);
    EXPECT_FALSE(r3.passed);
    bool found = false;
    for (const auto& f : r3.findings)
        if (f.code == "DFM-BLEND-MIN-D") { found = true; break; }
    EXPECT_TRUE(found);
}

// Test 3 — signature carries is_compound + subfeature_count ≥ 3.
TEST(SkillBlendMorphTwoShapes, SignatureIsCompound)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 30.0);
    skill::blend_morph_two_shapes::Input in;
    in.centre_a = gp_Pnt(30.0, 30.0, 30.0);
    in.radius_a = 8.0; in.z_a = 30.0;
    in.centre_b = gp_Pnt(30.0, 30.0, 45.0);
    in.radius_b = 4.0; in.z_b = 45.0;
    auto out = skill::blend_morph_two_shapes::apply(*stock, in);
    EXPECT_EQ(out.signature.skill_id, std::string("blend_morph_two_shapes"));
    EXPECT_TRUE(out.signature.pattern.value("is_compound", false));
    EXPECT_GE(out.signature.pattern.value("subfeature_count", 0), 3);

    // section_sources must be present (recorded provenance).
    ASSERT_TRUE(out.signature.pattern.contains("section_sources"));
    EXPECT_NEAR(
        out.signature.pattern["section_sources"]["a"].value("radius", 0.0),
        8.0, 1e-9);
}

// Test 4 — recognize replays metadata, confidence 1.0.
TEST(SkillBlendMorphTwoShapes, RecognizeReplays)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 30.0);
    skill::blend_morph_two_shapes::Input in;
    in.centre_a = gp_Pnt(30.0, 30.0, 30.0);
    in.radius_a = 8.0; in.z_a = 30.0;
    in.centre_b = gp_Pnt(30.0, 30.0, 45.0);
    in.radius_b = 4.0; in.z_b = 45.0;
    auto out = skill::blend_morph_two_shapes::apply(*stock, in);
    auto cands = skill::blend_morph_two_shapes::recognize(*out.workpiece);
    ASSERT_FALSE(cands.empty());
    EXPECT_DOUBLE_EQ(cands.front().confidence, 1.0);
    EXPECT_NEAR(cands.front().recovered_params.value("radius_a", 0.0),
                8.0, 1e-9);
    EXPECT_NEAR(cands.front().recovered_params.value("radius_b", 0.0),
                4.0, 1e-9);
}

// Test 5 — feature-specific: recorded blend_volume_mm3 matches
// analytic frustum volume within ±15%.
TEST(SkillBlendMorphTwoShapes, BlendVolumeMatchesFrustum)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 30.0);
    skill::blend_morph_two_shapes::Input in;
    in.centre_a = gp_Pnt(30.0, 30.0, 30.0);
    in.radius_a = 10.0; in.z_a = 30.0;
    in.centre_b = gp_Pnt(30.0, 30.0, 50.0);
    in.radius_b = 5.0;  in.z_b = 50.0;
    auto out = skill::blend_morph_two_shapes::apply(*stock, in);
    const double recordedVol =
        out.signature.pattern.value("blend_volume_mm3", -1.0);
    ASSERT_GT(recordedVol, 0.0);
    const double expected = frustumVolume(10.0, 5.0, 20.0);
    EXPECT_NEAR(recordedVol, expected, expected * 0.15);
}
