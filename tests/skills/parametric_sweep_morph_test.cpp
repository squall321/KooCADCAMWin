// @lat: [[process/test-strategy#skill round-trip]]
//
// parametric_sweep_morph — REAL compound sweep tests.
//   - Sweep solid CUT from stock removes the frustum volume to ±20%.
//   - DFM rejects bad input.
//   - Signature is_compound + subfeature_count ≥ 3.
//   - Recognize replays metadata.
//   - Feature-specific: the cavity bounding box matches the path extents.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/parametric_sweep_morph.hpp"

#include <BRepBndLib.hxx>
#include <BRepGProp.hxx>
#include <Bnd_Box.hxx>
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
double frustumVolume(double r1, double r2, double h)
{
    return (M_PI * h / 3.0) * (r1 * r1 + r1 * r2 + r2 * r2);
}
}

// Test 1 — sweep solid cut removes frustum-volume worth of material.
TEST(SkillParametricSweepMorph, ApplyRemovesSweepVolume)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 40.0);
    const double v0 = volumeOf(stock->shape());

    skill::parametric_sweep_morph::Input in;
    in.path_start         = gp_Pnt(20.0, 20.0, 5.0);
    in.path_end           = gp_Pnt(20.0, 20.0, 35.0);
    in.start_radius       = 6.0;
    in.end_radius         = 3.0;
    in.intermediate_count = 4;

    auto out = skill::parametric_sweep_morph::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    const double v1 = volumeOf(out.workpiece->shape());
    const double removed = v0 - v1;
    const double expected = frustumVolume(6.0, 3.0, 30.0);

    EXPECT_GT(removed, expected * 0.8);
    EXPECT_LT(removed, expected * 1.2);
}

// Test 2 — DFM rejects bad inputs.
TEST(SkillParametricSweepMorph, ValidateRejectsBad)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 40.0);

    // Zero-length path
    skill::parametric_sweep_morph::Input bad;
    bad.path_start = gp_Pnt(20.0, 20.0, 5.0);
    bad.path_end   = gp_Pnt(20.0, 20.0, 5.0);
    bad.start_radius = 4.0; bad.end_radius = 4.0;
    bad.intermediate_count = 3;
    auto r = skill::parametric_sweep_morph::validate(*stock, bad);
    EXPECT_FALSE(r.passed);
    EXPECT_THROW(skill::parametric_sweep_morph::apply(*stock, bad),
                 skill::SkillError);

    // Sub-min radius
    skill::parametric_sweep_morph::Input bad2;
    bad2.path_start = gp_Pnt(0,0,0);
    bad2.path_end   = gp_Pnt(0,0,10);
    bad2.start_radius = 0.1; bad2.end_radius = 0.1;
    bad2.intermediate_count = 3;
    auto r2 = skill::parametric_sweep_morph::validate(*stock, bad2);
    EXPECT_FALSE(r2.passed);
    bool found = false;
    for (const auto& f : r2.findings)
        if (f.code == "DFM-SWEEP-MIN-D") { found = true; break; }
    EXPECT_TRUE(found);

    // Too few intermediates
    skill::parametric_sweep_morph::Input bad3;
    bad3.path_start = gp_Pnt(0,0,0);
    bad3.path_end   = gp_Pnt(0,0,10);
    bad3.start_radius = 4.0; bad3.end_radius = 4.0;
    bad3.intermediate_count = 1;
    auto r3 = skill::parametric_sweep_morph::validate(*stock, bad3);
    EXPECT_FALSE(r3.passed);
}

// Test 3 — signature is_compound + subfeature_count ≥ 3.
TEST(SkillParametricSweepMorph, SignatureIsCompound)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 40.0);
    skill::parametric_sweep_morph::Input in;
    in.path_start = gp_Pnt(20.0, 20.0, 5.0);
    in.path_end   = gp_Pnt(20.0, 20.0, 30.0);
    in.start_radius = 5.0; in.end_radius = 2.5;
    in.intermediate_count = 4;
    auto out = skill::parametric_sweep_morph::apply(*stock, in);

    EXPECT_EQ(out.signature.skill_id, std::string("parametric_sweep_morph"));
    EXPECT_TRUE(out.signature.pattern.value("is_compound", false));
    EXPECT_GE(out.signature.pattern.value("subfeature_count", 0), 3);
    EXPECT_EQ(out.signature.params.value("intermediate_count", 0), 4);
    EXPECT_NEAR(out.signature.params.value("start_radius", 0.0), 5.0, 1e-9);
}

// Test 4 — recognize replays metadata.
TEST(SkillParametricSweepMorph, RecognizeReplays)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 40.0);
    skill::parametric_sweep_morph::Input in;
    in.path_start = gp_Pnt(20.0, 20.0, 5.0);
    in.path_end   = gp_Pnt(20.0, 20.0, 30.0);
    in.start_radius = 5.0; in.end_radius = 2.5;
    in.intermediate_count = 3;
    auto out = skill::parametric_sweep_morph::apply(*stock, in);
    auto cands = skill::parametric_sweep_morph::recognize(*out.workpiece);
    ASSERT_FALSE(cands.empty());
    EXPECT_DOUBLE_EQ(cands.front().confidence, 1.0);
    EXPECT_NEAR(cands.front().recovered_params.value("start_radius", 0.0),
                5.0, 1e-9);
}

// Test 5 — feature-specific: workpiece cavity bbox spans the path Z extent.
TEST(SkillParametricSweepMorph, CavityMatchesPathExtents)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 40.0);
    skill::parametric_sweep_morph::Input in;
    in.path_start = gp_Pnt(20.0, 20.0, 5.0);
    in.path_end   = gp_Pnt(20.0, 20.0, 30.0);
    in.start_radius = 6.0; in.end_radius = 3.0;
    in.intermediate_count = 4;
    auto out = skill::parametric_sweep_morph::apply(*stock, in);

    // Workpiece bbox shouldn't have changed (cavity is internal).
    Bnd_Box bb;
    BRepBndLib::Add(out.workpiece->shape(), bb);
    double xMin, yMin, zMin, xMax, yMax, zMax;
    bb.Get(xMin, yMin, zMin, xMax, yMax, zMax);
    EXPECT_NEAR(zMin, 0.0,  1e-2);
    EXPECT_NEAR(zMax, 40.0, 1e-2);

    // path_length_mm metadata must match the input path.
    EXPECT_NEAR(out.signature.pattern.value("path_length_mm", 0.0), 25.0, 1e-6);
}
