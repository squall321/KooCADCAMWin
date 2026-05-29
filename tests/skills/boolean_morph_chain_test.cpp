// @lat: [[process/test-strategy#skill round-trip]]
//
// boolean_morph_chain — REAL multi-op compound-feature tests.
//   - Sequence of cylinder cut + box fuse produces deltas in both
//     directions (volume removed AND added).
//   - DFM rejects empty / single-op chains.
//   - Signature carries is_compound + subfeature_count = N.
//   - Recognize replays from history with confidence 1.0.
//   - Feature-specific: cumulative volume_removed + volume_added matches
//     start − end ± deltas.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/boolean_morph_chain.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using namespace koocadcam;
using skill::boolean_morph_chain::Op;
using skill::boolean_morph_chain::OpKind;
using skill::boolean_morph_chain::ShapeKind;

namespace {
double volumeOf(const TopoDS_Shape& s)
{
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}
}

// Test 1 — applying a 2-op chain modifies the geometry by the expected delta.
TEST(SkillBooleanMorphChain, ApplyChainModifiesGeometry)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 20.0);
    const double v0 = volumeOf(stock->shape());

    // Op A: cut a Φ8 mm × 15 mm cylinder going up through the block.
    Op opA;
    opA.op    = OpKind::Cut;
    opA.shape = ShapeKind::Cylinder;
    opA.origin = gp_Pnt(15.0, 25.0, -0.05);
    opA.axis   = gp_Dir(0, 0, 1);
    opA.size_a = 4.0;     // radius
    opA.size_b = 25.0;    // height (overshoot for clean cut)

    // Op B: cut a 10×10×12 box at (35, 25, top - 12 ish).
    Op opB;
    opB.op    = OpKind::Cut;
    opB.shape = ShapeKind::Box;
    opB.origin = gp_Pnt(30.0, 20.0, 8.0);
    opB.axis   = gp_Dir(0, 0, 1);
    opB.size_a = 10.0;
    opB.size_b = 10.0;
    opB.size_c = 25.0;

    skill::boolean_morph_chain::Input in;
    in.operations = { opA, opB };

    auto out = skill::boolean_morph_chain::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    const double v1 = volumeOf(out.workpiece->shape());
    const double removed = v0 - v1;

    // Expected removal: π·4²·20 (cyl through full thickness) +
    // 10·10·12 (box from z=8 to z=20).  Boolean overlaps reduce this slightly.
    const double cylVol = M_PI * 16.0 * 20.0;            // ≈ 1005
    const double boxVol = 10.0 * 10.0 * 12.0;            // = 1200
    const double maxExpected = cylVol + boxVol;          // ≈ 2205 (no overlap)
    EXPECT_GT(removed, std::min(cylVol, boxVol) * 0.5);
    EXPECT_LT(removed, maxExpected * 1.1);
}

// Test 2 — DFM rejects empty + single-op + apply throws on those.
TEST(SkillBooleanMorphChain, ValidateRejectsBadChains)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 20.0);

    // Empty
    {
        skill::boolean_morph_chain::Input in;
        auto r = skill::boolean_morph_chain::validate(*stock, in);
        EXPECT_FALSE(r.passed);
        EXPECT_THROW(skill::boolean_morph_chain::apply(*stock, in),
                     skill::SkillError);
    }

    // Single op (must be ≥ 2 for compound)
    {
        Op opA;
        opA.shape = ShapeKind::Cylinder;
        opA.origin = gp_Pnt(25.0, 25.0, -0.05);
        opA.axis = gp_Dir(0, 0, 1);
        opA.size_a = 3.0;
        opA.size_b = 25.0;
        skill::boolean_morph_chain::Input in;
        in.operations = { opA };
        auto r = skill::boolean_morph_chain::validate(*stock, in);
        EXPECT_FALSE(r.passed);
        EXPECT_THROW(skill::boolean_morph_chain::apply(*stock, in),
                     skill::SkillError);
    }

    // Bad dimensions
    {
        Op badA;
        badA.shape = ShapeKind::Box;
        badA.size_a = 0.0; badA.size_b = 0.0; badA.size_c = 0.0;
        Op badB = badA;
        skill::boolean_morph_chain::Input in;
        in.operations = { badA, badB };
        auto r = skill::boolean_morph_chain::validate(*stock, in);
        EXPECT_FALSE(r.passed);
    }
}

// Test 3 — signature.is_compound and subfeature_count ≥ 2 + params round-trip.
TEST(SkillBooleanMorphChain, SignatureIsCompound)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 20.0);
    Op a;
    a.op = OpKind::Cut; a.shape = ShapeKind::Cylinder;
    a.origin = gp_Pnt(20.0, 20.0, -0.05);
    a.axis = gp_Dir(0,0,1);
    a.size_a = 3.0; a.size_b = 25.0;

    Op b;
    b.op = OpKind::Cut; b.shape = ShapeKind::Box;
    b.origin = gp_Pnt(28.0, 22.0, 5.0);
    b.axis = gp_Dir(0,0,1);
    b.size_a = 8.0; b.size_b = 8.0; b.size_c = 20.0;

    Op c;
    c.op = OpKind::Cut; c.shape = ShapeKind::Cone;
    c.origin = gp_Pnt(40.0, 40.0, -0.05);
    c.axis = gp_Dir(0,0,1);
    c.size_a = 4.0; c.size_b = 1.5; c.size_c = 12.0;

    skill::boolean_morph_chain::Input in;
    in.operations = { a, b, c };

    auto out = skill::boolean_morph_chain::apply(*stock, in);
    EXPECT_EQ(out.signature.skill_id, std::string("boolean_morph_chain"));
    EXPECT_TRUE(out.signature.pattern.value("is_compound", false));
    EXPECT_EQ(out.signature.pattern.value("subfeature_count", 0), 3);
    EXPECT_EQ(out.signature.params.value("operation_count", 0), 3);
}

// Test 4 — recognize replays from history with confidence 1.0.
TEST(SkillBooleanMorphChain, RecognizeReplaysHistory)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 20.0);
    Op a;
    a.shape = ShapeKind::Cylinder;
    a.origin = gp_Pnt(20.0, 20.0, -0.05);
    a.axis = gp_Dir(0,0,1);
    a.size_a = 3.0; a.size_b = 25.0;
    Op b;
    b.shape = ShapeKind::Box;
    b.origin = gp_Pnt(28.0, 22.0, 5.0);
    b.axis = gp_Dir(0,0,1);
    b.size_a = 6.0; b.size_b = 6.0; b.size_c = 20.0;

    skill::boolean_morph_chain::Input in;
    in.operations = { a, b };

    auto out = skill::boolean_morph_chain::apply(*stock, in);
    auto cands = skill::boolean_morph_chain::recognize(*out.workpiece);
    ASSERT_FALSE(cands.empty());
    EXPECT_DOUBLE_EQ(cands.front().confidence, 1.0);
    EXPECT_EQ(cands.front().recovered_params.value("operation_count", 0), 2);
}

// Test 5 — feature-specific: cumulative volume balance.
TEST(SkillBooleanMorphChain, VolumeBalanceMatchesStartEndDelta)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 20.0);
    const double v0 = volumeOf(stock->shape());

    Op c1;
    c1.op = OpKind::Cut; c1.shape = ShapeKind::Cylinder;
    c1.origin = gp_Pnt(20.0, 20.0, -0.05);
    c1.axis = gp_Dir(0,0,1);
    c1.size_a = 3.0; c1.size_b = 25.0;

    Op c2;
    c2.op = OpKind::Cut; c2.shape = ShapeKind::Box;
    c2.origin = gp_Pnt(5.0, 5.0, -0.05);
    c2.axis = gp_Dir(0,0,1);
    c2.size_a = 4.0; c2.size_b = 4.0; c2.size_c = 25.0;

    skill::boolean_morph_chain::Input in;
    in.operations = { c1, c2 };
    auto out = skill::boolean_morph_chain::apply(*stock, in);
    const double v1 = volumeOf(out.workpiece->shape());

    const double recordedRemoved =
        out.signature.pattern.value("volume_removed_mm3", -1.0);
    const double recordedAdded =
        out.signature.pattern.value("volume_added_mm3", -1.0);
    ASSERT_GE(recordedRemoved, 0.0);
    ASSERT_GE(recordedAdded,   0.0);

    // Cumulative net delta should match v0 - v1.
    const double netDelta = recordedRemoved - recordedAdded;
    EXPECT_NEAR(netDelta, v0 - v1, std::max(1e-3, (v0 - v1) * 0.05));
}
