// @lat: [[process/test-strategy#auto_chamfer_all_outer_edges]]

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/auto_chamfer_all_outer_edges.hpp"

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
    GProp_GProps p;
    BRepGProp::VolumeProperties(s, p);
    return p.Mass();
}

skill::auto_chamfer_all_outer_edges::Input goodInput()
{
    skill::auto_chamfer_all_outer_edges::Input in;
    in.chamfer_size_mm  = 1.0;
    in.chamfer_size_key = "std";
    in.include_top_edges   = true;
    in.include_bottom_edges = true;
    in.min_wall_thick_mm = 4.0;
    return in;
}

}  // namespace

// ─── T1: apply chamfers every outer convex edge; volume decreases ─────────
TEST(SkillAutoChamferAllOuterEdges, ApplyRemovesMaterial)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 20.0);
    ASSERT_FALSE(stock->shape().IsNull());
    const double v0 = volumeOf(stock->shape());

    auto in  = goodInput();
    auto out = skill::auto_chamfer_all_outer_edges::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    const double v1 = volumeOf(out.workpiece->shape());
    const double delta = v0 - v1;
    EXPECT_GT(delta, 0.0) << "chamfer should REMOVE material";

    // Approximate expected: 12 cuboid edges × 0.5 × c² × avg_len(~40/3)
    // + small relief.  Loose tolerance.
    const double cs = in.chamfer_size_mm;
    const double expected = 12 * 0.5 * cs * cs * 35.0;
    EXPECT_LE(delta, expected * 2.0)
        << "delta " << delta << " expected ≈ " << expected;
}

// ─── T2: validate rejects unknown size key; apply throws ──────────────────
TEST(SkillAutoChamferAllOuterEdges, RejectsUnknownSizeKey)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 20.0);
    auto in    = goodInput();
    in.chamfer_size_key = "XXL";

    auto r = skill::auto_chamfer_all_outer_edges::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-CHAMFER-SPEC") { found = true; break; }
    EXPECT_TRUE(found);
    EXPECT_THROW(skill::auto_chamfer_all_outer_edges::apply(*stock, in),
                 skill::SkillError);
}

// ─── T3: signature carries key + is_compound = true ───────────────────────
TEST(SkillAutoChamferAllOuterEdges, SignatureCarriesKeyCompound)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 20.0);
    auto in    = goodInput();
    auto out   = skill::auto_chamfer_all_outer_edges::apply(*stock, in);

    EXPECT_EQ(out.signature.skill_id,
              std::string("auto_chamfer_all_outer_edges"));
    EXPECT_TRUE(out.signature.pattern["is_compound"].get<bool>());
    EXPECT_EQ(out.signature.pattern["chamfer_size_key"].get<std::string>(),
              "std");
    EXPECT_GE(out.signature.pattern["subfeature_count"].get<int>(), 2);
}

// ─── T4: recognize returns ≥1 candidate ───────────────────────────────────
TEST(SkillAutoChamferAllOuterEdges, RecognizeAtLeastOne)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 20.0);
    auto in    = goodInput();
    auto out   = skill::auto_chamfer_all_outer_edges::apply(*stock, in);

    auto cands = skill::auto_chamfer_all_outer_edges::recognize(*out.workpiece);
    ASSERT_FALSE(cands.empty());
    EXPECT_EQ(cands.front().skill_id,
              std::string("auto_chamfer_all_outer_edges"));
    EXPECT_GT(cands.front().confidence, 0.0);
}

// ─── T5: context — outer edge count > 0 from topology walk ────────────────
TEST(SkillAutoChamferAllOuterEdges, ContextWalksOuterEdges)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 20.0);
    auto in    = goodInput();
    auto out   = skill::auto_chamfer_all_outer_edges::apply(*stock, in);

    const int outerCount     = out.signature.pattern["outer_edge_count"].get<int>();
    const int chamferedCount = out.signature.pattern["chamfered_count"].get<int>();

    EXPECT_GT(outerCount, 0);
    EXPECT_EQ(outerCount, chamferedCount)
        << "every detected outer edge should be chamfered";
    // A cuboid has 12 edges; all should be outer convex.
    EXPECT_LE(outerCount, 12);
}
