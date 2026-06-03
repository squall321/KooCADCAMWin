// @lat: [[process/test-strategy#skill round-trip]]
//
// boundary_surface_4edges — build a free-form face bounded by 4 edges
// using BRepFill_Filling, and attach it onto the workpiece as a compound.
//
// Tests (5):
//   1. Apply produces a compound including a non-null face (no volume change
//      since a face has zero volume).
//   2. DFM rejects boundary with < 2 points.
//   3. DFM rejects zero-length boundary (single repeated point).
//   4. Signature carries is_compound + section_count=4 + path_length.
//   5. Recognize replays history.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/boundary_surface_4edges.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

#include <array>
#include <cmath>
#include <vector>

using namespace koocadcam;

namespace {

double volumeOf(const TopoDS_Shape& s)
{
    GProp_GProps p;
    BRepGProp::VolumeProperties(s, p);
    return p.Mass();
}

// 4 boundaries forming a 10x10 square loop in z = 50, well clear of the stock.
std::array<std::vector<gp_Pnt>, 4> squareLoop10()
{
    return {{
        { gp_Pnt(100.0, 100.0, 50.0), gp_Pnt(110.0, 100.0, 50.0) },  // bottom
        { gp_Pnt(110.0, 100.0, 50.0), gp_Pnt(110.0, 110.0, 50.0) },  // right
        { gp_Pnt(110.0, 110.0, 50.0), gp_Pnt(100.0, 110.0, 50.0) },  // top
        { gp_Pnt(100.0, 110.0, 50.0), gp_Pnt(100.0, 100.0, 50.0) },  // left
    }};
}

}  // namespace

// ─── 1. Apply produces a filled face; profile area × length ≈ 100 mm² ─────
//
// A face has no proper solid volume — instead we verify that the apply
// succeeded (signature.pattern.fill_done == true) and the geometry math
// reports a reasonable boundary perimeter that matches the input loop.
// The shape itself is a compound of the original workpiece + the filled
// face, so it must be non-null.
TEST(SkillBoundarySurface4Edges, ApplyProducesFace)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 5.0);

    skill::boundary_surface_4edges::Input in;
    in.boundaries = squareLoop10();

    auto out = skill::boundary_surface_4edges::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    // Filling must have completed.
    EXPECT_TRUE(out.signature.pattern.value("fill_done", false));

    // The pattern reports geometric construction — boundary perimeter sums
    // to 4 × 10 = 40 mm (each side is one straight edge of 10 mm).
    const double sumLen = out.signature.pattern.value("path_length_mm", 0.0);
    EXPECT_NEAR(sumLen, 40.0, 1e-6);
}

// ─── 2. DFM rejects too-short boundary ────────────────────────────────────
TEST(SkillBoundarySurface4Edges, ValidateRejectsShortBoundary)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 5.0);

    skill::boundary_surface_4edges::Input in;
    in.boundaries = squareLoop10();
    in.boundaries[0] = { gp_Pnt(0.0, 0.0, 0.0) };   // 1 pt

    auto r = skill::boundary_surface_4edges::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_THROW(skill::boundary_surface_4edges::apply(*stock, in),
                 skill::SkillError);
}

// ─── 3. DFM rejects zero-length boundary ──────────────────────────────────
TEST(SkillBoundarySurface4Edges, ValidateRejectsZeroLenBoundary)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 5.0);

    skill::boundary_surface_4edges::Input in;
    in.boundaries = squareLoop10();
    in.boundaries[0] = {
        gp_Pnt(0.0, 0.0, 0.0), gp_Pnt(0.0, 0.0, 0.0)    // coincident
    };

    auto r = skill::boundary_surface_4edges::validate(*stock, in);
    EXPECT_FALSE(r.passed);
}

// ─── 4. Signature pattern ─────────────────────────────────────────────────
TEST(SkillBoundarySurface4Edges, SignatureCompound)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 5.0);

    skill::boundary_surface_4edges::Input in;
    in.boundaries = squareLoop10();

    auto out = skill::boundary_surface_4edges::apply(*stock, in);
    EXPECT_EQ(out.signature.skill_id, std::string("boundary_surface_4edges"));
    EXPECT_TRUE(out.signature.pattern.value("is_compound", false));
    EXPECT_EQ(out.signature.pattern.value("section_count", 0), 4);
    EXPECT_NEAR(out.signature.pattern.value("path_length_mm", 0.0),
                40.0, 1e-6);
}

// ─── 5. Recognize replays history ─────────────────────────────────────────
TEST(SkillBoundarySurface4Edges, RecognizeReplaysHistory)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 5.0);

    skill::boundary_surface_4edges::Input in;
    in.boundaries = squareLoop10();

    auto out   = skill::boundary_surface_4edges::apply(*stock, in);
    auto cands = skill::boundary_surface_4edges::recognize(*out.workpiece);
    ASSERT_GE(cands.size(), 1u);
    EXPECT_EQ(cands.front().skill_id, std::string("boundary_surface_4edges"));
    EXPECT_GT(cands.front().confidence, 0.4);
}
