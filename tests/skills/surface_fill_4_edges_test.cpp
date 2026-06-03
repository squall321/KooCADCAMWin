// @lat: [[process/test-strategy#skill round-trip]]
//
// surface_fill_4_edges — fill a 4-boundary region with BRepFill_Filling.
//
// Cases (5):
//   1. ApplyOnSquareProducesPositiveArea (≈ 100 mm² for a 10×10 square).
//   2. ValidateRejectsShortPolyline (single point).
//   3. SignatureRecordsKindAndEdgeCount.
//   4. PerimeterRecorded — sum of polyline lengths ≈ 40 for a 10×10 square.
//   5. RecognizeMetadataReplay.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/surface_fill_4_edges.hpp"

#include <gp_Pnt.hxx>

using namespace koocadcam;

namespace {

// Build the 4 boundary polylines of an axis-aligned square in the XY plane,
// from (0,0,0) to (side, side, 0).  Each polyline has 2 points (a straight
// segment).  Ordering: bottom, right, top, left.
skill::surface_fill_4_edges::Input makeSquareInput(double side)
{
    skill::surface_fill_4_edges::Input in;
    in.boundary_edges[0] = { gp_Pnt(0,    0,    0), gp_Pnt(side, 0,    0) };
    in.boundary_edges[1] = { gp_Pnt(side, 0,    0), gp_Pnt(side, side, 0) };
    in.boundary_edges[2] = { gp_Pnt(side, side, 0), gp_Pnt(0,    side, 0) };
    in.boundary_edges[3] = { gp_Pnt(0,    side, 0), gp_Pnt(0,    0,    0) };
    return in;
}

}  // namespace

// ─── 1. fill a 10×10 square → area ≈ 100 ──────────────────────────────────
TEST(SkillSurfaceFill4Edges, ApplyOnSquareProducesArea)
{
    auto stock = skill::createCuboidStock(20.0, 20.0, 5.0);
    auto in = makeSquareInput(10.0);
    auto out = skill::surface_fill_4_edges::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    EXPECT_NEAR(out.signature.pattern["derived_area"].get<double>(),
                100.0, 5.0);
}

// ─── 2. DFM rejects polyline with < 2 points ──────────────────────────────
TEST(SkillSurfaceFill4Edges, ValidateRejectsShortPolyline)
{
    auto stock = skill::createCuboidStock(20.0, 20.0, 5.0);
    auto in = makeSquareInput(10.0);
    in.boundary_edges[1] = { gp_Pnt(10, 0, 0) };          // only 1 point
    auto r = skill::surface_fill_4_edges::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool sawCode = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-EDGE-LENGTH") { sawCode = true; break; }
    EXPECT_TRUE(sawCode);
    EXPECT_THROW(skill::surface_fill_4_edges::apply(*stock, in),
                 skill::SkillError);
}

// ─── 3. signature records kind & edge_count = 4 ───────────────────────────
TEST(SkillSurfaceFill4Edges, SignatureRecordsKindAndEdgeCount)
{
    auto stock = skill::createCuboidStock(20.0, 20.0, 5.0);
    auto in = makeSquareInput(10.0);
    auto out = skill::surface_fill_4_edges::apply(*stock, in);
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("surface_fill_4_edges"));
    EXPECT_TRUE(out.signature.pattern["is_compound"].get<bool>());
    EXPECT_EQ(out.signature.pattern["edge_count"].get<int>(), 4);
    EXPECT_EQ(out.signature.pattern["source_face_count"].get<int>(), 0);
}

// ─── 4. perimeter ≈ 40 mm for a 10×10 square ──────────────────────────────
TEST(SkillSurfaceFill4Edges, PerimeterRecorded)
{
    auto stock = skill::createCuboidStock(20.0, 20.0, 5.0);
    auto in = makeSquareInput(10.0);
    auto out = skill::surface_fill_4_edges::apply(*stock, in);
    EXPECT_NEAR(out.signature.pattern["derived_perimeter"].get<double>(),
                40.0, 1e-6);
}

// ─── 5. recognize replay ──────────────────────────────────────────────────
TEST(SkillSurfaceFill4Edges, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(20.0, 20.0, 5.0);
    auto in = makeSquareInput(10.0);
    auto out = skill::surface_fill_4_edges::apply(*stock, in);
    auto cands = skill::surface_fill_4_edges::recognize(*out.workpiece);
    ASSERT_GE(cands.size(), 1u);
    EXPECT_EQ(cands.front().skill_id,
              std::string("surface_fill_4_edges"));
    EXPECT_DOUBLE_EQ(cands.front().confidence, 1.0);
}
