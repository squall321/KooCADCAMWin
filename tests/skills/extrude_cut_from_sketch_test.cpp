// @lat: [[process/test-strategy#skill round-trip]]
//
// extrude_cut_from_sketch — sketch-driven cut prism + Boolean cut.
//
// Tests:
//   1. Apply removes the expected polygon-area × height volume.
//   2. DFM rejects too-few vertices.
//   3. DFM rejects bad draft angle.
//   4. through_all=true punches through the stock thickness.
//   5. Recognize finds the cut walls.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/extrude_cut_from_sketch.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

#include <cmath>
#include <utility>
#include <vector>

using namespace koocadcam;

namespace {

double volumeOf(const TopoDS_Shape& s) {
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}

std::vector<std::pair<double,double>> squareSketch6()
{
    return { {-3.0, -3.0}, {3.0, -3.0}, {3.0, 3.0}, {-3.0, 3.0} };
}

}  // namespace

// ─── 1. Apply removes the expected volume ─────────────────────────────────
TEST(SkillExtrudeCutFromSketch, ApplyRemovesRealVolume)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 10.0);
    ASSERT_FALSE(stock->shape().IsNull());
    const double v0 = volumeOf(stock->shape());

    skill::extrude_cut_from_sketch::Input in;
    in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.polygon    = squareSketch6();
    in.height_mm  = 4.0;

    auto out = skill::extrude_cut_from_sketch::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    const double v1 = volumeOf(out.workpiece->shape());

    // Expected removed = 6 × 6 × 4 = 144 mm³.
    const double expected = 6.0 * 6.0 * 4.0;
    EXPECT_NEAR(v0 - v1, expected, expected * 0.05);
}

// ─── 2. DFM rejects too-few vertices ──────────────────────────────────────
TEST(SkillExtrudeCutFromSketch, ValidateRejectsTooFewVerts)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 10.0);

    skill::extrude_cut_from_sketch::Input in;
    in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.polygon    = { {0.0, 0.0}, {1.0, 0.0} };
    in.height_mm  = 4.0;

    auto r = skill::extrude_cut_from_sketch::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_THROW(skill::extrude_cut_from_sketch::apply(*stock, in),
                 skill::SkillError);
}

// ─── 3. DFM rejects bad draft angle ───────────────────────────────────────
TEST(SkillExtrudeCutFromSketch, ValidateRejectsBadDraft)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 10.0);

    skill::extrude_cut_from_sketch::Input in;
    in.entry_face     = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.polygon        = squareSketch6();
    in.height_mm      = 4.0;
    in.draft_angle_deg = 45.0;          // > 30 max

    auto r = skill::extrude_cut_from_sketch::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-DRAFT") { found = true; break; }
    EXPECT_TRUE(found);
    EXPECT_THROW(skill::extrude_cut_from_sketch::apply(*stock, in),
                 skill::SkillError);
}

// ─── 4. through_all=true punches through ──────────────────────────────────
TEST(SkillExtrudeCutFromSketch, ThroughAllPunchesAll)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 10.0);
    const double v0 = volumeOf(stock->shape());

    skill::extrude_cut_from_sketch::Input in;
    in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.polygon    = squareSketch6();
    in.height_mm  = 1.0;            // small, but through_all forces full depth
    in.through_all = true;

    auto out = skill::extrude_cut_from_sketch::apply(*stock, in);
    const double v1 = volumeOf(out.workpiece->shape());

    // Should remove the full 6×6×10 column (stock thickness).
    const double expected = 6.0 * 6.0 * 10.0;
    EXPECT_NEAR(v0 - v1, expected, expected * 0.05);
}

// ─── 5. Recognize finds the cut ───────────────────────────────────────────
TEST(SkillExtrudeCutFromSketch, RecognizeFindsCut)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 10.0);

    skill::extrude_cut_from_sketch::Input in;
    in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.polygon    = squareSketch6();
    in.height_mm  = 4.0;

    auto out = skill::extrude_cut_from_sketch::apply(*stock, in);
    auto cands = skill::extrude_cut_from_sketch::recognize(*out.workpiece);
    ASSERT_FALSE(cands.empty());
    EXPECT_GT(cands.front().confidence, 0.3);
}
