// @lat: [[process/test-strategy#skill round-trip]]
//
// extrude_boss_from_sketch — sketch-driven boss prism + fuse.
//
// Tests:
//   1. Apply adds the expected polygon-area × height volume.
//   2. DFM rejects too-few vertices.
//   3. DFM rejects zero / negative height.
//   4. Signature pattern carries the sketch vertex count.
//   5. Recognize finds the new vertical walls.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/extrude_boss_from_sketch.hpp"

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

// Square sketch: 10 × 10 mm centered on (0, 0) -> verts at (±5, ±5).
std::vector<std::pair<double,double>> squareSketch10()
{
    return { {-5.0, -5.0}, {5.0, -5.0}, {5.0, 5.0}, {-5.0, 5.0} };
}

}  // namespace

// ─── 1. Apply adds the expected volume ────────────────────────────────────
TEST(SkillExtrudeBossFromSketch, ApplyAddsRealVolume)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 10.0);
    ASSERT_FALSE(stock->shape().IsNull());
    const double v0 = volumeOf(stock->shape());

    skill::extrude_boss_from_sketch::Input in;
    in.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    // Sketch in face-local XY; stock top face is centered at (30, 30, 10).
    in.polygon      = squareSketch10();
    in.height_mm    = 4.0;

    auto out = skill::extrude_boss_from_sketch::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    const double v1 = volumeOf(out.workpiece->shape());

    // Expected added volume = 10 × 10 × 4 = 400 mm³.
    const double expected = 10.0 * 10.0 * 4.0;
    EXPECT_NEAR(v1 - v0, expected, expected * 0.05);
}

// ─── 2. DFM rejects too-few vertices ──────────────────────────────────────
TEST(SkillExtrudeBossFromSketch, ValidateRejectsTooFewVerts)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 10.0);

    skill::extrude_boss_from_sketch::Input in;
    in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.polygon    = { {0.0, 0.0}, {1.0, 0.0} };  // only 2 verts
    in.height_mm  = 4.0;

    auto r = skill::extrude_boss_from_sketch::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_THROW(skill::extrude_boss_from_sketch::apply(*stock, in),
                 skill::SkillError);
}

// ─── 3. DFM rejects non-positive height ───────────────────────────────────
TEST(SkillExtrudeBossFromSketch, ValidateRejectsBadHeight)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 10.0);

    skill::extrude_boss_from_sketch::Input in;
    in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.polygon    = squareSketch10();
    in.height_mm  = -1.0;

    auto r = skill::extrude_boss_from_sketch::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_THROW(skill::extrude_boss_from_sketch::apply(*stock, in),
                 skill::SkillError);
}

// ─── 4. Signature carries the sketch vertex count ────────────────────────
TEST(SkillExtrudeBossFromSketch, SignatureCompound)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 10.0);

    skill::extrude_boss_from_sketch::Input in;
    in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.polygon    = squareSketch10();
    in.height_mm  = 4.0;

    auto out = skill::extrude_boss_from_sketch::apply(*stock, in);
    EXPECT_EQ(out.signature.skill_id, std::string("extrude_boss_from_sketch"));
    EXPECT_TRUE(out.signature.pattern.value("is_compound", false));
    EXPECT_EQ(out.signature.pattern.value("sketch_vertex_count", 0), 4);
    EXPECT_NEAR(out.signature.pattern.value("height_mm", 0.0),
                in.height_mm, 1e-9);
    EXPECT_NEAR(out.signature.pattern.value("derived_volume_mm3", 0.0),
                400.0, 1.0);
}

// ─── 5. Recognize finds the boss ──────────────────────────────────────────
TEST(SkillExtrudeBossFromSketch, RecognizeFindsBoss)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 10.0);

    skill::extrude_boss_from_sketch::Input in;
    in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.polygon    = squareSketch10();
    in.height_mm  = 4.0;

    auto out = skill::extrude_boss_from_sketch::apply(*stock, in);
    auto cands = skill::extrude_boss_from_sketch::recognize(*out.workpiece);
    ASSERT_FALSE(cands.empty());
    EXPECT_GT(cands.front().confidence, 0.5);
}
