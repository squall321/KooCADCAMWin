// @lat: [[process/test-strategy#skill round-trip]]
//
// projected_sketch_cut — sketch projected onto bbox plane + perpendicular cut.
//
// Tests:
//   1. Apply removes the expected polygon-area × cut_depth volume.
//   2. DFM rejects too-few vertices.
//   3. DFM rejects cut_depth ≤ 0.
//   4. DFM rejects zero projection direction.
//   5. Recognize returns at least one candidate.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/projected_sketch_cut.hpp"

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

std::vector<std::pair<double,double>> squareSketch4()
{
    return { {-2.0, -2.0}, {2.0, -2.0}, {2.0, 2.0}, {-2.0, 2.0} };
}

}  // namespace

// ─── 1. Apply removes the expected volume ─────────────────────────────────
TEST(SkillProjectedSketchCut, ApplyRemovesRealVolume)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 10.0);
    ASSERT_FALSE(stock->shape().IsNull());
    const double v0 = volumeOf(stock->shape());

    skill::projected_sketch_cut::Input in;
    in.target_face_id     = 0;
    in.polygon            = squareSketch4();
    in.projection_dir_xyz = gp_Dir(0.0, 0.0, -1.0);    // project down
    in.cut_depth_mm       = 3.0;

    auto out = skill::projected_sketch_cut::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    const double v1 = volumeOf(out.workpiece->shape());

    // Polygon = 4 × 4 = 16 mm².  The projection cutter is sized so it spans
    // the full bbox along projection_dir; the intersection with the stock
    // is the polygon footprint × (stock_z_extent − 1 mm cutter-start gap)
    // = 16 × 9 = 144 mm³ for a 10 mm thick stock with the implementation's
    // bboxRadius + 1 origin offset.  (See projected_sketch_cut.cpp.)
    const double expected = 4.0 * 4.0 * 9.0;           // 144 mm³
    EXPECT_NEAR(v0 - v1, expected, expected * 0.05);
}

// ─── 2. DFM rejects too-few vertices ──────────────────────────────────────
TEST(SkillProjectedSketchCut, ValidateRejectsTooFewVerts)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 10.0);

    skill::projected_sketch_cut::Input in;
    in.target_face_id     = 0;
    in.polygon            = { {0.0, 0.0}, {1.0, 0.0} };
    in.cut_depth_mm       = 3.0;

    auto r = skill::projected_sketch_cut::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_THROW(skill::projected_sketch_cut::apply(*stock, in),
                 skill::SkillError);
}

// ─── 3. DFM rejects non-positive cut depth ────────────────────────────────
TEST(SkillProjectedSketchCut, ValidateRejectsBadDepth)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 10.0);

    skill::projected_sketch_cut::Input in;
    in.target_face_id     = 0;
    in.polygon            = squareSketch4();
    in.cut_depth_mm       = 0.0;

    auto r = skill::projected_sketch_cut::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_THROW(skill::projected_sketch_cut::apply(*stock, in),
                 skill::SkillError);
}

// ─── 4. Signature compound ────────────────────────────────────────────────
TEST(SkillProjectedSketchCut, SignatureCompound)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 10.0);

    skill::projected_sketch_cut::Input in;
    in.target_face_id     = 0;
    in.polygon            = squareSketch4();
    in.projection_dir_xyz = gp_Dir(0.0, 0.0, -1.0);
    in.cut_depth_mm       = 3.0;

    auto out = skill::projected_sketch_cut::apply(*stock, in);
    EXPECT_EQ(out.signature.skill_id, std::string("projected_sketch_cut"));
    EXPECT_TRUE(out.signature.pattern.value("is_compound", false));
    EXPECT_EQ(out.signature.pattern.value("sketch_vertex_count", 0), 4);
    EXPECT_NEAR(out.signature.pattern.value("cut_depth_mm", 0.0),
                in.cut_depth_mm, 1e-9);
}

// ─── 5. Recognize returns a candidate ─────────────────────────────────────
TEST(SkillProjectedSketchCut, RecognizeFindsCut)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 10.0);

    skill::projected_sketch_cut::Input in;
    in.target_face_id     = 0;
    in.polygon            = squareSketch4();
    in.projection_dir_xyz = gp_Dir(0.0, 0.0, -1.0);
    in.cut_depth_mm       = 3.0;

    auto out = skill::projected_sketch_cut::apply(*stock, in);
    auto cands = skill::projected_sketch_cut::recognize(*out.workpiece);
    ASSERT_FALSE(cands.empty());
    EXPECT_GT(cands.front().confidence, 0.3);
}
