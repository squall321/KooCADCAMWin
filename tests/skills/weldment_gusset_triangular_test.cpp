// @lat: [[process/test-strategy#skill round-trip]]
//
// weldment_gusset_triangular — triangular gusset plate between two
// intersecting members.
//
// Tests (5):
//   1. Apply: real volume ≈ tri-area × thickness.
//   2. DFM rejects zero thickness.
//   3. DFM rejects collinear legs.
//   4. Signature pattern carries is_compound + profile_kind = "triangle".
//   5. Recognize replays the feature from history.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/weldment_gusset_triangular.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

#include <array>

using namespace koocadcam;

namespace {

double volumeOf(const TopoDS_Shape& s)
{
    GProp_GProps p;
    BRepGProp::VolumeProperties(s, p);
    return p.Mass();
}

}  // namespace

// ─── 1. Apply: real volume ≈ tri-area × thickness ─────────────────────────
TEST(SkillWeldmentGussetTriangular, ApplyAddsRealVolume)
{
    auto stock = skill::createCuboidStock(20.0, 20.0, 20.0);
    const double v0 = volumeOf(stock->shape());

    skill::weldment_gusset_triangular::Input in;
    in.vertex_xyz           = { 200.0, 0.0, 0.0 };
    in.leg_a_xyz            = { 1.0, 0.0, 0.0 };
    in.leg_b_xyz            = { 0.0, 1.0, 0.0 };
    in.gusset_thickness_mm  = 5.0;
    in.gusset_leg_length_mm = 80.0;

    auto out = skill::weldment_gusset_triangular::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    const double v1 = volumeOf(out.workpiece->shape());

    // Triangle area = 0.5 × 80 × 80 = 3200 mm² ;  × 5 mm = 16 000 mm³.
    const double expected = 0.5 * 80.0 * 80.0 * 5.0;
    EXPECT_NEAR(v1 - v0, expected, expected * 0.20);
}

// ─── 2. DFM rejects zero thickness ────────────────────────────────────────
TEST(SkillWeldmentGussetTriangular, ValidateRejectsZeroThickness)
{
    auto stock = skill::createCuboidStock(20.0, 20.0, 20.0);

    skill::weldment_gusset_triangular::Input in;
    in.vertex_xyz           = { 0.0, 0.0, 0.0 };
    in.leg_a_xyz            = { 1.0, 0.0, 0.0 };
    in.leg_b_xyz            = { 0.0, 1.0, 0.0 };
    in.gusset_thickness_mm  = 0.0;
    in.gusset_leg_length_mm = 50.0;

    auto r = skill::weldment_gusset_triangular::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_THROW(skill::weldment_gusset_triangular::apply(*stock, in),
                 skill::SkillError);
}

// ─── 3. DFM rejects collinear legs ────────────────────────────────────────
TEST(SkillWeldmentGussetTriangular, ValidateRejectsCollinearLegs)
{
    auto stock = skill::createCuboidStock(20.0, 20.0, 20.0);

    skill::weldment_gusset_triangular::Input in;
    in.vertex_xyz           = { 0.0, 0.0, 0.0 };
    in.leg_a_xyz            = { 1.0, 0.0, 0.0 };
    in.leg_b_xyz            = { 2.0, 0.0, 0.0 };  // parallel to leg_a
    in.gusset_thickness_mm  = 5.0;
    in.gusset_leg_length_mm = 50.0;

    auto r = skill::weldment_gusset_triangular::validate(*stock, in);
    EXPECT_FALSE(r.passed);
}

// ─── 4. Signature pattern ─────────────────────────────────────────────────
TEST(SkillWeldmentGussetTriangular, SignatureCompound)
{
    auto stock = skill::createCuboidStock(20.0, 20.0, 20.0);

    skill::weldment_gusset_triangular::Input in;
    in.vertex_xyz           = { 200.0, 0.0, 0.0 };
    in.leg_a_xyz            = { 1.0, 0.0, 0.0 };
    in.leg_b_xyz            = { 0.0, 0.0, 1.0 };
    in.gusset_thickness_mm  = 6.0;
    in.gusset_leg_length_mm = 80.0;

    auto out = skill::weldment_gusset_triangular::apply(*stock, in);
    EXPECT_EQ(out.signature.skill_id,
              std::string("weldment_gusset_triangular"));
    EXPECT_TRUE(out.signature.pattern.value("is_compound", false));
    EXPECT_EQ(out.signature.pattern.value("profile_kind", std::string{}),
              std::string("triangle"));
    EXPECT_NEAR(out.signature.pattern.value("thickness_mm", 0.0), 6.0, 1e-9);
}

// ─── 5. Recognize replays history ─────────────────────────────────────────
TEST(SkillWeldmentGussetTriangular, RecognizeReplaysHistory)
{
    auto stock = skill::createCuboidStock(20.0, 20.0, 20.0);

    skill::weldment_gusset_triangular::Input in;
    in.vertex_xyz           = { 200.0, 0.0, 0.0 };
    in.leg_a_xyz            = { 1.0, 0.0, 0.0 };
    in.leg_b_xyz            = { 0.0, 1.0, 0.0 };
    in.gusset_thickness_mm  = 5.0;
    in.gusset_leg_length_mm = 60.0;

    auto out   = skill::weldment_gusset_triangular::apply(*stock, in);
    auto cands = skill::weldment_gusset_triangular::recognize(*out.workpiece);
    ASSERT_GE(cands.size(), 1u);
    EXPECT_EQ(cands.front().skill_id,
              std::string("weldment_gusset_triangular"));
    EXPECT_GT(cands.front().confidence, 0.3);
}
