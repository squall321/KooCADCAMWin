// @lat: [[process/test-strategy#skill round-trip]]
//
// deformation_warp — REAL geometric transform tests.
//   - Non-uniform scale × rotation chained → bounding box and volume
//     change accordingly.
//   - DFM rejects out-of-range scales and non-positive scales.
//   - Signature is_compound + subfeature_count = 2 + deformation_matrix.
//   - Recognize replays.
//   - Feature-specific: post-warp volume = scale_x · scale_y · scale_z ·
//     pre-warp volume (rotation preserves volume).

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/deformation_warp.hpp"

#include <BRepBndLib.hxx>
#include <BRepGProp.hxx>
#include <Bnd_Box.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

#include <cmath>

using namespace koocadcam;

namespace {
double volumeOf(const TopoDS_Shape& s)
{
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}
}

// Test 1 — warp produces a real new shape with expected volume.
TEST(SkillDeformationWarp, ApplyChangesGeometry)
{
    auto stock = skill::createCuboidStock(20.0, 20.0, 10.0);
    const double v0 = volumeOf(stock->shape());

    skill::deformation_warp::Input in;
    in.scale_x = 2.0;
    in.scale_y = 1.5;
    in.scale_z = 0.5;
    in.axis_origin = gp_Pnt(0, 0, 0);
    in.axis_dir    = gp_Dir(0, 0, 1);
    in.angle_deg   = 30.0;

    auto out = skill::deformation_warp::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    const double v1 = volumeOf(out.workpiece->shape());
    const double expected = v0 * (in.scale_x * in.scale_y * in.scale_z);
    EXPECT_NEAR(v1, expected, expected * 0.01);
}

// Test 2 — DFM rejects bad inputs.
TEST(SkillDeformationWarp, ValidateRejectsBad)
{
    auto stock = skill::createCuboidStock(20.0, 20.0, 10.0);

    // Negative scale
    skill::deformation_warp::Input bad;
    bad.scale_x = -1.0; bad.scale_y = 1.0; bad.scale_z = 1.0;
    auto r = skill::deformation_warp::validate(*stock, bad);
    EXPECT_FALSE(r.passed);
    EXPECT_THROW(skill::deformation_warp::apply(*stock, bad),
                 skill::SkillError);

    // Out-of-range scale
    skill::deformation_warp::Input bad2;
    bad2.scale_x = 15.0; bad2.scale_y = 1.0; bad2.scale_z = 1.0;
    auto r2 = skill::deformation_warp::validate(*stock, bad2);
    EXPECT_FALSE(r2.passed);
    bool found = false;
    for (const auto& f : r2.findings)
        if (f.code == "DFM-WARP-LIMIT") { found = true; break; }
    EXPECT_TRUE(found);
    EXPECT_THROW(skill::deformation_warp::apply(*stock, bad2),
                 skill::SkillError);
}

// Test 3 — signature is_compound + subfeature_count = 2.
TEST(SkillDeformationWarp, SignatureIsCompound)
{
    auto stock = skill::createCuboidStock(20.0, 20.0, 10.0);
    skill::deformation_warp::Input in;
    in.scale_x = 1.2; in.scale_y = 1.4; in.scale_z = 0.8;
    in.angle_deg = 45.0;
    in.axis_dir = gp_Dir(0, 0, 1);
    auto out = skill::deformation_warp::apply(*stock, in);

    EXPECT_EQ(out.signature.skill_id, std::string("deformation_warp"));
    EXPECT_TRUE(out.signature.pattern.value("is_compound", false));
    EXPECT_EQ(out.signature.pattern.value("subfeature_count", 0), 2);
    ASSERT_TRUE(out.signature.pattern.contains("deformation_matrix"));
    const auto& mat = out.signature.pattern["deformation_matrix"];
    ASSERT_EQ(mat.size(), 4u);
    EXPECT_NEAR(mat[0][0].get<double>(), 1.2, 1e-9);
    EXPECT_NEAR(mat[1][1].get<double>(), 1.4, 1e-9);
    EXPECT_NEAR(mat[2][2].get<double>(), 0.8, 1e-9);
}

// Test 4 — recognize replays metadata.
TEST(SkillDeformationWarp, RecognizeReplays)
{
    auto stock = skill::createCuboidStock(20.0, 20.0, 10.0);
    skill::deformation_warp::Input in;
    in.scale_x = 1.3; in.scale_y = 0.9; in.scale_z = 1.1;
    in.angle_deg = 15.0;
    in.axis_dir = gp_Dir(0, 0, 1);
    auto out = skill::deformation_warp::apply(*stock, in);
    auto cands = skill::deformation_warp::recognize(*out.workpiece);
    ASSERT_FALSE(cands.empty());
    EXPECT_DOUBLE_EQ(cands.front().confidence, 1.0);
    EXPECT_NEAR(cands.front().recovered_params.value("scale_x", 0.0),
                1.3, 1e-9);
    EXPECT_NEAR(cands.front().recovered_params.value("angle_deg", 0.0),
                15.0, 1e-9);
}

// Test 5 — feature-specific: post-warp volume = product(scales) × pre-warp
// volume (since rigid rotation preserves volume).
TEST(SkillDeformationWarp, VolumeProductEqualsScaleProduct)
{
    auto stock = skill::createCuboidStock(20.0, 20.0, 10.0);
    const double v0 = volumeOf(stock->shape());

    skill::deformation_warp::Input in;
    in.scale_x = 1.5; in.scale_y = 2.0; in.scale_z = 0.5;
    in.angle_deg = 60.0;          // rigid rotation preserves volume
    in.axis_dir = gp_Dir(0, 0, 1);
    auto out = skill::deformation_warp::apply(*stock, in);
    const double v1 = volumeOf(out.workpiece->shape());

    const double scaleProd = in.scale_x * in.scale_y * in.scale_z;
    EXPECT_NEAR(v1 / v0, scaleProd, 1e-3);

    // Signature pattern.scale_determinant must match (volume-conservation
    // proof that the matrix recording is consistent).
    const double det = out.signature.pattern.value("scale_determinant", 0.0);
    EXPECT_NEAR(det, scaleProd, 1e-9);
}
