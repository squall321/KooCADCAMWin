// @lat: [[process/test-strategy#skill round-trip]]
//
// reference_plane_at_3pts — plane through 3 non-collinear points.
//
// Cases (5):
//   1. ApplyDoesNotChangeShape.
//   2. SignatureRecordsKindAndPoints — A/B/C round-trip exactly through
//      pattern.point_*_xyz (within 1e-6).
//   3. PlaneNormalIsCrossProductUnit — n == unit((B-A) x (C-A)).
//   4. ValidateRejectsCollinearPoints.
//   5. RecognizeMetadataReplay.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/reference_plane_at_3pts.hpp"

#include <cmath>

using namespace koocadcam;

// ─── 1. Shape unchanged ───────────────────────────────────────────────────
TEST(SkillReferencePlaneAt3Pts, ApplyDoesNotChangeShape)
{
    auto stock = skill::createCuboidStock(20.0, 20.0, 20.0);
    skill::reference_plane_at_3pts::Input in;
    in.point_a = { 0.0, 0.0, 0.0 };
    in.point_b = { 10.0, 0.0, 0.0 };
    in.point_c = { 0.0, 10.0, 0.0 };
    in.name    = "PL_XY";

    auto out = skill::reference_plane_at_3pts::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    EXPECT_EQ(out.workpiece->faceCount(), stock->faceCount());
}

// ─── 2. Signature points round-trip ───────────────────────────────────────
TEST(SkillReferencePlaneAt3Pts, SignatureRecordsKindAndPoints)
{
    auto stock = skill::createCuboidStock(20.0, 20.0, 20.0);
    skill::reference_plane_at_3pts::Input in;
    in.point_a = { 1.25, 2.50, -3.75 };
    in.point_b = { 7.50, -2.00, 4.00 };
    in.point_c = { -1.0, 5.50, 6.25 };
    in.name    = "PL_ABC";

    auto out = skill::reference_plane_at_3pts::apply(*stock, in);
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("reference_plane_at_3pts"));
    EXPECT_EQ(out.signature.pattern["datum_class"].get<std::string>(),
              std::string("plane"));
    EXPECT_TRUE(out.signature.pattern["is_reference"].get<bool>());

    auto a = out.signature.pattern["point_a_xyz"];
    EXPECT_NEAR(a[0].get<double>(), 1.25,  1e-6);
    EXPECT_NEAR(a[1].get<double>(), 2.50,  1e-6);
    EXPECT_NEAR(a[2].get<double>(), -3.75, 1e-6);
    auto b = out.signature.pattern["point_b_xyz"];
    EXPECT_NEAR(b[0].get<double>(), 7.50,  1e-6);
    EXPECT_NEAR(b[1].get<double>(), -2.00, 1e-6);
    EXPECT_NEAR(b[2].get<double>(), 4.00,  1e-6);
    auto c = out.signature.pattern["point_c_xyz"];
    EXPECT_NEAR(c[0].get<double>(), -1.0,  1e-6);
    EXPECT_NEAR(c[1].get<double>(), 5.50,  1e-6);
    EXPECT_NEAR(c[2].get<double>(), 6.25,  1e-6);
}

// ─── 3. Plane normal = unit cross product ─────────────────────────────────
TEST(SkillReferencePlaneAt3Pts, PlaneNormalIsCrossProductUnit)
{
    auto stock = skill::createCuboidStock(20.0, 20.0, 20.0);
    // XY-plane 3 points → normal must be ±Z.
    skill::reference_plane_at_3pts::Input in;
    in.point_a = { 0.0, 0.0, 5.0 };
    in.point_b = { 4.0, 0.0, 5.0 };
    in.point_c = { 0.0, 3.0, 5.0 };   // |AB x AC| = 12
    in.name    = "Z5_PLANE";

    auto out = skill::reference_plane_at_3pts::apply(*stock, in);
    auto n = out.signature.pattern["plane_normal_xyz"];
    EXPECT_NEAR(std::abs(n[2].get<double>()), 1.0, 1e-9);
    EXPECT_NEAR(n[0].get<double>(), 0.0, 1e-9);
    EXPECT_NEAR(n[1].get<double>(), 0.0, 1e-9);
    EXPECT_NEAR(out.signature.pattern["cross_magnitude"].get<double>(),
                12.0, 1e-6);

    // Centroid sanity.
    auto org = out.signature.pattern["plane_origin_xyz"];
    EXPECT_NEAR(org[0].get<double>(), (0.0 + 4.0 + 0.0) / 3.0, 1e-9);
    EXPECT_NEAR(org[1].get<double>(), (0.0 + 0.0 + 3.0) / 3.0, 1e-9);
    EXPECT_NEAR(org[2].get<double>(), 5.0, 1e-9);
}

// ─── 4. DFM rejects collinear points ──────────────────────────────────────
TEST(SkillReferencePlaneAt3Pts, ValidateRejectsCollinearPoints)
{
    auto stock = skill::createCuboidStock(20.0, 20.0, 20.0);
    // All three on the X-axis.
    skill::reference_plane_at_3pts::Input in;
    in.point_a = { 0.0, 0.0, 0.0 };
    in.point_b = { 1.0, 0.0, 0.0 };
    in.point_c = { 5.0, 0.0, 0.0 };
    in.name    = "BAD";

    auto r = skill::reference_plane_at_3pts::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_THROW(skill::reference_plane_at_3pts::apply(*stock, in),
                 skill::SkillError);
}

// ─── 5. Recognize replay ──────────────────────────────────────────────────
TEST(SkillReferencePlaneAt3Pts, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(20.0, 20.0, 20.0);
    skill::reference_plane_at_3pts::Input in;
    in.point_a = { 0.0, 0.0, 0.0 };
    in.point_b = { 1.0, 0.0, 0.0 };
    in.point_c = { 0.0, 1.0, 0.0 };
    in.name    = "PL_XY";

    auto out   = skill::reference_plane_at_3pts::apply(*stock, in);
    auto cands = skill::reference_plane_at_3pts::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_DOUBLE_EQ(cands[0].confidence, 1.0);
    EXPECT_EQ(cands[0].recovered_params["name"].get<std::string>(),
              std::string("PL_XY"));
}
