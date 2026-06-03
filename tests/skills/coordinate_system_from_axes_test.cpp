// @lat: [[process/test-strategy#skill round-trip]]
//
// coordinate_system_from_axes — local CSYS from (origin, X-dir, Y-dir),
// with Z = unit(X x Y) computed.
//
// Cases (5):
//   1. ApplyDoesNotChangeShape.
//   2. SignatureRecordsKindAndCsysClass.
//   3. ZAxisIsRightHandedCrossOfXY  — for canonical X=+X, Y=+Y, Z=+Z.
//   4. ValidateRejectsBadOrthogonality (>5° from 90°).
//   5. RecognizeMetadataReplay.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/coordinate_system_from_axes.hpp"

#include <cmath>

using namespace koocadcam;

// ─── 1. Shape unchanged ───────────────────────────────────────────────────
TEST(SkillCoordinateSystemFromAxes, ApplyDoesNotChangeShape)
{
    auto stock = skill::createCuboidStock(20.0, 20.0, 20.0);
    skill::coordinate_system_from_axes::Input in;
    in.origin     = { 0.0, 0.0, 0.0 };
    in.x_axis_dir = { 1.0, 0.0, 0.0 };
    in.y_axis_dir = { 0.0, 1.0, 0.0 };
    in.name       = "CSYS_WORLD";

    auto out = skill::coordinate_system_from_axes::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    EXPECT_EQ(out.workpiece->faceCount(), stock->faceCount());
}

// ─── 2. Signature kind + datum_class ──────────────────────────────────────
TEST(SkillCoordinateSystemFromAxes, SignatureRecordsKindAndCsysClass)
{
    auto stock = skill::createCuboidStock(20.0, 20.0, 20.0);
    skill::coordinate_system_from_axes::Input in;
    in.origin     = { 5.0, 6.0, 7.0 };
    in.x_axis_dir = { 1.0, 0.0, 0.0 };
    in.y_axis_dir = { 0.0, 1.0, 0.0 };
    in.name       = "CSYS_AT_567";

    auto out = skill::coordinate_system_from_axes::apply(*stock, in);
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("coordinate_system_from_axes"));
    EXPECT_EQ(out.signature.pattern["datum_class"].get<std::string>(),
              std::string("coord_sys"));
    EXPECT_TRUE(out.signature.pattern["is_reference"].get<bool>());
    EXPECT_EQ(out.signature.pattern["named_id"].get<std::string>(),
              std::string("CSYS_AT_567"));
    auto org = out.signature.pattern["origin_xyz"];
    EXPECT_NEAR(org[0].get<double>(), 5.0, 1e-9);
    EXPECT_NEAR(org[1].get<double>(), 6.0, 1e-9);
    EXPECT_NEAR(org[2].get<double>(), 7.0, 1e-9);
}

// ─── 3. Z = X × Y, right-handed canonical case ────────────────────────────
TEST(SkillCoordinateSystemFromAxes, ZAxisIsRightHandedCrossOfXY)
{
    auto stock = skill::createCuboidStock(20.0, 20.0, 20.0);
    skill::coordinate_system_from_axes::Input in;
    in.origin     = { 0.0, 0.0, 0.0 };
    in.x_axis_dir = { 1.0, 0.0, 0.0 };
    in.y_axis_dir = { 0.0, 1.0, 0.0 };
    in.name       = "CSYS_WORLD";

    auto out = skill::coordinate_system_from_axes::apply(*stock, in);

    auto x = out.signature.pattern["x_axis_xyz"];
    auto y = out.signature.pattern["y_axis_xyz"];
    auto z = out.signature.pattern["z_axis_xyz"];
    EXPECT_NEAR(x[0].get<double>(), 1.0, 1e-9);
    EXPECT_NEAR(y[1].get<double>(), 1.0, 1e-9);
    EXPECT_NEAR(z[2].get<double>(), 1.0, 1e-9);
    EXPECT_NEAR(out.signature.pattern["angle_xy_deg"].get<double>(),
                90.0, 1e-6);
}

// ─── 4. DFM rejects non-orthogonal X & Y (>5°) ────────────────────────────
TEST(SkillCoordinateSystemFromAxes, ValidateRejectsBadOrthogonality)
{
    auto stock = skill::createCuboidStock(20.0, 20.0, 20.0);
    skill::coordinate_system_from_axes::Input in;
    in.origin     = { 0.0, 0.0, 0.0 };
    in.x_axis_dir = { 1.0, 0.0, 0.0 };
    // 45° from +X — far from 90°.
    in.y_axis_dir = { 1.0, 1.0, 0.0 };
    in.name       = "BAD";

    auto r = skill::coordinate_system_from_axes::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_THROW(skill::coordinate_system_from_axes::apply(*stock, in),
                 skill::SkillError);
}

// ─── 5. Recognize replay ──────────────────────────────────────────────────
TEST(SkillCoordinateSystemFromAxes, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(20.0, 20.0, 20.0);
    skill::coordinate_system_from_axes::Input in;
    in.origin     = { 1.0, 2.0, 3.0 };
    in.x_axis_dir = { 1.0, 0.0, 0.0 };
    in.y_axis_dir = { 0.0, 1.0, 0.0 };
    in.name       = "CSYS_123";

    auto out   = skill::coordinate_system_from_axes::apply(*stock, in);
    auto cands = skill::coordinate_system_from_axes::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_DOUBLE_EQ(cands[0].confidence, 1.0);
    EXPECT_EQ(cands[0].recovered_params["name"].get<std::string>(),
              std::string("CSYS_123"));
}
