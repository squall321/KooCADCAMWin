// @lat: [[process/test-strategy#skill round-trip]]
//
// reference_axis_two_pts — axis through 2 distinct points.
//
// Cases (5):
//   1. ApplyDoesNotChangeShape.
//   2. SignatureRecordsKindAndDatumClass.
//   3. AxisDirIsUnitAndLengthCorrect — direction round-trips to within 1e-6.
//   4. ValidateRejectsCoincidentPoints.
//   5. RecognizeMetadataReplay.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/reference_axis_two_pts.hpp"

#include <cmath>

using namespace koocadcam;

// ─── 1. Shape unchanged ───────────────────────────────────────────────────
TEST(SkillReferenceAxisTwoPts, ApplyDoesNotChangeShape)
{
    auto stock = skill::createCuboidStock(30.0, 30.0, 30.0);
    skill::reference_axis_two_pts::Input in;
    in.point_a = { 0.0, 0.0, 0.0 };
    in.point_b = { 0.0, 0.0, 10.0 };
    in.name    = "AX_Z";

    auto out = skill::reference_axis_two_pts::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    EXPECT_EQ(out.workpiece->faceCount(), stock->faceCount());
}

// ─── 2. Signature kind + datum_class ──────────────────────────────────────
TEST(SkillReferenceAxisTwoPts, SignatureRecordsKindAndDatumClass)
{
    auto stock = skill::createCuboidStock(30.0, 30.0, 30.0);
    skill::reference_axis_two_pts::Input in;
    in.point_a = { 1.0, 2.0, 3.0 };
    in.point_b = { 4.0, 6.0, 3.0 };
    in.name    = "AX_DIAG";

    auto out = skill::reference_axis_two_pts::apply(*stock, in);
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("reference_axis_two_pts"));
    EXPECT_EQ(out.signature.pattern["datum_class"].get<std::string>(),
              std::string("axis"));
    EXPECT_TRUE(out.signature.pattern["is_reference"].get<bool>());
    EXPECT_EQ(out.signature.pattern["named_id"].get<std::string>(),
              std::string("AX_DIAG"));
}

// ─── 3. axis_dir is unit, axis_length matches ─────────────────────────────
TEST(SkillReferenceAxisTwoPts, AxisDirIsUnitAndLengthCorrect)
{
    auto stock = skill::createCuboidStock(30.0, 30.0, 30.0);
    skill::reference_axis_two_pts::Input in;
    in.point_a = { 0.0, 0.0, 0.0 };
    in.point_b = { 3.0, 4.0, 0.0 };   // length 5
    in.name    = "AX_345";

    auto out = skill::reference_axis_two_pts::apply(*stock, in);
    auto dir = out.signature.pattern["axis_dir_xyz"];
    EXPECT_NEAR(dir[0].get<double>(), 0.6, 1e-6);
    EXPECT_NEAR(dir[1].get<double>(), 0.8, 1e-6);
    EXPECT_NEAR(dir[2].get<double>(), 0.0, 1e-6);
    EXPECT_NEAR(out.signature.pattern["axis_length"].get<double>(),
                5.0, 1e-6);

    auto org = out.signature.pattern["axis_origin_xyz"];
    EXPECT_NEAR(org[0].get<double>(), 0.0, 1e-6);
    EXPECT_NEAR(org[1].get<double>(), 0.0, 1e-6);
    EXPECT_NEAR(org[2].get<double>(), 0.0, 1e-6);
}

// ─── 4. DFM rejects coincident points ─────────────────────────────────────
TEST(SkillReferenceAxisTwoPts, ValidateRejectsCoincidentPoints)
{
    auto stock = skill::createCuboidStock(30.0, 30.0, 30.0);
    skill::reference_axis_two_pts::Input in;
    in.point_a = { 5.0, 5.0, 5.0 };
    in.point_b = { 5.0, 5.0, 5.0 };
    in.name    = "BAD";
    auto r = skill::reference_axis_two_pts::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_THROW(skill::reference_axis_two_pts::apply(*stock, in),
                 skill::SkillError);
}

// ─── 5. Recognize replay ──────────────────────────────────────────────────
TEST(SkillReferenceAxisTwoPts, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(30.0, 30.0, 30.0);
    skill::reference_axis_two_pts::Input in;
    in.point_a = { 0.0, 0.0, 0.0 };
    in.point_b = { 0.0, 0.0, 10.0 };
    in.name    = "AX_Z10";

    auto out   = skill::reference_axis_two_pts::apply(*stock, in);
    auto cands = skill::reference_axis_two_pts::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_DOUBLE_EQ(cands[0].confidence, 1.0);
    EXPECT_EQ(cands[0].recovered_params["name"].get<std::string>(),
              std::string("AX_Z10"));
}
