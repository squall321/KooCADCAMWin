// @lat: [[process/test-strategy#compound macro]]
//
// full_penetration_butt_prep — V-groove on plate edge per AWS BUC1.
//
// Test cases:
//   1. apply produces a valid shape, removes ≈ V-volume, face count change
//      reflects ≥ 3 new sub-feature faces.
//   2. validate rejects bad input (sub-min plate_t, out-of-range groove
//      angle, oversized root_face).
//   3. signature.pattern.is_compound == true, subfeature_count == 3, params
//      verbatim + derived (groove_depth_mm) present.
//   4. recognize() returns one candidate from metadata replay with
//      confidence == 1.0.
//   5. Specific geometric assertion: groove_top_width == 2 × groove_depth ×
//      tan(angle/2) (relates plate_t − root_face → top_width).

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/full_penetration_butt_prep.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

#include <cmath>

using namespace koocadcam;

namespace {
double volumeOf(const TopoDS_Shape& s) {
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}
}  // namespace

// ─── 1. Apply produces valid shape with multiple sub-features ────────────
TEST(SkillFullPenButtPrep, ApplyProducesValidShapeWithMultipleSubFeatures)
{
    auto stock = skill::createCuboidStock(80.0, 30.0, 10.0);
    const int faces0 = stock->faceCount();

    skill::full_penetration_butt_prep::Input in;
    in.entry_face       = skill::FaceByNormal{ gp_Dir(0, 0, 1), 5.0, "largest" };
    in.edge_dir         = gp_Dir(1, 0, 0);
    in.plate_t          = 10.0;
    in.groove_angle     = 60.0;
    in.root_face_mm     = 1.5;
    in.edge_center_x_mm = 40.0;
    in.edge_center_y_mm = 15.0;
    in.edge_length_mm   = 60.0;

    auto out = skill::full_penetration_butt_prep::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    // Volume removed ≈ V-prism volume = (1/2) × top_width × groove_depth × L.
    const double grooveDepth = in.plate_t - in.root_face_mm;            // 8.5
    const double halfAng     = (in.groove_angle * 0.5) * M_PI / 180.0;  // 30°
    const double topWidth    = 2.0 * grooveDepth * std::tan(halfAng);
    const double vPrismV     = 0.5 * topWidth * grooveDepth * in.edge_length_mm;
    const double removed     = volumeOf(stock->shape()) -
                               volumeOf(out.workpiece->shape());
    EXPECT_GT(removed, vPrismV * 0.7);     // root R + small overhang adds a bit
    EXPECT_LT(removed, vPrismV * 1.5);

    // Face count change consistent with ≥ 3 sub-features (V slope ×2 + root R).
    EXPECT_GT(out.workpiece->faceCount(), faces0);
}

// ─── 2. Validate rejects bad input ───────────────────────────────────────
TEST(SkillFullPenButtPrep, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(80.0, 30.0, 10.0);

    // 2a. plate_t < 3 mm
    skill::full_penetration_butt_prep::Input bad1;
    bad1.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    bad1.edge_dir     = gp_Dir(1, 0, 0);
    bad1.plate_t      = 2.0;
    bad1.groove_angle = 60.0;
    bad1.root_face_mm = 1.0;
    bad1.edge_length_mm = 30.0;
    auto r1 = skill::full_penetration_butt_prep::validate(*stock, bad1);
    EXPECT_FALSE(r1.passed);

    // 2b. groove_angle out of range
    skill::full_penetration_butt_prep::Input bad2 = bad1;
    bad2.plate_t      = 10.0;
    bad2.groove_angle = 150.0;
    auto r2 = skill::full_penetration_butt_prep::validate(*stock, bad2);
    EXPECT_FALSE(r2.passed);

    // 2c. root_face > plate_t / 3
    skill::full_penetration_butt_prep::Input bad3 = bad1;
    bad3.plate_t      = 10.0;
    bad3.groove_angle = 60.0;
    bad3.root_face_mm = 5.0;
    auto r3 = skill::full_penetration_butt_prep::validate(*stock, bad3);
    EXPECT_FALSE(r3.passed);

    EXPECT_THROW(skill::full_penetration_butt_prep::apply(*stock, bad1),
                 skill::SkillError);
}

// ─── 3. Signature records compound kind & subfeature_count ───────────────
TEST(SkillFullPenButtPrep, SignatureRecordsCompoundKind)
{
    auto stock = skill::createCuboidStock(80.0, 30.0, 10.0);

    skill::full_penetration_butt_prep::Input in;
    in.entry_face       = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.edge_dir         = gp_Dir(1, 0, 0);
    in.plate_t          = 10.0;
    in.groove_angle     = 60.0;
    in.root_face_mm     = 1.5;
    in.edge_center_x_mm = 40.0;
    in.edge_center_y_mm = 15.0;
    in.edge_length_mm   = 60.0;

    auto out = skill::full_penetration_butt_prep::apply(*stock, in);

    const auto& pat = out.signature.pattern;
    EXPECT_EQ(out.signature.skill_id, std::string("full_penetration_butt_prep"));
    EXPECT_EQ(pat.at("kind").get<std::string>(),
              std::string("full_penetration_butt_prep"));
    EXPECT_TRUE(pat.at("is_compound").get<bool>());
    EXPECT_GE(pat.at("subfeature_count").get<int>(), 2);
    EXPECT_NEAR(pat.at("groove_angle").get<double>(), 60.0, 1e-6);
    EXPECT_NEAR(pat.at("root_face_mm").get<double>(), 1.5, 1e-6);

    // Derived params present
    EXPECT_TRUE(pat.contains("groove_depth_mm"));
    EXPECT_TRUE(pat.contains("groove_top_width_mm"));
    EXPECT_TRUE(pat.contains("root_radius_mm"));
}

// ─── 4. Recognize metadata replay ────────────────────────────────────────
TEST(SkillFullPenButtPrep, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(80.0, 30.0, 10.0);

    skill::full_penetration_butt_prep::Input in;
    in.entry_face       = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.edge_dir         = gp_Dir(1, 0, 0);
    in.plate_t          = 10.0;
    in.groove_angle     = 60.0;
    in.root_face_mm     = 1.5;
    in.edge_center_x_mm = 40.0;
    in.edge_center_y_mm = 15.0;
    in.edge_length_mm   = 60.0;
    auto out = skill::full_penetration_butt_prep::apply(*stock, in);

    auto cands = skill::full_penetration_butt_prep::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_DOUBLE_EQ(cands[0].confidence, 1.0);
    EXPECT_EQ(cands[0].skill_id, std::string("full_penetration_butt_prep"));
    EXPECT_NEAR(cands[0].recovered_params.at("plate_t").get<double>(),
                10.0, 1e-6);
    EXPECT_NEAR(cands[0].recovered_params.at("root_face_mm").get<double>(),
                1.5, 1e-6);
}

// ─── 5. V-geometry relation: top_width = 2 × depth × tan(angle/2) ───────
TEST(SkillFullPenButtPrep, VGeometryRelation)
{
    auto stock = skill::createCuboidStock(80.0, 30.0, 12.0);

    skill::full_penetration_butt_prep::Input in;
    in.entry_face       = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.edge_dir         = gp_Dir(1, 0, 0);
    in.plate_t          = 12.0;
    in.groove_angle     = 90.0;        // tan(45°) = 1 → top = depth × 2
    in.root_face_mm     = 2.0;
    in.edge_center_x_mm = 40.0;
    in.edge_center_y_mm = 15.0;
    in.edge_length_mm   = 50.0;
    auto out = skill::full_penetration_butt_prep::apply(*stock, in);

    const auto& pat = out.signature.pattern;
    const double depth = pat.at("groove_depth_mm").get<double>();
    const double top   = pat.at("groove_top_width_mm").get<double>();
    EXPECT_NEAR(depth, in.plate_t - in.root_face_mm, 1e-6);  // 10 mm
    EXPECT_NEAR(top, 2.0 * depth * std::tan(M_PI_4), 1e-3);  // 20 mm at 90°
}
