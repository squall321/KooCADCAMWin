// @lat: [[process/test-strategy#compound macro]]
//
// partial_pen_fillet_prep — single bevel + back radius.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/partial_pen_fillet_prep.hpp"

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
TEST(SkillPartialPenFilletPrep, ApplyProducesValidShapeWithMultipleSubFeatures)
{
    auto stock = skill::createCuboidStock(80.0, 30.0, 10.0);
    const int faces0 = stock->faceCount();

    skill::partial_pen_fillet_prep::Input in;
    in.entry_face       = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.edge_dir         = gp_Dir(1, 0, 0);
    in.plate_t          = 10.0;
    in.bevel_angle      = 45.0;
    in.land_mm          = 2.0;
    in.edge_center_x_mm = 40.0;
    in.edge_center_y_mm = 15.0;
    in.edge_length_mm   = 50.0;

    auto out = skill::partial_pen_fillet_prep::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    const double removed = volumeOf(stock->shape()) -
                           volumeOf(out.workpiece->shape());
    EXPECT_GT(removed, 0.0);          // some material gone
    EXPECT_GT(out.workpiece->faceCount(), faces0);
}

// ─── 2. Validate rejects bad input ───────────────────────────────────────
TEST(SkillPartialPenFilletPrep, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(80.0, 30.0, 10.0);

    // 2a. bevel_angle out of range
    skill::partial_pen_fillet_prep::Input bad1;
    bad1.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    bad1.edge_dir     = gp_Dir(1, 0, 0);
    bad1.plate_t      = 10.0;
    bad1.bevel_angle  = 10.0;       // < 25°
    bad1.land_mm      = 2.0;
    bad1.edge_length_mm = 30.0;
    auto r1 = skill::partial_pen_fillet_prep::validate(*stock, bad1);
    EXPECT_FALSE(r1.passed);

    // 2b. land too small
    skill::partial_pen_fillet_prep::Input bad2 = bad1;
    bad2.bevel_angle  = 45.0;
    bad2.land_mm      = 0.3;
    auto r2 = skill::partial_pen_fillet_prep::validate(*stock, bad2);
    EXPECT_FALSE(r2.passed);

    // 2c. plate too thin
    skill::partial_pen_fillet_prep::Input bad3 = bad1;
    bad3.bevel_angle = 45.0;
    bad3.plate_t     = 2.0;
    auto r3 = skill::partial_pen_fillet_prep::validate(*stock, bad3);
    EXPECT_FALSE(r3.passed);

    EXPECT_THROW(skill::partial_pen_fillet_prep::apply(*stock, bad1),
                 skill::SkillError);
}

// ─── 3. Signature records compound kind & subfeature_count ───────────────
TEST(SkillPartialPenFilletPrep, SignatureRecordsCompoundKind)
{
    auto stock = skill::createCuboidStock(80.0, 30.0, 10.0);
    skill::partial_pen_fillet_prep::Input in;
    in.entry_face       = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.edge_dir         = gp_Dir(1, 0, 0);
    in.plate_t          = 10.0;
    in.bevel_angle      = 45.0;
    in.land_mm          = 2.0;
    in.edge_center_x_mm = 40.0;
    in.edge_center_y_mm = 15.0;
    in.edge_length_mm   = 50.0;
    auto out = skill::partial_pen_fillet_prep::apply(*stock, in);

    const auto& pat = out.signature.pattern;
    EXPECT_EQ(pat.at("kind").get<std::string>(),
              std::string("partial_pen_fillet_prep"));
    EXPECT_TRUE(pat.at("is_compound").get<bool>());
    EXPECT_GE(pat.at("subfeature_count").get<int>(), 2);
    EXPECT_NEAR(pat.at("bevel_angle").get<double>(), 45.0, 1e-6);
    EXPECT_NEAR(pat.at("land_mm").get<double>(), 2.0, 1e-6);
    EXPECT_TRUE(pat.contains("bevel_depth_mm"));
    EXPECT_TRUE(pat.contains("back_radius_mm"));
}

// ─── 4. Recognize metadata replay ────────────────────────────────────────
TEST(SkillPartialPenFilletPrep, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(80.0, 30.0, 10.0);
    skill::partial_pen_fillet_prep::Input in;
    in.entry_face       = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.edge_dir         = gp_Dir(1, 0, 0);
    in.plate_t          = 10.0;
    in.bevel_angle      = 45.0;
    in.land_mm          = 2.0;
    in.edge_center_x_mm = 40.0;
    in.edge_center_y_mm = 15.0;
    in.edge_length_mm   = 50.0;
    auto out = skill::partial_pen_fillet_prep::apply(*stock, in);

    auto cands = skill::partial_pen_fillet_prep::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_DOUBLE_EQ(cands[0].confidence, 1.0);
    EXPECT_NEAR(cands[0].recovered_params.at("plate_t").get<double>(),
                10.0, 1e-6);
}

// ─── 5. Bevel-depth relation: bevel_depth = plate_t − land ──────────────
TEST(SkillPartialPenFilletPrep, BevelDepthRelation)
{
    auto stock = skill::createCuboidStock(80.0, 30.0, 12.0);
    skill::partial_pen_fillet_prep::Input in;
    in.entry_face       = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.edge_dir         = gp_Dir(1, 0, 0);
    in.plate_t          = 12.0;
    in.bevel_angle      = 45.0;
    in.land_mm          = 3.0;
    in.edge_center_x_mm = 40.0;
    in.edge_center_y_mm = 15.0;
    in.edge_length_mm   = 40.0;
    auto out = skill::partial_pen_fillet_prep::apply(*stock, in);

    const auto& pat = out.signature.pattern;
    EXPECT_NEAR(pat.at("bevel_depth_mm").get<double>(),
                in.plate_t - in.land_mm, 1e-6);    // 9 mm
    EXPECT_NEAR(pat.at("bevel_top_offset_mm").get<double>(),
                (in.plate_t - in.land_mm) * std::tan(M_PI_4), 1e-6);
}
