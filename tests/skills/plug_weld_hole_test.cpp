// @lat: [[process/test-strategy#compound macro]]
//
// plug_weld_hole — through hole + countersink on one side.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/plug_weld_hole.hpp"

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
TEST(SkillPlugWeldHole, ApplyProducesValidShapeWithMultipleSubFeatures)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 8.0);
    const int faces0 = stock->faceCount();

    skill::plug_weld_hole::Input in;
    in.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1), 5.0, "largest" };
    in.position_x_mm = 20.0;
    in.position_y_mm = 20.0;
    in.axis_dir      = gp_Dir(0, 0, -1);
    in.hole_dia      = 6.0;
    in.cs_top_dia    = 9.0;
    in.cs_angle_deg  = 90.0;

    auto out = skill::plug_weld_hole::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    // Volume removed ≈ through cyl volume + cone frustum volume (above top).
    const double R = in.hole_dia / 2.0;
    const double cyV = M_PI * R * R * 8.0;
    const double removed = volumeOf(stock->shape()) -
                           volumeOf(out.workpiece->shape());
    EXPECT_GT(removed, cyV * 0.9);
    EXPECT_LT(removed, cyV * 2.5);   // cone adds some more

    // Face count: through hole adds cyl + 2 circular edges, cone adds 1 cone
    // face — total +2 cylinder/cone faces, ≥ 1 of original entry face lost
    // and split.  Just check it changed.
    EXPECT_NE(out.workpiece->faceCount(), faces0);
}

// ─── 2. Validate rejects bad input ───────────────────────────────────────
TEST(SkillPlugWeldHole, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 8.0);

    // 2a. hole_dia too small
    skill::plug_weld_hole::Input bad1;
    bad1.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    bad1.position_x_mm = 20.0;
    bad1.position_y_mm = 20.0;
    bad1.hole_dia      = 2.0;     // < 3 mm
    bad1.cs_top_dia    = 5.0;
    bad1.cs_angle_deg  = 90.0;
    auto r1 = skill::plug_weld_hole::validate(*stock, bad1);
    EXPECT_FALSE(r1.passed);

    // 2b. hole_dia too large
    skill::plug_weld_hole::Input bad2 = bad1;
    bad2.hole_dia      = 15.0;
    bad2.cs_top_dia    = 20.0;
    auto r2 = skill::plug_weld_hole::validate(*stock, bad2);
    EXPECT_FALSE(r2.passed);

    EXPECT_THROW(skill::plug_weld_hole::apply(*stock, bad1),
                 skill::SkillError);
}

// ─── 3. Signature records compound kind & subfeature_count == 2 ─────────
TEST(SkillPlugWeldHole, SignatureRecordsCompoundKind)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 8.0);
    skill::plug_weld_hole::Input in;
    in.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 20.0;
    in.position_y_mm = 20.0;
    in.hole_dia      = 6.0;
    in.cs_top_dia    = 9.0;
    in.cs_angle_deg  = 90.0;
    auto out = skill::plug_weld_hole::apply(*stock, in);

    const auto& pat = out.signature.pattern;
    EXPECT_EQ(pat.at("kind").get<std::string>(),
              std::string("plug_weld_hole"));
    EXPECT_TRUE(pat.at("is_compound").get<bool>());
    EXPECT_EQ(pat.at("subfeature_count").get<int>(), 2);
    EXPECT_NEAR(pat.at("hole_dia").get<double>(), 6.0, 1e-6);
    EXPECT_NEAR(pat.at("cs_top_dia").get<double>(), 9.0, 1e-6);
    EXPECT_TRUE(pat.contains("cs_depth_mm"));
}

// ─── 4. Recognize metadata replay ────────────────────────────────────────
TEST(SkillPlugWeldHole, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 8.0);
    skill::plug_weld_hole::Input in;
    in.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 20.0;
    in.position_y_mm = 20.0;
    in.hole_dia      = 6.0;
    in.cs_top_dia    = 9.0;
    in.cs_angle_deg  = 90.0;
    auto out = skill::plug_weld_hole::apply(*stock, in);

    auto cands = skill::plug_weld_hole::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_DOUBLE_EQ(cands[0].confidence, 1.0);
    EXPECT_NEAR(cands[0].recovered_params.at("hole_dia").get<double>(),
                6.0, 1e-6);
}

// ─── 5. Countersink depth relation: cs_depth = (cs_top − hole)/2/tan(α/2) ─
TEST(SkillPlugWeldHole, CountersinkDepthRelation)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 8.0);
    skill::plug_weld_hole::Input in;
    in.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 20.0;
    in.position_y_mm = 20.0;
    in.hole_dia      = 6.0;
    in.cs_top_dia    = 10.0;
    in.cs_angle_deg  = 90.0;   // tan(45°) = 1 → cs_depth = 2 mm
    auto out = skill::plug_weld_hole::apply(*stock, in);

    const auto& pat = out.signature.pattern;
    const double half = (in.cs_angle_deg * 0.5) * M_PI / 180.0;
    const double expected = (in.cs_top_dia - in.hole_dia) * 0.5 / std::tan(half);
    EXPECT_NEAR(pat.at("cs_depth_mm").get<double>(), expected, 1e-3);
}
