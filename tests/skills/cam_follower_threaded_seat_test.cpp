// @lat: [[process/test-strategy#compound macro]]
//
// cam_follower_threaded_seat — COMPOUND macro: tapped blind hole +
// counterbore for lock nut + entry chamfer.  Tests:
//   1. apply produces valid shape with multiple sub-features.
//   2. validate rejects bad input (unknown M-size).
//   3. signature records compound kind, subfeature_count >= 2.
//   4. recognize metadata replay (confidence 1.0).
//   5. total_depth = chamfer + counterbore_depth + tap_depth (invariant).

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/cam_follower_threaded_seat.hpp"

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

// ─── 1. Apply produces valid shape with multiple sub-features ─────────────
TEST(SkillCamFollowerThreadedSeat, ApplyProducesValidShapeWithMultipleSubFeatures)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 30.0);

    skill::cam_follower_threaded_seat::Input in;
    in.entry_face       = skill::FaceByNormal{ gp_Dir(0, 0, 1), 5.0, "largest" };
    in.position_x_mm    = 25.0;
    in.position_y_mm    = 25.0;
    in.axis_dir         = gp_Dir(0, 0, -1);
    in.cam_follower_M   = "M10";
    in.tap_depth_mm     = 0.0;        // → derived to 1.5 × 10 = 15 mm

    auto out = skill::cam_follower_threaded_seat::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    const double vBefore = volumeOf(stock->shape());
    const double vAfter  = volumeOf(out.workpiece->shape());
    EXPECT_LT(vAfter, vBefore);
    EXPECT_GT(out.workpiece->faceCount(), stock->faceCount());

    EXPECT_EQ(out.signature.pattern.at("subfeature_count").get<int>(), 3);
}

// ─── 2. Validate rejects bad input ────────────────────────────────────────
TEST(SkillCamFollowerThreadedSeat, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 30.0);

    skill::cam_follower_threaded_seat::Input in;
    in.entry_face     = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm  = 25.0;
    in.position_y_mm  = 25.0;
    in.cam_follower_M = "M99";   // unknown
    in.tap_depth_mm   = 15.0;

    auto r = skill::cam_follower_threaded_seat::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-CF-TAP") { found = true; break; }
    EXPECT_TRUE(found);

    EXPECT_THROW(skill::cam_follower_threaded_seat::apply(*stock, in),
                 skill::SkillError);
}

// ─── 3. Signature records compound kind ───────────────────────────────────
TEST(SkillCamFollowerThreadedSeat, SignatureRecordsCompoundKind)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 30.0);

    skill::cam_follower_threaded_seat::Input in;
    in.entry_face     = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm  = 25.0;
    in.position_y_mm  = 25.0;
    in.cam_follower_M = "M8";
    in.tap_depth_mm   = 0.0;   // derived

    auto out = skill::cam_follower_threaded_seat::apply(*stock, in);

    EXPECT_EQ(out.signature.skill_id, std::string("cam_follower_threaded_seat"));
    EXPECT_EQ(out.signature.pattern.at("kind").get<std::string>(),
              std::string("cam_follower_threaded_seat"));
    EXPECT_TRUE(out.signature.pattern.at("is_compound").get<bool>());
    EXPECT_GE(out.signature.pattern.at("subfeature_count").get<int>(), 2);
    EXPECT_EQ(out.signature.pattern.at("cam_follower_M").get<std::string>(),
              std::string("M8"));
}

// ─── 4. Recognize metadata replay ─────────────────────────────────────────
TEST(SkillCamFollowerThreadedSeat, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 30.0);

    skill::cam_follower_threaded_seat::Input in;
    in.entry_face     = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm  = 25.0;
    in.position_y_mm  = 25.0;
    in.cam_follower_M = "M8";
    in.tap_depth_mm   = 0.0;

    auto out = skill::cam_follower_threaded_seat::apply(*stock, in);

    auto cands = skill::cam_follower_threaded_seat::recognize(*out.workpiece);
    ASSERT_GE(cands.size(), 1u);
    EXPECT_DOUBLE_EQ(cands[0].confidence, 1.0);
    EXPECT_EQ(cands[0].recovered_params.at("cam_follower_M").get<std::string>(),
              std::string("M8"));
}

// ─── 5. total_depth = chamfer + counterbore_depth + tap_depth (invariant) ─
TEST(SkillCamFollowerThreadedSeat, TotalDepthMatchesSubFeatureSum)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 30.0);

    skill::cam_follower_threaded_seat::Input in;
    in.entry_face     = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm  = 25.0;
    in.position_y_mm  = 25.0;
    in.cam_follower_M = "M10";
    in.tap_depth_mm   = 18.0;   // explicit (>= 1.5 × 10)

    auto out = skill::cam_follower_threaded_seat::apply(*stock, in);

    const double totalD = out.signature.pattern.at("total_depth_mm").get<double>();
    const double tapD   = out.signature.pattern.at("tap_depth_mm").get<double>();
    const double cboreD = out.signature.pattern.at("counterbore_depth_mm").get<double>();
    const double chamfS = out.signature.pattern.at("chamfer_size_mm").get<double>();
    EXPECT_NEAR(totalD, chamfS + cboreD + tapD, 1e-9);
}
