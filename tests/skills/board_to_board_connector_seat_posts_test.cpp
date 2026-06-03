// @lat: [[process/test-strategy#board_to_board_connector_seat]]
//
// board_to_board_connector_seat — Slice-13 mounting-post extension (5 tests).
//
// The existing skill's pocket behaviour is covered by
// board_to_board_connector_seat_test.cpp; this file exercises ONLY the new
// `mounting_post_count` + `post_dia_mm` fields added in slice 13.
//
// Cases:
//   1. ApplyWithTwoPostsAddsBackMaterial — vol grows vs. no-posts variant
//   2. ApplyWithFourPostsAddsMore        — vol(4) > vol(2)
//   3. ValidateRejectsBadPostCount       — must be 0/2/4
//   4. ValidateRejectsZeroPostDia        — when count > 0
//   5. SignatureCarriesMountingPostCount

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/board_to_board_connector_seat.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

using namespace koocadcam;

namespace {

double volumeOf(const TopoDS_Shape& s)
{
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}

int findTopFace(const skill::Workpiece& wp)
{
    int bestId = -1; double bestArea = 0.0;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFacePlanar(i)) continue;
        gp_Dir n; try { n = wp.faceNormal(i); } catch (...) { continue; }
        if (n.Z() < 0.9) continue;
        const double a = wp.faceArea(i);
        if (a > bestArea) { bestArea = a; bestId = i; }
    }
    return bestId;
}

skill::board_to_board_connector_seat::Input good(int faceId)
{
    skill::board_to_board_connector_seat::Input in;
    in.face_id                 = faceId;
    in.connector_position_x_mm = 30.0;
    in.connector_position_y_mm = 20.0;
    in.connector_length_mm     = 12.0;
    in.connector_width_mm      = 2.0;
    in.connector_height_mm     = 0.8;
    in.keep_out_margin_mm      = 0.3;
    return in;
}

}  // namespace

// ─── 1. 2-post variant adds material relative to 0-post ──────────────────
TEST(SkillBtbConnectorSeatPosts, ApplyWithTwoPostsAddsBackMaterial)
{
    auto stock0 = skill::createCuboidStock(80.0, 50.0, 8.0);
    auto stock2 = skill::createCuboidStock(80.0, 50.0, 8.0);
    const int t0 = findTopFace(*stock0);
    const int t2 = findTopFace(*stock2);

    auto in0 = good(t0);
    auto in2 = good(t2);
    in2.mounting_post_count = 2;
    in2.post_dia_mm         = 0.8;

    auto out0 = skill::board_to_board_connector_seat::apply(*stock0, in0);
    auto out2 = skill::board_to_board_connector_seat::apply(*stock2, in2);

    const double v0 = volumeOf(out0.workpiece->shape());
    const double v2 = volumeOf(out2.workpiece->shape());

    // 2 posts add material → v2 > v0 (same pocket removal).
    EXPECT_GT(v2, v0);
    EXPECT_EQ(out2.signature.pattern["mounting_post_count"].get<int>(), 2);
}

// ─── 2. 4 posts add even more material ───────────────────────────────────
TEST(SkillBtbConnectorSeatPosts, ApplyWithFourPostsAddsMore)
{
    auto stock2 = skill::createCuboidStock(80.0, 50.0, 8.0);
    auto stock4 = skill::createCuboidStock(80.0, 50.0, 8.0);
    const int t2 = findTopFace(*stock2);
    const int t4 = findTopFace(*stock4);

    auto in2 = good(t2);
    in2.mounting_post_count = 2;
    in2.post_dia_mm         = 0.8;
    auto in4 = good(t4);
    in4.mounting_post_count = 4;
    in4.post_dia_mm         = 0.8;

    auto out2 = skill::board_to_board_connector_seat::apply(*stock2, in2);
    auto out4 = skill::board_to_board_connector_seat::apply(*stock4, in4);
    EXPECT_GT(volumeOf(out4.workpiece->shape()),
              volumeOf(out2.workpiece->shape()));
    EXPECT_EQ(out4.signature.pattern["mounting_post_count"].get<int>(), 4);
}

// ─── 3. Validate rejects bad post count ──────────────────────────────────
TEST(SkillBtbConnectorSeatPosts, ValidateRejectsBadPostCount)
{
    auto stock = skill::createCuboidStock(80.0, 50.0, 8.0);
    const int topId = findTopFace(*stock);
    auto in = good(topId);
    in.mounting_post_count = 3;     // must be 0, 2 or 4
    in.post_dia_mm         = 0.8;
    auto r = skill::board_to_board_connector_seat::validate(*stock, in);
    EXPECT_FALSE(r.passed);
}

// ─── 4. Validate rejects zero post dia when count > 0 ────────────────────
TEST(SkillBtbConnectorSeatPosts, ValidateRejectsZeroPostDia)
{
    auto stock = skill::createCuboidStock(80.0, 50.0, 8.0);
    const int topId = findTopFace(*stock);
    auto in = good(topId);
    in.mounting_post_count = 2;
    in.post_dia_mm         = 0.0;
    auto r = skill::board_to_board_connector_seat::validate(*stock, in);
    EXPECT_FALSE(r.passed);
}

// ─── 5. Signature carries mounting post count ────────────────────────────
TEST(SkillBtbConnectorSeatPosts, SignatureCarriesMountingPostCount)
{
    auto stock = skill::createCuboidStock(80.0, 50.0, 8.0);
    const int topId = findTopFace(*stock);
    auto in = good(topId);
    in.mounting_post_count = 4;
    in.post_dia_mm         = 0.8;
    auto out = skill::board_to_board_connector_seat::apply(*stock, in);
    EXPECT_EQ(out.signature.pattern["phone_feature_type"].get<std::string>(),
              std::string("board_to_board_connector_seat"));
    EXPECT_EQ(out.signature.pattern["mounting_post_count"].get<int>(), 4);
    EXPECT_NEAR(out.signature.pattern["post_dia_mm"].get<double>(), 0.8, 1e-6);
    EXPECT_GT(out.signature.pattern["post_added_volume_mm3"].get<double>(), 0.0);
}
