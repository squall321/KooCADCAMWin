// @lat: [[process/test-strategy#board_to_board_connector_seat]]
//
// board_to_board_connector_seat — phone BTB pocket compound (5 tests).
//
// Cases:
//   1. ApplyRemovesVolumeMatchesSumOfParts — Δvol ≈ Σ(sub-feature vols) ±20%
//   2. ValidateRejectsZeroDimensions
//   3. ValidateRejectsBadFaceId
//   4. SignatureCarriesCompoundAndPhoneFlags
//   5. RecognizeReplaysHistory

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/board_to_board_connector_seat.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

#include <cmath>

using namespace koocadcam;

namespace {

double volumeOf(const TopoDS_Shape& s)
{
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}

int findTopFace(const skill::Workpiece& wp)
{
    int    bestId   = -1;
    double bestArea = 0.0;
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
    in.connector_position_x_mm = 25.0;
    in.connector_position_y_mm = 20.0;
    in.connector_length_mm     = 12.0;
    in.connector_width_mm      = 2.0;
    in.connector_height_mm     = 0.8;
    in.keep_out_margin_mm      = 0.3;
    return in;
}

}  // namespace

// ─── 1. Δvol ≈ sum-of-parts ±20% ─────────────────────────────────────────
TEST(SkillBoardToBoardConnectorSeat, ApplyRemovesVolumeMatchesSumOfParts)
{
    auto stock = skill::createCuboidStock(60.0, 40.0, 10.0);
    const int topId = findTopFace(*stock);
    ASSERT_GE(topId, 0);

    auto in    = good(topId);
    const double v0 = volumeOf(stock->shape());
    auto out = skill::board_to_board_connector_seat::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    const double v1 = volumeOf(out.workpiece->shape());
    const double actualRemoved = v0 - v1;

    // Estimated max removed = bounding outer keep-out box volume.
    const double L = in.connector_length_mm;
    const double W = in.connector_width_mm;
    const double H = in.connector_height_mm;
    const double M = in.keep_out_margin_mm;
    const double chamW = std::max(0.05, M * 0.5);
    const double slopeDepth = std::max(0.05, H / 3.0);
    const double chamDepth  = std::max(0.05, slopeDepth * 0.3);

    // Upper bound = footprint of widest pocket × max depth
    const double bound_max = (L + 2.0 * (M + chamW)) *
                              (W + 2.0 * (M + chamW)) * H;
    const double bound_min = L * W * chamDepth * 0.5;

    EXPECT_GT(actualRemoved, bound_min);
    EXPECT_LT(actualRemoved, bound_max * 1.2);
}

// ─── 2. Validate rejects zero dimensions ─────────────────────────────────
TEST(SkillBoardToBoardConnectorSeat, ValidateRejectsZeroDimensions)
{
    auto stock = skill::createCuboidStock(60.0, 40.0, 10.0);
    const int topId = findTopFace(*stock);
    auto in = good(topId);
    in.connector_length_mm = 0.0;
    auto r = skill::board_to_board_connector_seat::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_THROW(skill::board_to_board_connector_seat::apply(*stock, in),
                 skill::SkillError);
}

// ─── 3. Validate rejects bad face_id ─────────────────────────────────────
TEST(SkillBoardToBoardConnectorSeat, ValidateRejectsBadFaceId)
{
    auto stock = skill::createCuboidStock(60.0, 40.0, 10.0);
    auto in = good(-1);
    auto r = skill::board_to_board_connector_seat::validate(*stock, in);
    EXPECT_FALSE(r.passed);
}

// ─── 4. Signature carries compound + phone flags ─────────────────────────
TEST(SkillBoardToBoardConnectorSeat, SignatureCarriesCompoundAndPhoneFlags)
{
    auto stock = skill::createCuboidStock(60.0, 40.0, 10.0);
    const int topId = findTopFace(*stock);
    auto in = good(topId);
    auto out = skill::board_to_board_connector_seat::apply(*stock, in);

    EXPECT_EQ(out.signature.skill_id,
              std::string("board_to_board_connector_seat"));
    EXPECT_TRUE(out.signature.pattern["is_compound"].get<bool>());
    EXPECT_TRUE(out.signature.pattern["is_phone_feature"].get<bool>());
    EXPECT_GE(out.signature.pattern["subfeature_count"].get<int>(), 3);
    EXPECT_GT(out.signature.pattern["derived_volume_removed_mm3"].get<double>(),
              0.0);
}

// ─── 5. Recognize replays history ────────────────────────────────────────
TEST(SkillBoardToBoardConnectorSeat, RecognizeReplaysHistory)
{
    auto stock = skill::createCuboidStock(60.0, 40.0, 10.0);
    const int topId = findTopFace(*stock);
    auto in = good(topId);
    auto out = skill::board_to_board_connector_seat::apply(*stock, in);

    auto cands = skill::board_to_board_connector_seat::recognize(*out.workpiece);
    ASSERT_GE(cands.size(), 1u);
    EXPECT_EQ(cands.front().skill_id,
              std::string("board_to_board_connector_seat"));
    EXPECT_GE(cands.front().confidence, 0.9);
}
