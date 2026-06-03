// @lat: [[process/test-strategy#zif_connector_seat]]
//
// zif_connector_seat — phone ZIF connector pocket compound (5 tests).
//
// Cases:
//   1. ApplyRemovesVolumeInBounds — Δvol within bounds of sum-of-parts ±20%
//   2. ValidateRejectsZeroDimensions
//   3. ValidateRejectsNegativeClearance
//   4. SignatureCarriesPhoneFlag
//   5. RecognizeReplaysHistory

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/zif_connector_seat.hpp"

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

skill::zif_connector_seat::Input good(int faceId)
{
    skill::zif_connector_seat::Input in;
    in.face_id             = faceId;
    in.position_x_mm       = 20.0;
    in.position_y_mm       = 15.0;
    in.fpc_width_mm        = 6.0;
    in.connector_height_mm = 0.8;
    in.latch_clearance_mm  = 0.4;
    return in;
}

}  // namespace

// ─── 1. Δvol approx sum-of-parts ─────────────────────────────────────────
TEST(SkillZifConnectorSeat, ApplyRemovesVolumeInBounds)
{
    auto stock = skill::createCuboidStock(50.0, 40.0, 10.0);
    const int topId = findTopFace(*stock);
    ASSERT_GE(topId, 0);

    auto in = good(topId);
    const double v0 = volumeOf(stock->shape());
    auto out = skill::zif_connector_seat::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    const double v1 = volumeOf(out.workpiece->shape());
    const double actualRemoved = v0 - v1;

    constexpr double kConnLen = 8.0;
    const double mainLen = kConnLen;
    const double mainWid = in.fpc_width_mm + 0.5;
    const double mainDep = in.connector_height_mm;

    const double mainVol = mainLen * mainWid * mainDep;
    // Upper bound: ~2.5× the main pocket
    EXPECT_GT(actualRemoved, mainVol * 0.5);
    EXPECT_LT(actualRemoved, mainVol * 2.5);
}

// ─── 2. Validate rejects zero dim ────────────────────────────────────────
TEST(SkillZifConnectorSeat, ValidateRejectsZeroDimensions)
{
    auto stock = skill::createCuboidStock(50.0, 40.0, 10.0);
    const int topId = findTopFace(*stock);
    auto in = good(topId);
    in.fpc_width_mm = 0.0;
    auto r = skill::zif_connector_seat::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_THROW(skill::zif_connector_seat::apply(*stock, in),
                 skill::SkillError);
}

// ─── 3. Validate rejects negative clearance ──────────────────────────────
TEST(SkillZifConnectorSeat, ValidateRejectsNegativeClearance)
{
    auto stock = skill::createCuboidStock(50.0, 40.0, 10.0);
    const int topId = findTopFace(*stock);
    auto in = good(topId);
    in.latch_clearance_mm = -0.1;
    auto r = skill::zif_connector_seat::validate(*stock, in);
    EXPECT_FALSE(r.passed);
}

// ─── 4. Signature carries phone flag ─────────────────────────────────────
TEST(SkillZifConnectorSeat, SignatureCarriesPhoneFlag)
{
    auto stock = skill::createCuboidStock(50.0, 40.0, 10.0);
    const int topId = findTopFace(*stock);
    auto in = good(topId);
    auto out = skill::zif_connector_seat::apply(*stock, in);

    EXPECT_EQ(out.signature.skill_id, std::string("zif_connector_seat"));
    EXPECT_TRUE(out.signature.pattern["is_compound"].get<bool>());
    EXPECT_TRUE(out.signature.pattern["is_phone_feature"].get<bool>());
    EXPECT_GE(out.signature.pattern["subfeature_count"].get<int>(), 3);
}

// ─── 5. Recognize replays history ────────────────────────────────────────
TEST(SkillZifConnectorSeat, RecognizeReplaysHistory)
{
    auto stock = skill::createCuboidStock(50.0, 40.0, 10.0);
    const int topId = findTopFace(*stock);
    auto in = good(topId);
    auto out = skill::zif_connector_seat::apply(*stock, in);

    auto cands = skill::zif_connector_seat::recognize(*out.workpiece);
    ASSERT_GE(cands.size(), 1u);
    EXPECT_EQ(cands.front().skill_id, std::string("zif_connector_seat"));
    EXPECT_GE(cands.front().confidence, 0.9);
}
