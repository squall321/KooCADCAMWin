// @lat: [[process/test-strategy#antenna_keepout_zone]]
//
// antenna_keepout_zone — polyline metal-free trough cut (5 tests).
//
// Cases:
//   1. ApplyRemovesVolumeMatchesPolylineSum — Δvol ≈ Σ(seg L × W × D) ±20%
//   2. ValidateRejectsShortPolyline (< 2 pts)
//   3. ValidateRejectsZeroWidth
//   4. SignatureCarriesSegmentCount
//   5. RecognizeReplaysHistory

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/antenna_keepout_zone.hpp"

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

}  // namespace

// ─── 1. Δvol ≈ sum of segment volumes ±20% ────────────────────────────────
TEST(SkillAntennaKeepoutZone, ApplyRemovesVolumeMatchesPolylineSum)
{
    auto stock = skill::createCuboidStock(60.0, 40.0, 10.0);
    const int topId = findTopFace(*stock);
    ASSERT_GE(topId, 0);

    skill::antenna_keepout_zone::Input in;
    in.face_id           = topId;
    in.polyline          = { { 10.0, 10.0 }, { 50.0, 10.0 }, { 50.0, 30.0 } };
    in.keep_out_width_mm = 1.5;
    in.keep_out_depth_mm = 0.6;

    const double v0 = volumeOf(stock->shape());
    auto out = skill::antenna_keepout_zone::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    const double v1 = volumeOf(out.workpiece->shape());
    const double actualRemoved = v0 - v1;

    // Two segments: 40 mm horizontal + 20 mm vertical = 60 mm total length
    // Estimated removal = 60 × 1.5 × 0.6 = 54 mm³ (corner overlap adds a bit)
    const double expected = 60.0 * 1.5 * 0.6;
    EXPECT_NEAR(actualRemoved, expected, expected * 0.20)
        << "actual=" << actualRemoved << " expected≈" << expected;
}

// ─── 2. Validate rejects short polyline ───────────────────────────────────
TEST(SkillAntennaKeepoutZone, ValidateRejectsShortPolyline)
{
    auto stock = skill::createCuboidStock(60.0, 40.0, 10.0);
    const int topId = findTopFace(*stock);

    skill::antenna_keepout_zone::Input in;
    in.face_id           = topId;
    in.polyline          = { { 10.0, 10.0 } };       // only 1 point
    in.keep_out_width_mm = 1.5;
    in.keep_out_depth_mm = 0.6;

    auto r = skill::antenna_keepout_zone::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_THROW(skill::antenna_keepout_zone::apply(*stock, in),
                 skill::SkillError);
}

// ─── 3. Validate rejects zero width ──────────────────────────────────────
TEST(SkillAntennaKeepoutZone, ValidateRejectsZeroWidth)
{
    auto stock = skill::createCuboidStock(60.0, 40.0, 10.0);
    const int topId = findTopFace(*stock);
    skill::antenna_keepout_zone::Input in;
    in.face_id           = topId;
    in.polyline          = { { 10.0, 10.0 }, { 50.0, 10.0 } };
    in.keep_out_width_mm = 0.0;
    in.keep_out_depth_mm = 0.6;
    auto r = skill::antenna_keepout_zone::validate(*stock, in);
    EXPECT_FALSE(r.passed);
}

// ─── 4. Signature carries segment count ──────────────────────────────────
TEST(SkillAntennaKeepoutZone, SignatureCarriesSegmentCount)
{
    auto stock = skill::createCuboidStock(60.0, 40.0, 10.0);
    const int topId = findTopFace(*stock);
    skill::antenna_keepout_zone::Input in;
    in.face_id           = topId;
    in.polyline          = { { 10.0, 10.0 }, { 30.0, 10.0 }, { 50.0, 20.0 } };
    in.keep_out_width_mm = 1.5;
    in.keep_out_depth_mm = 0.6;

    auto out = skill::antenna_keepout_zone::apply(*stock, in);
    EXPECT_EQ(out.signature.skill_id, std::string("antenna_keepout_zone"));
    EXPECT_TRUE(out.signature.pattern["is_compound"].get<bool>());
    EXPECT_TRUE(out.signature.pattern["is_phone_feature"].get<bool>());
    EXPECT_EQ(out.signature.pattern["segment_count"].get<int>(), 2);
}

// ─── 5. Recognize replays history ────────────────────────────────────────
TEST(SkillAntennaKeepoutZone, RecognizeReplaysHistory)
{
    auto stock = skill::createCuboidStock(60.0, 40.0, 10.0);
    const int topId = findTopFace(*stock);
    skill::antenna_keepout_zone::Input in;
    in.face_id           = topId;
    in.polyline          = { { 10.0, 10.0 }, { 50.0, 10.0 } };
    in.keep_out_width_mm = 1.5;
    in.keep_out_depth_mm = 0.6;
    auto out = skill::antenna_keepout_zone::apply(*stock, in);

    auto cands = skill::antenna_keepout_zone::recognize(*out.workpiece);
    ASSERT_GE(cands.size(), 1u);
    EXPECT_EQ(cands.front().skill_id, std::string("antenna_keepout_zone"));
    EXPECT_GE(cands.front().confidence, 0.9);
}
