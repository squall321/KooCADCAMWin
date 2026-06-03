// @lat: [[process/test-strategy#antenna_keepout_zone]]
//
// antenna_keepout_zone — Slice-13 polygon-pocket extension (5 tests).
//
// The polyline-trough behaviour is covered by antenna_keepout_zone_test.cpp;
// this file exercises ONLY the polygon mode added in Slice-13.
//
// Cases:
//   1. ApplyPolygonRemovesAreaTimesDepth  — Δvol ≈ polygon_area × depth
//   2. ValidateRejectsZeroDepth
//   3. ValidateRejectsExcessiveTaper      — implied draft > 15°
//   4. SignatureCarriesPolygonMode
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

// Polygon area via shoelace formula.
double polygonArea(const std::vector<std::pair<double,double>>& pts)
{
    double area = 0.0;
    const size_t n = pts.size();
    for (size_t i = 0; i < n; ++i) {
        const auto& a = pts[i];
        const auto& b = pts[(i + 1) % n];
        area += a.first * b.second - b.first * a.second;
    }
    return std::abs(area) * 0.5;
}

}  // namespace

// ─── 1. Δvol ≈ polygon_area × depth ──────────────────────────────────────
TEST(SkillAntennaKeepoutZonePolygon, ApplyPolygonRemovesAreaTimesDepth)
{
    auto stock = skill::createCuboidStock(60.0, 40.0, 6.0);
    const int topId = findTopFace(*stock);
    ASSERT_GE(topId, 0);

    skill::antenna_keepout_zone::Input in;
    in.face_id = topId;
    in.polygon_xy_points = {
        { 10.0, 10.0 }, { 30.0, 10.0 },
        { 30.0, 25.0 }, { 10.0, 25.0 },
    };
    in.depth_mm = 1.0;

    const double v0 = volumeOf(stock->shape());
    auto out = skill::antenna_keepout_zone::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    const double v1 = volumeOf(out.workpiece->shape());
    const double removed = v0 - v1;

    const double expected = polygonArea(in.polygon_xy_points) * in.depth_mm;
    EXPECT_NEAR(removed, expected, expected * 0.20);
}

// ─── 2. Validate rejects zero depth in polygon mode ──────────────────────
TEST(SkillAntennaKeepoutZonePolygon, ValidateRejectsZeroDepth)
{
    auto stock = skill::createCuboidStock(60.0, 40.0, 6.0);
    const int topId = findTopFace(*stock);
    skill::antenna_keepout_zone::Input in;
    in.face_id = topId;
    in.polygon_xy_points = { {10,10}, {30,10}, {20,25} };
    in.depth_mm = 0.0;
    auto r = skill::antenna_keepout_zone::validate(*stock, in);
    EXPECT_FALSE(r.passed);
}

// ─── 3. Validate rejects taper that implies > 15° draft ──────────────────
TEST(SkillAntennaKeepoutZonePolygon, ValidateRejectsExcessiveTaper)
{
    auto stock = skill::createCuboidStock(60.0, 40.0, 6.0);
    const int topId = findTopFace(*stock);
    skill::antenna_keepout_zone::Input in;
    in.face_id = topId;
    in.polygon_xy_points = { {10,10}, {30,10}, {30,25}, {10,25} };
    in.depth_mm       = 1.0;
    in.edge_taper_mm  = 5.0;   // arctan(5) ≈ 79° >> 15°
    auto r = skill::antenna_keepout_zone::validate(*stock, in);
    EXPECT_FALSE(r.passed);
}

// ─── 4. Signature carries polygon-mode flags ─────────────────────────────
TEST(SkillAntennaKeepoutZonePolygon, SignatureCarriesPolygonMode)
{
    auto stock = skill::createCuboidStock(60.0, 40.0, 6.0);
    const int topId = findTopFace(*stock);
    skill::antenna_keepout_zone::Input in;
    in.face_id = topId;
    in.polygon_xy_points = {
        { 10.0, 10.0 }, { 30.0, 10.0 },
        { 30.0, 25.0 }, { 10.0, 25.0 },
    };
    in.depth_mm      = 1.0;
    in.edge_taper_mm = 0.05;
    auto out = skill::antenna_keepout_zone::apply(*stock, in);

    EXPECT_TRUE(out.signature.pattern["is_compound"].get<bool>());
    EXPECT_TRUE(out.signature.pattern["is_phone_feature"].get<bool>());
    EXPECT_EQ(out.signature.pattern["mode"].get<std::string>(),
              std::string("polygon"));
    EXPECT_EQ(out.signature.pattern["phone_feature_type"].get<std::string>(),
              std::string("antenna_keepout"));
    EXPECT_EQ(out.signature.pattern["polygon_point_count"].get<int>(), 4);
    EXPECT_GT(out.signature.pattern["derived_volume_removed_mm3"].get<double>(),
              0.0);
}

// ─── 5. Recognize replays history ────────────────────────────────────────
TEST(SkillAntennaKeepoutZonePolygon, RecognizeReplaysHistory)
{
    auto stock = skill::createCuboidStock(60.0, 40.0, 6.0);
    const int topId = findTopFace(*stock);
    skill::antenna_keepout_zone::Input in;
    in.face_id = topId;
    in.polygon_xy_points = { {10,10}, {30,10}, {30,25}, {10,25} };
    in.depth_mm = 1.0;
    auto out = skill::antenna_keepout_zone::apply(*stock, in);

    auto cands = skill::antenna_keepout_zone::recognize(*out.workpiece);
    ASSERT_GE(cands.size(), 1u);
    EXPECT_EQ(cands.front().skill_id, std::string("antenna_keepout_zone"));
    EXPECT_GE(cands.front().confidence, 0.9);
}
