// @lat: [[process/test-strategy#skill round-trip]]
//
// cooling_channel — polyline of cylindrical channels through a mold.
//
// Cases:
//   1. ApplyRemovesMaterial : 3-waypoint polyline removes ~ π r² × totalL.
//   2. ValidateRejectsBadInput : single waypoint → DFM-COOL-N error.
//   3. SignatureRecordsKind + dia + waypoint_count + segment_count.
//   4. RecognizeMetadataReplay : recognize() replays waypoints from history.
//   5. SmallDiaRejected (specific) : dia < 4 mm → DFM-COOL-DIA error.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/cooling_channel.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using namespace koocadcam;

namespace {

double volumeOf(const TopoDS_Shape& s) {
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}

}  // namespace

// ─── 1. Apply removes material ────────────────────────────────────────────
TEST(SkillCoolingChannel, ApplyRemovesMaterial)
{
    auto stock = skill::createCuboidStock(80.0, 60.0, 30.0);
    const double stockVol = volumeOf(stock->shape());

    skill::cooling_channel::Input in;
    in.dia_mm = 6.0;
    // Z-stacked U: along +X, then +Y, then -X (each 40 mm).
    in.waypoints.push_back({ 10.0, 10.0, 15.0 });
    in.waypoints.push_back({ 50.0, 10.0, 15.0 });
    in.waypoints.push_back({ 50.0, 50.0, 15.0 });
    in.waypoints.push_back({ 10.0, 50.0, 15.0 });

    auto out = skill::cooling_channel::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    const double removed = stockVol - volumeOf(out.workpiece->shape());
    const double r = 3.0;
    const double approxIfNonOverlap = M_PI * r * r * 120.0;
    EXPECT_GT(removed, 0.0);
    // Cylinder segments share endpoints so there's slight double-coverage;
    // expect the removed volume to be close to but ≤ the no-overlap upper.
    EXPECT_LT(removed, approxIfNonOverlap * 1.1);
}

// ─── 2. Validate rejects bad input (single waypoint) ──────────────────────
TEST(SkillCoolingChannel, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(80.0, 60.0, 30.0);

    skill::cooling_channel::Input in;
    in.dia_mm = 6.0;
    in.waypoints.push_back({ 10.0, 10.0, 15.0 });    // only 1 waypoint

    auto r = skill::cooling_channel::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool sawCode = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-COOL-N") { sawCode = true; break; }
    EXPECT_TRUE(sawCode);

    EXPECT_THROW(skill::cooling_channel::apply(*stock, in), skill::SkillError);
}

// ─── 3. Signature records kind + key params ───────────────────────────────
TEST(SkillCoolingChannel, SignatureRecordsKind)
{
    auto stock = skill::createCuboidStock(80.0, 60.0, 30.0);

    skill::cooling_channel::Input in;
    in.dia_mm = 8.0;
    in.waypoints.push_back({ 10.0, 10.0, 15.0 });
    in.waypoints.push_back({ 50.0, 10.0, 15.0 });
    in.waypoints.push_back({ 50.0, 50.0, 15.0 });

    auto out = skill::cooling_channel::apply(*stock, in);
    EXPECT_EQ(out.signature.skill_id, std::string("cooling_channel"));
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("cooling_channel"));
    EXPECT_NEAR(out.signature.pattern["dia_mm"].get<double>(), 8.0, 1e-9);
    EXPECT_EQ(out.signature.pattern["waypoint_count"].get<int>(), 3);
    EXPECT_EQ(out.signature.pattern["segment_count"].get<int>(),  2);
}

// ─── 4. Recognize metadata replay ─────────────────────────────────────────
TEST(SkillCoolingChannel, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(80.0, 60.0, 30.0);

    skill::cooling_channel::Input in;
    in.dia_mm = 5.5;
    in.waypoints.push_back({ 12.0, 11.0, 15.0 });
    in.waypoints.push_back({ 48.0, 11.0, 15.0 });
    in.waypoints.push_back({ 48.0, 49.0, 15.0 });

    auto out = skill::cooling_channel::apply(*stock, in);
    auto cands = skill::cooling_channel::recognize(*out.workpiece);
    ASSERT_GE(cands.size(), 1u);

    const auto& c = cands.front();
    EXPECT_GT(c.confidence, 0.7);
    EXPECT_NEAR(c.recovered_params["dia_mm"].get<double>(), 5.5, 1e-9);
    ASSERT_EQ(c.recovered_params["waypoints"].size(), 3u);
    EXPECT_NEAR(c.recovered_params["waypoints"][0]["x_mm"].get<double>(),
                12.0, 1e-9);
}

// ─── 5. Small dia rejected (specific) ─────────────────────────────────────
TEST(SkillCoolingChannel, SmallDiaRejected)
{
    auto stock = skill::createCuboidStock(80.0, 60.0, 30.0);

    skill::cooling_channel::Input in;
    in.dia_mm = 2.0;     // < 4.0 → DFM-COOL-DIA
    in.waypoints.push_back({ 10.0, 10.0, 15.0 });
    in.waypoints.push_back({ 50.0, 10.0, 15.0 });

    auto r = skill::cooling_channel::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool sawCode = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-COOL-DIA") { sawCode = true; break; }
    EXPECT_TRUE(sawCode);
}
