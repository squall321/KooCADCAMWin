// @lat: [[process/test-strategy#skill round-trip]]
//
// lidar_scan — INSPECTION skill: pulsed-laser point-cloud sweep.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/lidar_scan.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

#include <cmath>

using namespace koocadcam;

namespace {

double volumeOf(const TopoDS_Shape& s)
{
    GProp_GProps p;
    BRepGProp::VolumeProperties(s, p);
    return p.Mass();
}

}  // namespace

// ── 1. Apply clones geometry unchanged ───────────────────────────────────
TEST(SkillLidarScan, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);
    const double volBefore = volumeOf(stock->shape());

    skill::lidar_scan::Input in;
    in.range_m        = 30.0;
    in.points_per_sec = 5.0e5;
    in.ranging_class  = "ToF";

    auto out = skill::lidar_scan::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, volBefore * 1e-6);
    EXPECT_TRUE(out.signature.pattern["is_inspection_only"].get<bool>());
    EXPECT_FALSE(out.signature.pattern["geometric_change"].get<bool>());
}

// ── 2. ValidateRejectsBadInput: bad range / rate ─────────────────────────
TEST(SkillLidarScan, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::lidar_scan::Input badRange;
    badRange.range_m        = 0.0;
    badRange.points_per_sec = 5.0e5;
    badRange.ranging_class  = "ToF";
    auto r1 = skill::lidar_scan::validate(*stock, badRange);
    EXPECT_FALSE(r1.passed);
    EXPECT_THROW(skill::lidar_scan::apply(*stock, badRange),
                 skill::SkillError);

    skill::lidar_scan::Input badRate;
    badRate.range_m        = 30.0;
    badRate.points_per_sec = 0.0;
    badRate.ranging_class  = "ToF";
    auto r2 = skill::lidar_scan::validate(*stock, badRate);
    EXPECT_FALSE(r2.passed);

    skill::lidar_scan::Input tooFar;
    tooFar.range_m        = 1500.0;        // > 1000 m
    tooFar.points_per_sec = 5.0e5;
    tooFar.ranging_class  = "ToF";
    auto r3 = skill::lidar_scan::validate(*stock, tooFar);
    EXPECT_FALSE(r3.passed);
}

// ── 3. Signature records kind + tooling metadata ─────────────────────────
TEST(SkillLidarScan, SignatureRecordsKindAndTooling)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::lidar_scan::Input in;
    in.range_m        = 50.0;
    in.points_per_sec = 1.0e6;
    in.ranging_class  = "FMCW";

    auto out = skill::lidar_scan::apply(*stock, in);
    EXPECT_EQ(out.signature.skill_id, std::string("lidar_scan"));
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("lidar_scan"));
    EXPECT_EQ(out.signature.tooling.tool_type, std::string("lidar_scanner"));
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
    EXPECT_GT(out.signature.tooling.est_cycle_time_s, 0.0);
}

// ── 4. Recognize metadata replay ─────────────────────────────────────────
TEST(SkillLidarScan, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::lidar_scan::Input in;
    in.range_m        = 30.0;
    in.points_per_sec = 5.0e5;
    in.ranging_class  = "AMCW";
    auto out = skill::lidar_scan::apply(*stock, in);

    auto cands = skill::lidar_scan::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-9);
    EXPECT_EQ(cands[0].skill_id, std::string("lidar_scan"));

    skill::Workpiece raw(out.workpiece->shape());
    EXPECT_TRUE(skill::lidar_scan::recognize(raw).empty());
}

// ── 5. range_m + points_per_sec + ranging_class round-trip ───────────────
TEST(SkillLidarScan, RangeRateClassRoundtrip)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::lidar_scan::Input in;
    in.range_m        = 75.5;
    in.points_per_sec = 8.0e5;
    in.ranging_class  = "FMCW";

    auto out = skill::lidar_scan::apply(*stock, in);
    EXPECT_NEAR(out.signature.pattern["range_m"].get<double>(), 75.5, 1e-12);
    EXPECT_NEAR(out.signature.pattern["points_per_sec"].get<double>(),
                8.0e5, 1e-3);
    EXPECT_EQ(out.signature.pattern["ranging_class"].get<std::string>(),
              std::string("FMCW"));
    EXPECT_TRUE(out.signature.pattern["long_range"].get<bool>())
        << "75.5 m exceeds the 10 m long-range threshold";
}
