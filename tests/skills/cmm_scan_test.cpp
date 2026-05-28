// @lat: [[process/test-strategy#skill round-trip]]
//
// cmm_scan — INSPECTION skill: caller-supplied waypoint program with
// nominal / measured deltas, mapped to a CMM macro.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/cmm_scan.hpp"

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

// ── 1. Apply: geometry unchanged (clone, volume preserved) ───────────────
TEST(SkillCmmScan, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);
    const double volBefore = volumeOf(stock->shape());

    skill::cmm_scan::Input in;
    in.nominal_points_3d   = { gp_Pnt(10, 10, 10), gp_Pnt(25, 25, 10), gp_Pnt(40, 40, 10) };
    in.measured_mm         = { 0.005, -0.003, 0.010 };
    in.target_tolerance_mm = 0.025;
    in.stylus_diameter_mm  = 2.0;

    auto out = skill::cmm_scan::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, volBefore * 1e-6);
    EXPECT_TRUE(out.signature.pattern["is_inspection_only"].get<bool>());
    EXPECT_FALSE(out.signature.pattern["geometric_change"].get<bool>());
}

// ── 2. ValidateRejectsBadInput: empty waypoint list, zero tolerance ──────
TEST(SkillCmmScan, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::cmm_scan::Input empty;
    empty.target_tolerance_mm = 0.025;
    auto r1 = skill::cmm_scan::validate(*stock, empty);
    EXPECT_FALSE(r1.passed);
    EXPECT_THROW(skill::cmm_scan::apply(*stock, empty), skill::SkillError);

    skill::cmm_scan::Input zeroTol;
    zeroTol.nominal_points_3d   = { gp_Pnt(25, 25, 10) };
    zeroTol.target_tolerance_mm = 0.0;
    auto r2 = skill::cmm_scan::validate(*stock, zeroTol);
    EXPECT_FALSE(r2.passed);

    // Mismatched measured vs nominal sizes.
    skill::cmm_scan::Input mism;
    mism.nominal_points_3d   = { gp_Pnt(10, 10, 10), gp_Pnt(20, 20, 10) };
    mism.measured_mm         = { 0.001 };       // size mismatch
    mism.target_tolerance_mm = 0.025;
    auto r3 = skill::cmm_scan::validate(*stock, mism);
    EXPECT_FALSE(r3.passed);
}

// ── 3. Signature records kind + tooling metadata ─────────────────────────
TEST(SkillCmmScan, SignatureRecordsKindAndTooling)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::cmm_scan::Input in;
    in.nominal_points_3d   = { gp_Pnt(25, 25, 10) };
    in.target_tolerance_mm = 0.050;
    in.stylus_diameter_mm  = 3.0;

    auto out = skill::cmm_scan::apply(*stock, in);
    EXPECT_EQ(out.signature.skill_id, std::string("cmm_scan"));
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("cmm_scan"));
    EXPECT_EQ(out.signature.tooling.tool_type, std::string("touch_probe"));
    EXPECT_NEAR(out.signature.tooling.tool_dia_mm, 3.0, 1e-9);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
    EXPECT_GT(out.signature.tooling.est_cycle_time_s, 0.0);
}

// ── 4. Recognize: metadata replay yields confidence 1.0 ──────────────────
TEST(SkillCmmScan, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::cmm_scan::Input in;
    in.nominal_points_3d   = { gp_Pnt(25, 25, 10), gp_Pnt(30, 30, 10) };
    in.target_tolerance_mm = 0.025;
    auto out = skill::cmm_scan::apply(*stock, in);

    auto cands = skill::cmm_scan::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-9);
    EXPECT_EQ(cands[0].skill_id, std::string("cmm_scan"));

    // Stripped workpiece → no recognition.
    skill::Workpiece raw(out.workpiece->shape());
    EXPECT_TRUE(skill::cmm_scan::recognize(raw).empty());
}

// ── 5. Target tolerance + waypoint count round-trip through pattern ──────
TEST(SkillCmmScan, TargetToleranceAndWaypointsRoundtrip)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::cmm_scan::Input in;
    in.nominal_points_3d   = {
        gp_Pnt(5, 5, 10), gp_Pnt(45, 5, 10),
        gp_Pnt(45, 45, 10), gp_Pnt(5, 45, 10),
    };
    in.measured_mm         = { 0.001, -0.002, 0.030, -0.001 };  // one outside ±0.025
    in.target_tolerance_mm = 0.025;
    in.stylus_diameter_mm  = 2.0;

    auto out = skill::cmm_scan::apply(*stock, in);
    EXPECT_NEAR(out.signature.pattern["target_tolerance_mm"].get<double>(),
                0.025, 1e-12);
    EXPECT_EQ(out.signature.pattern["waypoint_count"].get<int>(), 4);
    EXPECT_EQ(out.signature.pattern["deltas_mm"].size(), 4u);
    EXPECT_TRUE(out.signature.pattern["has_measurements"].get<bool>());
    EXPECT_FALSE(out.signature.pattern["all_conform"].get<bool>())
        << "one waypoint deviation (0.030) exceeds target ±0.025";
    EXPECT_NEAR(out.signature.pattern["deltas_mm"][2].get<double>(),
                0.030, 1e-12);
}
