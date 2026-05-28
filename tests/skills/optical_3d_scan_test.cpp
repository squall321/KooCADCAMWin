// @lat: [[process/test-strategy#skill round-trip]]
//
// optical_3d_scan — INSPECTION skill: passive stereo triangulation.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/optical_3d_scan.hpp"

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
TEST(SkillOptical3dScan, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);
    const double volBefore = volumeOf(stock->shape());

    skill::optical_3d_scan::Input in;
    in.baseline_mm   = 120.0;
    in.resolution_um = 50.0;

    auto out = skill::optical_3d_scan::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, volBefore * 1e-6);
    EXPECT_TRUE(out.signature.pattern["is_inspection_only"].get<bool>());
    EXPECT_FALSE(out.signature.pattern["geometric_change"].get<bool>());
}

// ── 2. ValidateRejectsBadInput: bad baseline / resolution ────────────────
TEST(SkillOptical3dScan, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::optical_3d_scan::Input badBase;
    badBase.baseline_mm   = 0.0;
    badBase.resolution_um = 50.0;
    auto r1 = skill::optical_3d_scan::validate(*stock, badBase);
    EXPECT_FALSE(r1.passed);
    EXPECT_THROW(skill::optical_3d_scan::apply(*stock, badBase),
                 skill::SkillError);

    skill::optical_3d_scan::Input badRes;
    badRes.baseline_mm   = 120.0;
    badRes.resolution_um = -10.0;
    auto r2 = skill::optical_3d_scan::validate(*stock, badRes);
    EXPECT_FALSE(r2.passed);
}

// ── 3. Signature records kind + tooling metadata ─────────────────────────
TEST(SkillOptical3dScan, SignatureRecordsKindAndTooling)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::optical_3d_scan::Input in;
    in.baseline_mm   = 200.0;
    in.resolution_um = 30.0;

    auto out = skill::optical_3d_scan::apply(*stock, in);
    EXPECT_EQ(out.signature.skill_id, std::string("optical_3d_scan"));
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("optical_3d_scan"));
    EXPECT_EQ(out.signature.tooling.tool_type,
              std::string("stereo_camera_pair"));
    EXPECT_EQ(out.signature.tooling.tool_material,
              std::string("optical_glass"));
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
    EXPECT_GT(out.signature.tooling.est_cycle_time_s, 0.0);
}

// ── 4. Recognize metadata replay ─────────────────────────────────────────
TEST(SkillOptical3dScan, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::optical_3d_scan::Input in;
    in.baseline_mm   = 120.0;
    in.resolution_um = 50.0;
    auto out = skill::optical_3d_scan::apply(*stock, in);

    auto cands = skill::optical_3d_scan::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-9);
    EXPECT_EQ(cands[0].skill_id, std::string("optical_3d_scan"));

    skill::Workpiece raw(out.workpiece->shape());
    EXPECT_TRUE(skill::optical_3d_scan::recognize(raw).empty());
}

// ── 5. baseline_mm + resolution_um + metrology_grade round-trip ──────────
TEST(SkillOptical3dScan, BaselineResolutionMetrologyRoundtrip)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::optical_3d_scan::Input metro;
    metro.baseline_mm   = 250.0;
    metro.resolution_um = 25.0;
    auto outM = skill::optical_3d_scan::apply(*stock, metro);
    EXPECT_NEAR(outM.signature.pattern["baseline_mm"].get<double>(),
                250.0, 1e-12);
    EXPECT_NEAR(outM.signature.pattern["resolution_um"].get<double>(),
                25.0, 1e-12);
    EXPECT_TRUE(outM.signature.pattern["metrology_grade"].get<bool>())
        << "25 µm pixel footprint qualifies as metrology grade (≤ 50 µm)";

    skill::optical_3d_scan::Input coarse;
    coarse.baseline_mm   = 80.0;
    coarse.resolution_um = 80.0;
    auto outC = skill::optical_3d_scan::apply(*stock, coarse);
    EXPECT_FALSE(outC.signature.pattern["metrology_grade"].get<bool>())
        << "80 µm pixel footprint is above metrology threshold";
}
