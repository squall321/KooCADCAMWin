// @lat: [[process/test-strategy#skill round-trip]]
//
// ct_scan — INSPECTION skill: industrial X-ray computed tomography.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/ct_scan.hpp"

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
TEST(SkillCtScan, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);
    const double volBefore = volumeOf(stock->shape());

    skill::ct_scan::Input in;
    in.voxel_size_mm         = 0.050;
    in.dose_kvp              = 225.0;
    in.defect_size_target_mm = 0.150;

    auto out = skill::ct_scan::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, volBefore * 1e-6);
    EXPECT_TRUE(out.signature.pattern["is_inspection_only"].get<bool>());
    EXPECT_FALSE(out.signature.pattern["geometric_change"].get<bool>());
}

// ── 2. ValidateRejectsBadInput: insane voxel / dose / Nyquist violation ──
TEST(SkillCtScan, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::ct_scan::Input badVoxel;
    badVoxel.voxel_size_mm         = 0.0;       // must be > 0
    badVoxel.dose_kvp              = 225.0;
    badVoxel.defect_size_target_mm = 0.150;
    auto r1 = skill::ct_scan::validate(*stock, badVoxel);
    EXPECT_FALSE(r1.passed);
    EXPECT_THROW(skill::ct_scan::apply(*stock, badVoxel), skill::SkillError);

    skill::ct_scan::Input badDose;
    badDose.voxel_size_mm         = 0.050;
    badDose.dose_kvp              = 10.0;       // < 40 kVp lower bound
    badDose.defect_size_target_mm = 0.150;
    auto r2 = skill::ct_scan::validate(*stock, badDose);
    EXPECT_FALSE(r2.passed);

    skill::ct_scan::Input nyquist;
    nyquist.voxel_size_mm         = 0.500;
    nyquist.dose_kvp              = 225.0;
    nyquist.defect_size_target_mm = 0.100;      // smaller than voxel
    auto r3 = skill::ct_scan::validate(*stock, nyquist);
    EXPECT_FALSE(r3.passed);
}

// ── 3. Signature records kind + tooling metadata ─────────────────────────
TEST(SkillCtScan, SignatureRecordsKindAndTooling)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::ct_scan::Input in;
    in.voxel_size_mm         = 0.025;
    in.dose_kvp              = 300.0;
    in.defect_size_target_mm = 0.100;

    auto out = skill::ct_scan::apply(*stock, in);
    EXPECT_EQ(out.signature.skill_id, std::string("ct_scan"));
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("ct_scan"));
    EXPECT_EQ(out.signature.tooling.tool_type, std::string("ct_scanner"));
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
    EXPECT_GT(out.signature.tooling.est_cycle_time_s, 0.0);
}

// ── 4. Recognize metadata replay ─────────────────────────────────────────
TEST(SkillCtScan, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::ct_scan::Input in;
    in.voxel_size_mm         = 0.050;
    in.dose_kvp              = 225.0;
    in.defect_size_target_mm = 0.150;
    auto out = skill::ct_scan::apply(*stock, in);

    auto cands = skill::ct_scan::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-9);
    EXPECT_EQ(cands[0].skill_id, std::string("ct_scan"));

    skill::Workpiece raw(out.workpiece->shape());
    EXPECT_TRUE(skill::ct_scan::recognize(raw).empty());
}

// ── 5. voxel/dose + internal-defect capability round-trip ────────────────
TEST(SkillCtScan, VoxelDoseAndCapabilityRoundtrip)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::ct_scan::Input in;
    in.voxel_size_mm         = 0.050;
    in.dose_kvp              = 300.0;
    in.defect_size_target_mm = 0.200;

    auto out = skill::ct_scan::apply(*stock, in);
    EXPECT_NEAR(out.signature.pattern["voxel_size_mm"].get<double>(),
                0.050, 1e-12);
    EXPECT_NEAR(out.signature.pattern["dose_kvp"].get<double>(),
                300.0, 1e-12);
    EXPECT_NEAR(out.signature.pattern["defect_size_target_mm"].get<double>(),
                0.200, 1e-12);
    EXPECT_TRUE(out.signature.pattern["detect_internal_defects"].get<bool>())
        << "voxel 50µm easily resolves a 200µm defect";
}
