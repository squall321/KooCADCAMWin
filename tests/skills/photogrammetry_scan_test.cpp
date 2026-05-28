// @lat: [[process/test-strategy#skill round-trip]]
//
// photogrammetry_scan — INSPECTION skill: multi-camera SfM/MVS scan.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/photogrammetry_scan.hpp"

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
TEST(SkillPhotogrammetryScan, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);
    const double volBefore = volumeOf(stock->shape());

    skill::photogrammetry_scan::Input in;
    in.camera_count   = 12;
    in.accuracy_class = "precision";
    in.coverage_pct   = 90.0;

    auto out = skill::photogrammetry_scan::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, volBefore * 1e-6);
    EXPECT_TRUE(out.signature.pattern["is_inspection_only"].get<bool>());
    EXPECT_FALSE(out.signature.pattern["geometric_change"].get<bool>());
}

// ── 2. ValidateRejectsBadInput: too few cameras, bad coverage ────────────
TEST(SkillPhotogrammetryScan, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::photogrammetry_scan::Input tooFew;
    tooFew.camera_count   = 1;        // < 2 minimum
    tooFew.accuracy_class = "precision";
    tooFew.coverage_pct   = 90.0;
    auto r1 = skill::photogrammetry_scan::validate(*stock, tooFew);
    EXPECT_FALSE(r1.passed);
    EXPECT_THROW(skill::photogrammetry_scan::apply(*stock, tooFew),
                 skill::SkillError);

    skill::photogrammetry_scan::Input badCov;
    badCov.camera_count   = 8;
    badCov.accuracy_class = "precision";
    badCov.coverage_pct   = 150.0;    // > 100
    auto r2 = skill::photogrammetry_scan::validate(*stock, badCov);
    EXPECT_FALSE(r2.passed);
}

// ── 3. Signature records kind + tooling metadata ─────────────────────────
TEST(SkillPhotogrammetryScan, SignatureRecordsKindAndTooling)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::photogrammetry_scan::Input in;
    in.camera_count   = 16;
    in.accuracy_class = "metrology";
    in.coverage_pct   = 95.0;

    auto out = skill::photogrammetry_scan::apply(*stock, in);
    EXPECT_EQ(out.signature.skill_id, std::string("photogrammetry_scan"));
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("photogrammetry_scan"));
    EXPECT_EQ(out.signature.tooling.tool_type, std::string("camera_array"));
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
    EXPECT_GT(out.signature.tooling.est_cycle_time_s, 0.0);
}

// ── 4. Recognize metadata replay ─────────────────────────────────────────
TEST(SkillPhotogrammetryScan, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::photogrammetry_scan::Input in;
    in.camera_count   = 8;
    in.accuracy_class = "precision";
    in.coverage_pct   = 85.0;
    auto out = skill::photogrammetry_scan::apply(*stock, in);

    auto cands = skill::photogrammetry_scan::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-9);
    EXPECT_EQ(cands[0].skill_id, std::string("photogrammetry_scan"));

    skill::Workpiece raw(out.workpiece->shape());
    EXPECT_TRUE(skill::photogrammetry_scan::recognize(raw).empty());
}

// ── 5. camera_count + accuracy_class → target_accuracy_mm round-trip ─────
TEST(SkillPhotogrammetryScan, CameraAccuracyClassRoundtrip)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::photogrammetry_scan::Input metro;
    metro.camera_count   = 24;
    metro.accuracy_class = "metrology";
    metro.coverage_pct   = 92.0;
    auto outM = skill::photogrammetry_scan::apply(*stock, metro);
    EXPECT_EQ(outM.signature.pattern["camera_count"].get<int>(), 24);
    EXPECT_EQ(outM.signature.pattern["accuracy_class"].get<std::string>(),
              std::string("metrology"));
    EXPECT_NEAR(outM.signature.pattern["target_accuracy_mm"].get<double>(),
                0.005, 1e-9);

    skill::photogrammetry_scan::Input ind;
    ind.camera_count   = 4;
    ind.accuracy_class = "industrial";
    ind.coverage_pct   = 70.0;
    auto outI = skill::photogrammetry_scan::apply(*stock, ind);
    EXPECT_NEAR(outI.signature.pattern["target_accuracy_mm"].get<double>(),
                0.100, 1e-9);
}
