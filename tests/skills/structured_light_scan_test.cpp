// @lat: [[process/test-strategy#skill round-trip]]
//
// structured_light_scan — INSPECTION skill: encoded-pattern projector +
// camera(s) triangulation.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/structured_light_scan.hpp"

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
TEST(SkillStructuredLightScan, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);
    const double volBefore = volumeOf(stock->shape());

    skill::structured_light_scan::Input in;
    in.projector_pattern = "phase_shift";
    in.camera_count      = 2;
    in.accuracy_um       = 25.0;

    auto out = skill::structured_light_scan::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, volBefore * 1e-6);
    EXPECT_TRUE(out.signature.pattern["is_inspection_only"].get<bool>());
    EXPECT_FALSE(out.signature.pattern["geometric_change"].get<bool>());
}

// ── 2. ValidateRejectsBadInput: bad camera count / accuracy ──────────────
TEST(SkillStructuredLightScan, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::structured_light_scan::Input noCam;
    noCam.projector_pattern = "phase_shift";
    noCam.camera_count      = 0;
    noCam.accuracy_um       = 25.0;
    auto r1 = skill::structured_light_scan::validate(*stock, noCam);
    EXPECT_FALSE(r1.passed);
    EXPECT_THROW(skill::structured_light_scan::apply(*stock, noCam),
                 skill::SkillError);

    skill::structured_light_scan::Input badAcc;
    badAcc.projector_pattern = "phase_shift";
    badAcc.camera_count      = 2;
    badAcc.accuracy_um       = -1.0;
    auto r2 = skill::structured_light_scan::validate(*stock, badAcc);
    EXPECT_FALSE(r2.passed);
}

// ── 3. Signature records kind + tooling metadata ─────────────────────────
TEST(SkillStructuredLightScan, SignatureRecordsKindAndTooling)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::structured_light_scan::Input in;
    in.projector_pattern = "hybrid";
    in.camera_count      = 3;
    in.accuracy_um       = 15.0;

    auto out = skill::structured_light_scan::apply(*stock, in);
    EXPECT_EQ(out.signature.skill_id, std::string("structured_light_scan"));
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("structured_light_scan"));
    EXPECT_EQ(out.signature.tooling.tool_type,
              std::string("structured_light_scanner"));
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
    EXPECT_GT(out.signature.tooling.est_cycle_time_s, 0.0);
    EXPECT_NEAR(out.signature.pattern["accuracy_um"].get<double>(), 15.0, 1e-12);
}

// ── 4. Recognize metadata replay ─────────────────────────────────────────
TEST(SkillStructuredLightScan, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::structured_light_scan::Input in;
    in.projector_pattern = "gray";
    in.camera_count      = 2;
    in.accuracy_um       = 30.0;
    auto out = skill::structured_light_scan::apply(*stock, in);

    auto cands = skill::structured_light_scan::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-9);
    EXPECT_EQ(cands[0].skill_id, std::string("structured_light_scan"));

    skill::Workpiece raw(out.workpiece->shape());
    EXPECT_TRUE(skill::structured_light_scan::recognize(raw).empty());
}

// ── 5. Projector pattern → frames_captured + stereo round-trip ───────────
TEST(SkillStructuredLightScan, PatternFramesAndStereoRoundtrip)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::structured_light_scan::Input gray;
    gray.projector_pattern = "gray";
    gray.camera_count      = 2;
    gray.accuracy_um       = 30.0;
    auto outG = skill::structured_light_scan::apply(*stock, gray);
    EXPECT_EQ(outG.signature.pattern["frames_captured"].get<int>(), 10);
    EXPECT_TRUE(outG.signature.pattern["stereo_enabled"].get<bool>());

    skill::structured_light_scan::Input phase;
    phase.projector_pattern = "phase_shift";
    phase.camera_count      = 1;
    phase.accuracy_um       = 20.0;
    auto outP = skill::structured_light_scan::apply(*stock, phase);
    EXPECT_EQ(outP.signature.pattern["frames_captured"].get<int>(), 4);
    EXPECT_FALSE(outP.signature.pattern["stereo_enabled"].get<bool>())
        << "camera_count == 1 should disable stereo cross-check";
    EXPECT_EQ(outP.signature.pattern["projector_pattern"].get<std::string>(),
              std::string("phase_shift"));

    skill::structured_light_scan::Input hyb;
    hyb.projector_pattern = "hybrid";
    hyb.camera_count      = 2;
    hyb.accuracy_um       = 10.0;
    auto outH = skill::structured_light_scan::apply(*stock, hyb);
    EXPECT_EQ(outH.signature.pattern["frames_captured"].get<int>(), 14)
        << "hybrid captures Gray + phase frames combined";
}
