// @lat: [[process/test-strategy#skill round-trip]]
//
// force_feedback_scan — INSPECTION skill: closed-loop force-controlled
// haptic surface sweep.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/force_feedback_scan.hpp"

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
TEST(SkillForceFeedbackScan, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);
    const double volBefore = volumeOf(stock->shape());

    skill::force_feedback_scan::Input in;
    in.max_force_n        = 0.5;
    in.scan_resolution_um = 10.0;

    auto out = skill::force_feedback_scan::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, volBefore * 1e-6);
    EXPECT_TRUE(out.signature.pattern["is_inspection_only"].get<bool>());
    EXPECT_FALSE(out.signature.pattern["geometric_change"].get<bool>());
}

// ── 2. ValidateRejectsBadInput: zero force / zero resolution ─────────────
TEST(SkillForceFeedbackScan, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::force_feedback_scan::Input noForce;
    noForce.max_force_n        = 0.0;
    noForce.scan_resolution_um = 10.0;
    auto r1 = skill::force_feedback_scan::validate(*stock, noForce);
    EXPECT_FALSE(r1.passed);
    EXPECT_THROW(skill::force_feedback_scan::apply(*stock, noForce),
                 skill::SkillError);

    skill::force_feedback_scan::Input badRes;
    badRes.max_force_n        = 0.5;
    badRes.scan_resolution_um = -1.0;
    auto r2 = skill::force_feedback_scan::validate(*stock, badRes);
    EXPECT_FALSE(r2.passed);
}

// ── 3. Signature records kind + tooling metadata ─────────────────────────
TEST(SkillForceFeedbackScan, SignatureRecordsKindAndTooling)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::force_feedback_scan::Input in;
    in.max_force_n        = 1.0;
    in.scan_resolution_um = 5.0;

    auto out = skill::force_feedback_scan::apply(*stock, in);
    EXPECT_EQ(out.signature.skill_id, std::string("force_feedback_scan"));
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("force_feedback_scan"));
    EXPECT_EQ(out.signature.tooling.tool_type,
              std::string("haptic_stylus"));
    EXPECT_EQ(out.signature.tooling.tool_material,
              std::string("sapphire"));
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
    EXPECT_GT(out.signature.tooling.est_cycle_time_s, 0.0);
}

// ── 4. Recognize metadata replay ─────────────────────────────────────────
TEST(SkillForceFeedbackScan, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::force_feedback_scan::Input in;
    in.max_force_n        = 0.3;
    in.scan_resolution_um = 20.0;
    auto out = skill::force_feedback_scan::apply(*stock, in);

    auto cands = skill::force_feedback_scan::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-9);
    EXPECT_EQ(cands[0].skill_id, std::string("force_feedback_scan"));

    skill::Workpiece raw(out.workpiece->shape());
    EXPECT_TRUE(skill::force_feedback_scan::recognize(raw).empty());
}

// ── 5. max_force_n + scan_resolution_um + low_force flag round-trip ──────
TEST(SkillForceFeedbackScan, ForceResolutionLowForceRoundtrip)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::force_feedback_scan::Input gentle;
    gentle.max_force_n        = 0.05;     // 50 mN — under 100 mN low-force threshold
    gentle.scan_resolution_um = 2.0;
    auto outG = skill::force_feedback_scan::apply(*stock, gentle);
    EXPECT_NEAR(outG.signature.pattern["max_force_n"].get<double>(),
                0.05, 1e-12);
    EXPECT_NEAR(outG.signature.pattern["scan_resolution_um"].get<double>(),
                2.0, 1e-12);
    EXPECT_TRUE(outG.signature.pattern["low_force_mode"].get<bool>())
        << "50 mN max force should activate low-force mode (< 100 mN)";

    skill::force_feedback_scan::Input firm;
    firm.max_force_n        = 2.0;
    firm.scan_resolution_um = 30.0;
    auto outF = skill::force_feedback_scan::apply(*stock, firm);
    EXPECT_FALSE(outF.signature.pattern["low_force_mode"].get<bool>())
        << "2 N max force is above low-force threshold";
}
