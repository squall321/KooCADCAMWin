// @lat: [[process/test-strategy#skill round-trip]]
//
// pid_tune skill — 5-case sweep covering metadata-only synthesis,
// DFM rejection of bad gains, signature recording, recognize replay,
// and a specific assertion (response_ms → cycle_time relationship).

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/pid_tune.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

using namespace koocadcam;

namespace {

double volumeOf(const TopoDS_Shape& s)
{
    GProp_GProps p;
    BRepGProp::VolumeProperties(s, p);
    return p.Mass();
}

bool hasFinding(const skill::DFMReport& r, const std::string& code)
{
    for (const auto& f : r.findings) if (f.code == code) return true;
    return false;
}

skill::pid_tune::Input goodInput()
{
    skill::pid_tune::Input in;
    in.kp                 = 1.5;
    in.ki                 = 0.2;
    in.kd                 = 0.05;
    in.target_response_ms = 250.0;
    in.overshoot_pct      = 5.0;
    return in;
}

}  // namespace

// ─── 1. Apply: geometry CLONED unchanged ──────────────────────────────────
TEST(SkillPidTune, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    auto out = skill::pid_tune::apply(*stock, goodInput());
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("pid_tune"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. Validate REJECTS bad input (negative kp, zero target_response) ────
TEST(SkillPidTune, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto in = goodInput();
    in.kp = -0.5;
    auto r = skill::pid_tune::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-INPUT"));
    EXPECT_THROW(skill::pid_tune::apply(*stock, in), skill::SkillError);

    auto in2 = goodInput();
    in2.target_response_ms = 0.0;
    auto r2 = skill::pid_tune::validate(*stock, in2);
    EXPECT_FALSE(r2.passed);
    EXPECT_TRUE(hasFinding(r2, "DFM-INPUT"));

    auto in3 = goodInput();
    in3.overshoot_pct = 80.0;       // > 50% band
    auto r3 = skill::pid_tune::validate(*stock, in3);
    EXPECT_FALSE(r3.passed);
    EXPECT_TRUE(hasFinding(r3, "DFM-PID-OVER"));
}

// ─── 3. Signature records kind + kp/ki/kd/target_response/overshoot ──────
TEST(SkillPidTune, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto in = goodInput();
    in.kp                 = 2.7;
    in.ki                 = 0.4;
    in.kd                 = 0.08;
    in.target_response_ms = 500.0;
    in.overshoot_pct      = 10.0;

    auto out = skill::pid_tune::apply(*stock, in);

    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("pid_tune"));
    EXPECT_NEAR(out.signature.pattern["kp"].get<double>(), 2.7,  1e-9);
    EXPECT_NEAR(out.signature.pattern["ki"].get<double>(), 0.4,  1e-9);
    EXPECT_NEAR(out.signature.pattern["kd"].get<double>(), 0.08, 1e-9);
    EXPECT_NEAR(out.signature.pattern["target_response_ms"].get<double>(),
                500.0, 1e-9);
    EXPECT_NEAR(out.signature.pattern["overshoot_pct"].get<double>(),
                10.0, 1e-9);
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
    EXPECT_EQ(out.signature.tooling.tool_type,
              std::string("process_controller"));
}

// ─── 4. Recognize via metadata replay ─────────────────────────────────────
TEST(SkillPidTune, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    auto out = skill::pid_tune::apply(*stock, goodInput());

    auto cands = skill::pid_tune::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_NEAR(cands[0].recovered_params["kp"].get<double>(), 1.5, 1e-9);

    // Strip metadata → recognize empty.
    skill::Workpiece raw(out.workpiece->shape());
    auto cands2 = skill::pid_tune::recognize(raw);
    EXPECT_TRUE(cands2.empty())
        << "pid_tune cannot be recognized geometrically";
}

// ─── 5. SPECIFIC: target_response_ms drives est_cycle_time_s × 10 ─────────
TEST(SkillPidTune, CycleTimeReflectsTargetResponse)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto fastIn = goodInput();
    fastIn.target_response_ms = 100.0;
    auto fastOut = skill::pid_tune::apply(*stock, fastIn);
    // 0.1 s × 10 = 1.0 s commissioning budget.
    EXPECT_NEAR(fastOut.signature.tooling.est_cycle_time_s, 1.0, 1e-6);

    auto slowIn = goodInput();
    slowIn.target_response_ms = 2000.0;
    auto slowOut = skill::pid_tune::apply(*stock, slowIn);
    EXPECT_NEAR(slowOut.signature.tooling.est_cycle_time_s, 20.0, 1e-6);

    // A slower loop should always take longer to commission.
    EXPECT_LT(fastOut.signature.tooling.est_cycle_time_s,
              slowOut.signature.tooling.est_cycle_time_s);
}
