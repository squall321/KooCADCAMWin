// @lat: [[process/test-strategy#skill round-trip]]
//
// schedule_parameter_ramp skill — 5-case sweep covering metadata-only
// synthesis, DFM rejection of bad input, signature recording,
// recognize replay, and a specific assertion (derived step_delta).

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/schedule_parameter_ramp.hpp"

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

skill::schedule_parameter_ramp::Input goodInput()
{
    skill::schedule_parameter_ramp::Input in;
    in.start_value   = 20.0;
    in.end_value     = 120.0;
    in.ramp_time_min = 60.0;
    in.steps         = 10;
    return in;
}

}  // namespace

// ─── 1. Apply: geometry CLONED unchanged ──────────────────────────────────
TEST(SkillScheduleParameterRamp, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    auto out = skill::schedule_parameter_ramp::apply(*stock, goodInput());
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("schedule_parameter_ramp"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. Validate REJECTS bad input (zero time, zero steps) ────────────────
TEST(SkillScheduleParameterRamp, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto in = goodInput();
    in.ramp_time_min = 0.0;
    auto r = skill::schedule_parameter_ramp::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-INPUT"));
    EXPECT_THROW(skill::schedule_parameter_ramp::apply(*stock, in),
                 skill::SkillError);

    auto in2 = goodInput();
    in2.steps = 0;
    auto r2 = skill::schedule_parameter_ramp::validate(*stock, in2);
    EXPECT_FALSE(r2.passed);
    EXPECT_TRUE(hasFinding(r2, "DFM-INPUT"));

    auto in3 = goodInput();
    in3.ramp_time_min = -10.0;
    auto r3 = skill::schedule_parameter_ramp::validate(*stock, in3);
    EXPECT_FALSE(r3.passed);
    EXPECT_TRUE(hasFinding(r3, "DFM-INPUT"));
}

// ─── 3. Signature records kind + start/end/time/steps ─────────────────────
TEST(SkillScheduleParameterRamp, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto in = goodInput();
    in.start_value   = 5.0;
    in.end_value     = 95.0;
    in.ramp_time_min = 30.0;
    in.steps         = 6;

    auto out = skill::schedule_parameter_ramp::apply(*stock, in);

    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("schedule_parameter_ramp"));
    EXPECT_NEAR(out.signature.pattern["start_value"].get<double>(),
                5.0, 1e-9);
    EXPECT_NEAR(out.signature.pattern["end_value"].get<double>(),
                95.0, 1e-9);
    EXPECT_NEAR(out.signature.pattern["ramp_time_min"].get<double>(),
                30.0, 1e-9);
    EXPECT_EQ(out.signature.pattern["steps"].get<int>(), 6);
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
    EXPECT_EQ(out.signature.tooling.tool_type,
              std::string("ramp_scheduler"));
}

// ─── 4. Recognize via metadata replay ─────────────────────────────────────
TEST(SkillScheduleParameterRamp, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    auto out = skill::schedule_parameter_ramp::apply(*stock, goodInput());

    auto cands = skill::schedule_parameter_ramp::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_NEAR(cands[0].recovered_params["start_value"].get<double>(),
                20.0, 1e-9);
    EXPECT_NEAR(cands[0].recovered_params["end_value"].get<double>(),
                120.0, 1e-9);
    EXPECT_EQ(cands[0].recovered_params["steps"].get<int>(), 10);

    // Strip metadata → recognize empty.
    skill::Workpiece raw(out.workpiece->shape());
    auto cands2 = skill::schedule_parameter_ramp::recognize(raw);
    EXPECT_TRUE(cands2.empty())
        << "schedule_parameter_ramp cannot be recognized geometrically";
}

// ─── 5. SPECIFIC: derived step_delta == (end - start) / steps ─────────────
TEST(SkillScheduleParameterRamp, DerivedStepDeltaMatchesRamp)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto in = goodInput();
    in.start_value = 20.0;
    in.end_value   = 120.0;
    in.steps       = 10;        // delta = 10.0 per step

    auto out = skill::schedule_parameter_ramp::apply(*stock, in);
    EXPECT_NEAR(out.signature.pattern["step_delta"].get<double>(),
                10.0, 1e-9);

    // est_cycle_time_s should equal ramp_time_min × 60.
    EXPECT_NEAR(out.signature.tooling.est_cycle_time_s,
                in.ramp_time_min * 60.0, 1e-6);

    // Flat ramp (start == end) → DFM info, step_delta == 0.
    auto flatIn = goodInput();
    flatIn.start_value = 50.0;
    flatIn.end_value   = 50.0;
    auto flatR = skill::schedule_parameter_ramp::validate(*stock, flatIn);
    EXPECT_TRUE(flatR.passed)
        << "flat ramp is INFO-level, not blocking";
    EXPECT_TRUE(hasFinding(flatR, "DFM-RAMP-SLOPE"));

    auto flatOut = skill::schedule_parameter_ramp::apply(*stock, flatIn);
    EXPECT_NEAR(flatOut.signature.pattern["step_delta"].get<double>(),
                0.0, 1e-9);
}
