// @lat: [[process/test-strategy#skill round-trip]]
//
// alarm_threshold_set skill — 5-case sweep covering metadata-only
// synthesis, DFM rejection of inverted band, signature recording,
// recognize replay, and a specific assertion (unknown action → info,
// known action → no info).

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/alarm_threshold_set.hpp"

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

skill::alarm_threshold_set::Input goodInput()
{
    skill::alarm_threshold_set::Input in;
    in.alarm_id       = "ALM-TEMP-01";
    in.parameter_name = "reactor_temp_c";
    in.low            = 60.0;
    in.high           = 95.0;
    in.action         = "alert";
    return in;
}

}  // namespace

// ─── 1. Apply: geometry CLONED unchanged ──────────────────────────────────
TEST(SkillAlarmThresholdSet, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    auto out = skill::alarm_threshold_set::apply(*stock, goodInput());
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("alarm_threshold_set"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. Validate REJECTS bad input (low >= high, empty ids) ───────────────
TEST(SkillAlarmThresholdSet, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto in = goodInput();
    in.low  = 100.0;
    in.high = 90.0;            // inverted band
    auto r = skill::alarm_threshold_set::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-ALARM-BAND"));
    EXPECT_THROW(skill::alarm_threshold_set::apply(*stock, in),
                 skill::SkillError);

    auto in2 = goodInput();
    in2.alarm_id = "";
    auto r2 = skill::alarm_threshold_set::validate(*stock, in2);
    EXPECT_FALSE(r2.passed);
    EXPECT_TRUE(hasFinding(r2, "DFM-INPUT"));

    auto in3 = goodInput();
    in3.parameter_name = "";
    auto r3 = skill::alarm_threshold_set::validate(*stock, in3);
    EXPECT_FALSE(r3.passed);
    EXPECT_TRUE(hasFinding(r3, "DFM-INPUT"));

    // Equal low/high — also a band error.
    auto in4 = goodInput();
    in4.low  = 50.0;
    in4.high = 50.0;
    auto r4 = skill::alarm_threshold_set::validate(*stock, in4);
    EXPECT_FALSE(r4.passed);
    EXPECT_TRUE(hasFinding(r4, "DFM-ALARM-BAND"));
}

// ─── 3. Signature records kind + alarm_id/param/low/high/action ──────────
TEST(SkillAlarmThresholdSet, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto in = goodInput();
    in.alarm_id       = "ALM-PRESS-42";
    in.parameter_name = "vessel_pressure_psi";
    in.low            = 10.0;
    in.high           = 200.0;
    in.action         = "shutdown";

    auto out = skill::alarm_threshold_set::apply(*stock, in);

    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("alarm_threshold_set"));
    EXPECT_EQ(out.signature.pattern["alarm_id"].get<std::string>(),
              std::string("ALM-PRESS-42"));
    EXPECT_EQ(out.signature.pattern["parameter_name"].get<std::string>(),
              std::string("vessel_pressure_psi"));
    EXPECT_NEAR(out.signature.pattern["low"].get<double>(),  10.0, 1e-9);
    EXPECT_NEAR(out.signature.pattern["high"].get<double>(), 200.0, 1e-9);
    EXPECT_EQ(out.signature.pattern["action"].get<std::string>(),
              std::string("shutdown"));
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
    EXPECT_EQ(out.signature.tooling.tool_type,
              std::string("process_alarm"));
}

// ─── 4. Recognize via metadata replay ─────────────────────────────────────
TEST(SkillAlarmThresholdSet, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    auto out = skill::alarm_threshold_set::apply(*stock, goodInput());

    auto cands = skill::alarm_threshold_set::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["alarm_id"].get<std::string>(),
              std::string("ALM-TEMP-01"));
    EXPECT_NEAR(cands[0].recovered_params["low"].get<double>(),  60.0, 1e-9);
    EXPECT_NEAR(cands[0].recovered_params["high"].get<double>(), 95.0, 1e-9);

    // Strip metadata → recognize empty.
    skill::Workpiece raw(out.workpiece->shape());
    auto cands2 = skill::alarm_threshold_set::recognize(raw);
    EXPECT_TRUE(cands2.empty())
        << "alarm_threshold_set cannot be recognized geometrically";
}

// ─── 5. SPECIFIC: unknown action → info; known actions → no info ─────────
TEST(SkillAlarmThresholdSet, UnknownActionEmitsInfoOnly)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto in = goodInput();
    in.action = "nuke_the_reactor";   // not in the known set
    auto r = skill::alarm_threshold_set::validate(*stock, in);
    EXPECT_TRUE(r.passed)
        << "unknown action is INFO-level, not blocking";
    EXPECT_TRUE(hasFinding(r, "DFM-ALARM-ACTION"));
    EXPECT_NO_THROW(skill::alarm_threshold_set::apply(*stock, in));

    // Each known action does NOT emit DFM-ALARM-ACTION.
    for (const std::string a : { "alert", "shutdown", "interlock" }) {
        auto kIn = goodInput();
        kIn.action = a;
        auto kR = skill::alarm_threshold_set::validate(*stock, kIn);
        EXPECT_TRUE(kR.passed);
        EXPECT_FALSE(hasFinding(kR, "DFM-ALARM-ACTION"))
            << "known action '" << a << "' should NOT emit DFM-ALARM-ACTION";
    }
}
