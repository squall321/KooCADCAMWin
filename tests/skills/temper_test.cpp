// @lat: [[process/test-strategy#skill round-trip]]
//
// temper skill — heat-treatment no-op sweep.  Covers:
//   1. identity geometry on apply
//   2. post_temper_hrc decreases monotonically with temperature
//   3. DFM info on temperature outside [150, 700] °C window
//   4. DFM error on zero hold_time_min
//   5. recognize via metadata replay; empty without metadata
//   6. non-temperable material (e.g. brass_360) emits info

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/temper.hpp"

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

}  // namespace

// ─── 1. Apply: geometry is COMPLETELY unchanged ──────────────────────────
TEST(SkillTemper, ApplyDoesNotChangeGeometry)
{
    auto stock = skill::createCuboidStock(30.0, 30.0, 10.0, "carbon_steel_1045");
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();

    skill::temper::Input in;
    in.material      = "carbon_steel_1045";
    in.temperature_c = 200.0;
    in.hold_time_min = 60.0;

    auto out = skill::temper::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("temper"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
}

// ─── 2. post_temper_hrc decreases as temperature rises ───────────────────
TEST(SkillTemper, PostTemperHrcMonotonicallyDecreasing)
{
    auto stock = skill::createCuboidStock(30.0, 30.0, 10.0, "carbon_steel_1045");

    skill::temper::Input low;
    low.material      = "carbon_steel_1045";
    low.temperature_c = 200.0;       // low-temper for tool-steel-like service
    low.hold_time_min = 60.0;
    auto outLow = skill::temper::apply(*stock, low);

    skill::temper::Input high = low;
    high.temperature_c = 600.0;       // high-temper for spring service
    auto outHigh = skill::temper::apply(*stock, high);

    const double hrcLow  = outLow.signature.pattern["post_temper_hrc"].get<double>();
    const double hrcHigh = outHigh.signature.pattern["post_temper_hrc"].get<double>();
    EXPECT_GT(hrcLow, hrcHigh)
        << "low-temper HRC " << hrcLow
        << " should exceed high-temper HRC " << hrcHigh;
    EXPECT_GT(hrcLow, 0.0);
    EXPECT_GT(hrcHigh, 0.0);
}

// ─── 3. DFM info on temperature outside [150, 700] °C window ─────────────
TEST(SkillTemper, OutOfWindowTemperatureEmitsInfo)
{
    auto stock = skill::createCuboidStock(30.0, 30.0, 10.0, "carbon_steel_1045");

    skill::temper::Input in;
    in.material      = "carbon_steel_1045";
    in.temperature_c = 100.0;       // < 150 °C — too low to be effective
    in.hold_time_min = 60.0;

    auto r = skill::temper::validate(*stock, in);
    EXPECT_TRUE(r.passed)
        << "out-of-window temperature is info-only, must not block";
    EXPECT_TRUE(hasFinding(r, "DFM-HT-TEMPER-RANGE"));
    EXPECT_NO_THROW(skill::temper::apply(*stock, in));
}

// ─── 4. DFM error on zero hold_time_min ──────────────────────────────────
TEST(SkillTemper, DfmErrorOnZeroHoldTime)
{
    auto stock = skill::createCuboidStock(30.0, 30.0, 10.0, "carbon_steel_1045");

    skill::temper::Input in;
    in.material      = "carbon_steel_1045";
    in.temperature_c = 200.0;
    in.hold_time_min = 0.0;

    auto r = skill::temper::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_THROW(skill::temper::apply(*stock, in), skill::SkillError);
}

// ─── 5. Recognize via metadata replay; empty without metadata ────────────
TEST(SkillTemper, RecognizeFromMetadata)
{
    auto stock = skill::createCuboidStock(30.0, 30.0, 10.0, "alloy_steel_4140");

    skill::temper::Input in;
    in.material      = "alloy_steel_4140";
    in.temperature_c = 425.0;
    in.hold_time_min = 60.0;
    auto out = skill::temper::apply(*stock, in);

    auto cands = skill::temper::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_NEAR(cands[0].recovered_params["temperature_c"].get<double>(),
                425.0, 1e-6);

    skill::Workpiece raw(out.workpiece->shape());
    auto cands2 = skill::temper::recognize(raw);
    EXPECT_TRUE(cands2.empty());
}

// ─── 6. Non-temperable material emits info — operation still proceeds ───
TEST(SkillTemper, NonTemperableMaterialEmitsInfo)
{
    auto stock = skill::createCuboidStock(30.0, 30.0, 10.0, "brass_360");

    skill::temper::Input in;
    in.material      = "brass_360";
    in.temperature_c = 300.0;        // within general window
    in.hold_time_min = 30.0;

    auto r = skill::temper::validate(*stock, in);
    EXPECT_TRUE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-HT-NOT-TEMPERABLE"));
    EXPECT_NO_THROW(skill::temper::apply(*stock, in));
}
