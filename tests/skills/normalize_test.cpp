// @lat: [[process/test-strategy#skill round-trip]]
//
// normalize skill — heat-treatment no-op sweep.  Covers:
//   1. identity geometry on apply
//   2. signature pattern advertises air-cool + fine grain
//   3. DFM info on out-of-range temperature
//   4. material without normalize range (aluminum_6061) emits NA finding
//   5. recognize via metadata replay
//   6. DFM error on zero hold_time_min

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/normalize.hpp"

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
TEST(SkillNormalize, ApplyDoesNotChangeGeometry)
{
    auto stock = skill::createCuboidStock(35.0, 35.0, 10.0, "carbon_steel_1045");
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    skill::normalize::Input in;
    in.material                 = "carbon_steel_1045";
    in.normalize_temperature_c  = 870.0;     // mid of 845–900 range
    in.hold_time_min            = 30.0;

    auto out = skill::normalize::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("normalize"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
}

// ─── 2. Signature advertises air-cool + fine grain ───────────────────────
TEST(SkillNormalize, SignatureFlagsAirCoolFineGrain)
{
    auto stock = skill::createCuboidStock(35.0, 35.0, 10.0, "alloy_steel_4140");

    skill::normalize::Input in;
    in.material                 = "alloy_steel_4140";
    in.normalize_temperature_c  = 885.0;
    in.hold_time_min            = 30.0;

    auto out = skill::normalize::apply(*stock, in);
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("normalize"));
    EXPECT_EQ(out.signature.pattern["cooling_rate"].get<std::string>(),
              std::string("air"));
    EXPECT_EQ(out.signature.pattern["expected_grain_size"].get<std::string>(),
              std::string("fine"));
    EXPECT_EQ(out.signature.pattern["residual_stress"].get<std::string>(),
              std::string("low"));
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
}

// ─── 3. DFM info on out-of-range temperature ─────────────────────────────
TEST(SkillNormalize, OutOfRangeTempIsInfo)
{
    auto stock = skill::createCuboidStock(35.0, 35.0, 10.0, "carbon_steel_1045");

    skill::normalize::Input in;
    in.material                 = "carbon_steel_1045";
    in.normalize_temperature_c  = 700.0;       // < 845 min
    in.hold_time_min            = 30.0;

    auto r = skill::normalize::validate(*stock, in);
    EXPECT_TRUE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-HT-TEMP-LOW"));
    EXPECT_NO_THROW(skill::normalize::apply(*stock, in));
}

// ─── 4. Material without normalize range (Al 6061) emits NA ──────────────
TEST(SkillNormalize, NonApplicableMaterialEmitsNA)
{
    auto stock = skill::createCuboidStock(35.0, 35.0, 10.0, "aluminum_6061");

    skill::normalize::Input in;
    in.material                 = "aluminum_6061";
    in.normalize_temperature_c  = 400.0;
    in.hold_time_min            = 30.0;

    auto r = skill::normalize::validate(*stock, in);
    EXPECT_TRUE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-HT-NA"))
        << "aluminum has no normalize range — should emit DFM-HT-NA info";
    EXPECT_NO_THROW(skill::normalize::apply(*stock, in));
}

// ─── 5. Recognize via metadata replay ─────────────────────────────────────
TEST(SkillNormalize, RecognizeFromMetadata)
{
    auto stock = skill::createCuboidStock(35.0, 35.0, 10.0, "carbon_steel_1045");

    skill::normalize::Input in;
    in.material                 = "carbon_steel_1045";
    in.normalize_temperature_c  = 870.0;
    in.hold_time_min            = 30.0;
    auto out = skill::normalize::apply(*stock, in);

    auto cands = skill::normalize::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_NEAR(cands[0].recovered_params["normalize_temperature_c"].get<double>(),
                870.0, 1e-6);

    skill::Workpiece raw(out.workpiece->shape());
    auto cands2 = skill::normalize::recognize(raw);
    EXPECT_TRUE(cands2.empty());
}

// ─── 6. DFM error on zero hold_time_min ──────────────────────────────────
TEST(SkillNormalize, DfmErrorOnZeroHoldTime)
{
    auto stock = skill::createCuboidStock(35.0, 35.0, 10.0, "carbon_steel_1045");

    skill::normalize::Input in;
    in.material                 = "carbon_steel_1045";
    in.normalize_temperature_c  = 870.0;
    in.hold_time_min            = 0.0;

    auto r = skill::normalize::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_THROW(skill::normalize::apply(*stock, in), skill::SkillError);
}
