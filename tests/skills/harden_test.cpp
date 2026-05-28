// @lat: [[process/test-strategy#skill round-trip]]
//
// harden skill — heat-treatment no-op sweep.  Covers:
//   1. identity geometry on apply
//   2. signature carries expected_hardness_hrc + martensite_pct + requires_temper
//   3. DFM info on water-quenching alloy_steel_4140
//   4. non-hardenable material (e.g. stainless_316) emits info
//   5. recognize via metadata replay
//   6. DFM error on zero austenitize_temperature_c

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/harden.hpp"

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
TEST(SkillHarden, ApplyDoesNotChangeGeometry)
{
    auto stock = skill::createCuboidStock(30.0, 30.0, 10.0, "carbon_steel_1045");
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    skill::harden::Input in;
    in.material                  = "carbon_steel_1045";
    in.austenitize_temperature_c = 845.0;     // mid of 820–870 range
    in.hold_time_min             = 30.0;
    in.quench_medium             = "oil";

    auto out = skill::harden::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("harden"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. Signature records HRC + martensite_pct + requires_temper ────────
TEST(SkillHarden, SignatureRecordsHardnessExpectations)
{
    auto stock = skill::createCuboidStock(30.0, 30.0, 10.0, "alloy_steel_4140");

    skill::harden::Input in;
    in.material                  = "alloy_steel_4140";
    in.austenitize_temperature_c = 850.0;
    in.hold_time_min             = 30.0;
    in.quench_medium             = "oil";

    auto out = skill::harden::apply(*stock, in);
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(), std::string("harden"));
    EXPECT_NEAR(out.signature.pattern["expected_hardness_hrc"].get<double>(), 58.0, 1e-6);
    EXPECT_NEAR(out.signature.pattern["martensite_pct"].get<double>(),       90.0, 1e-6);
    EXPECT_TRUE(out.signature.pattern["requires_temper"].get<bool>());
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
}

// ─── 3. DFM info on water-quench of alloy_steel_4140 ─────────────────────
TEST(SkillHarden, WaterQuenchOnChromolyEmitsCompat)
{
    auto stock = skill::createCuboidStock(30.0, 30.0, 10.0, "alloy_steel_4140");

    skill::harden::Input in;
    in.material                  = "alloy_steel_4140";
    in.austenitize_temperature_c = 850.0;
    in.hold_time_min             = 30.0;
    in.quench_medium             = "water";       // known antipattern

    auto r = skill::harden::validate(*stock, in);
    EXPECT_TRUE(r.passed)
        << "compat is info-level — should not block";
    EXPECT_TRUE(hasFinding(r, "DFM-HT-QUENCH"));
    EXPECT_NO_THROW(skill::harden::apply(*stock, in));
}

// ─── 4. Non-hardenable material emits info — does not block ──────────────
TEST(SkillHarden, NonHardenableMaterialEmitsInfo)
{
    auto stock = skill::createCuboidStock(30.0, 30.0, 10.0, "stainless_316");

    skill::harden::Input in;
    in.material                  = "stainless_316";
    in.austenitize_temperature_c = 1050.0;        // a plausible solution temp
    in.hold_time_min             = 30.0;
    in.quench_medium             = "water";

    auto r = skill::harden::validate(*stock, in);
    EXPECT_TRUE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-HT-NOT-HARDENABLE"));
    // Should also flag the temp-NA gate since austenitize range = 0/0.
    EXPECT_TRUE(hasFinding(r, "DFM-HT-NA"));
}

// ─── 5. Recognize via metadata replay; empty on featureless workpiece ───
TEST(SkillHarden, RecognizeFromMetadata)
{
    auto stock = skill::createCuboidStock(30.0, 30.0, 10.0, "carbon_steel_1045");

    skill::harden::Input in;
    in.material                  = "carbon_steel_1045";
    in.austenitize_temperature_c = 845.0;
    in.hold_time_min             = 30.0;
    in.quench_medium             = "oil";
    auto out = skill::harden::apply(*stock, in);

    auto cands = skill::harden::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["quench_medium"].get<std::string>(),
              std::string("oil"));

    skill::Workpiece raw(out.workpiece->shape());
    auto cands2 = skill::harden::recognize(raw);
    EXPECT_TRUE(cands2.empty());
}

// ─── 6. DFM error on zero austenitize_temperature_c ──────────────────────
TEST(SkillHarden, DfmErrorOnZeroAustenitizeTemp)
{
    auto stock = skill::createCuboidStock(30.0, 30.0, 10.0, "carbon_steel_1045");

    skill::harden::Input in;
    in.material                  = "carbon_steel_1045";
    in.austenitize_temperature_c = 0.0;       // invalid
    in.hold_time_min             = 30.0;

    auto r = skill::harden::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_THROW(skill::harden::apply(*stock, in), skill::SkillError);
}
