// @lat: [[process/test-strategy#skill round-trip]]
//
// payload_qualify skill — metadata-only sweep.  Covers:
//   1. apply clones geometry unchanged
//   2. validate rejects bad input (zero tested_mass_kg)
//   3. signature records kind + key params
//   4. recognize via metadata replay
//   5. skill-specific: brake_test_passed=false blocks production

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/payload_qualify.hpp"

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

// ─── 1. Apply: geometry COMPLETELY unchanged ──────────────────────────────
TEST(SkillPayloadQualify, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(60.0, 40.0, 20.0, "carbon_steel_1045");
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    skill::payload_qualify::Input in;
    auto out = skill::payload_qualify::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. Validate rejects bad input ────────────────────────────────────────
TEST(SkillPayloadQualify, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(60.0, 40.0, 20.0, "carbon_steel_1045");

    skill::payload_qualify::Input in;
    in.tested_mass_kg = 0.0;       // invalid

    auto r = skill::payload_qualify::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_THROW(skill::payload_qualify::apply(*stock, in), skill::SkillError);
}

// ─── 3. Signature records kind + key params ──────────────────────────────
TEST(SkillPayloadQualify, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCuboidStock(60.0, 40.0, 20.0, "carbon_steel_1045");

    skill::payload_qualify::Input in;
    in.tested_mass_kg    = 15.5;
    in.max_speed_pct     = 85.0;
    in.brake_test_passed = true;

    auto out = skill::payload_qualify::apply(*stock, in);
    EXPECT_EQ(out.signature.skill_id, std::string("payload_qualify"));
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("payload_qualify"));
    EXPECT_NEAR(out.signature.pattern["tested_mass_kg"].get<double>(),
                15.5, 1e-9);
    EXPECT_NEAR(out.signature.pattern["max_speed_pct"].get<double>(),
                85.0, 1e-9);
    EXPECT_TRUE(out.signature.pattern["brake_test_passed"].get<bool>());
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
}

// ─── 4. Recognize via metadata replay ─────────────────────────────────────
TEST(SkillPayloadQualify, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(60.0, 40.0, 20.0, "carbon_steel_1045");

    skill::payload_qualify::Input in;
    in.tested_mass_kg    = 8.0;
    in.max_speed_pct     = 50.0;
    in.brake_test_passed = true;
    auto out = skill::payload_qualify::apply(*stock, in);

    auto cands = skill::payload_qualify::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_NEAR(cands[0].recovered_params["tested_mass_kg"].get<double>(),
                8.0, 1e-9);

    skill::Workpiece raw(out.workpiece->shape());
    auto cands2 = skill::payload_qualify::recognize(raw);
    EXPECT_TRUE(cands2.empty());
}

// ─── 5. Skill-specific: brake_test_passed=false blocks production ────────
TEST(SkillPayloadQualify, FailedBrakeTestBlocksProduction)
{
    auto stock = skill::createCuboidStock(60.0, 40.0, 20.0, "carbon_steel_1045");

    skill::payload_qualify::Input in;
    in.tested_mass_kg    = 10.0;
    in.max_speed_pct     = 100.0;
    in.brake_test_passed = false;       // safety violation

    auto r = skill::payload_qualify::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-PAYLOAD-BRAKE"));
    EXPECT_THROW(skill::payload_qualify::apply(*stock, in), skill::SkillError);
}
