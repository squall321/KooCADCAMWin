// @lat: [[process/test-strategy#skill round-trip]]
//
// safety_assess_robot skill — metadata-only sweep.  Covers:
//   1. apply clones geometry unchanged
//   2. validate rejects bad input (invalid risk_class)
//   3. signature records kind + key params
//   4. recognize via metadata replay
//   5. skill-specific: PLc (below PLd) is DFM-SAFETY-CAT error

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/safety_assess_robot.hpp"

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
TEST(SkillSafetyAssessRobot, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(45.0, 35.0, 18.0, "carbon_steel_1045");
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    skill::safety_assess_robot::Input in;
    auto out = skill::safety_assess_robot::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. Validate rejects bad input ────────────────────────────────────────
TEST(SkillSafetyAssessRobot, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(45.0, 35.0, 18.0, "carbon_steel_1045");

    skill::safety_assess_robot::Input in;
    in.risk_class = "extreme";        // invalid — must be low/medium/high

    auto r = skill::safety_assess_robot::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_THROW(skill::safety_assess_robot::apply(*stock, in), skill::SkillError);
}

// ─── 3. Signature records kind + key params ──────────────────────────────
TEST(SkillSafetyAssessRobot, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCuboidStock(45.0, 35.0, 18.0, "carbon_steel_1045");

    skill::safety_assess_robot::Input in;
    in.safety_category = "PLe";
    in.risk_class      = "high";

    auto out = skill::safety_assess_robot::apply(*stock, in);
    EXPECT_EQ(out.signature.skill_id, std::string("safety_assess_robot"));
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("safety_assess_robot"));
    EXPECT_EQ(out.signature.pattern["safety_category"].get<std::string>(),
              std::string("PLe"));
    EXPECT_EQ(out.signature.pattern["risk_class"].get<std::string>(),
              std::string("high"));
    EXPECT_EQ(out.signature.pattern["standard"].get<std::string>(),
              std::string("ISO_13849-1"));
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
}

// ─── 4. Recognize via metadata replay ─────────────────────────────────────
TEST(SkillSafetyAssessRobot, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(45.0, 35.0, 18.0, "carbon_steel_1045");

    skill::safety_assess_robot::Input in;
    in.safety_category = "PLd";
    in.risk_class      = "medium";
    auto out = skill::safety_assess_robot::apply(*stock, in);

    auto cands = skill::safety_assess_robot::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["safety_category"].get<std::string>(),
              std::string("PLd"));

    skill::Workpiece raw(out.workpiece->shape());
    auto cands2 = skill::safety_assess_robot::recognize(raw);
    EXPECT_TRUE(cands2.empty());
}

// ─── 5. Skill-specific: PLc (below PLd) is DFM-SAFETY-CAT error ──────────
TEST(SkillSafetyAssessRobot, BelowPLdIsError)
{
    auto stock = skill::createCuboidStock(45.0, 35.0, 18.0, "carbon_steel_1045");

    skill::safety_assess_robot::Input in;
    in.safety_category = "PLc";       // below collaborative-robot minimum
    in.risk_class      = "medium";

    auto r = skill::safety_assess_robot::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-SAFETY-CAT"));
    EXPECT_THROW(skill::safety_assess_robot::apply(*stock, in), skill::SkillError);
}
