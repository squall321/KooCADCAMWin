// @lat: [[process/test-strategy#skill round-trip]]
//
// sterilize — terminal sterilization metadata stamp.
// Cases (5):
//   1. ApplyClonesGeometryUnchanged
//   2. ValidateRejectsBadInput
//   3. SignatureRecordsKindAndParams
//   4. RecognizeMetadataReplay
//   5. ToolTypeMatchesMethod

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/sterilize.hpp"

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

// ─── 1. Apply clones geometry unchanged ───────────────────────────────────
TEST(SkillSterilize, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(20.0, 20.0, 5.0, "stainless_316");
    const double volBefore = volumeOf(stock->shape());
    const int faceBefore   = stock->faceCount();
    const int edgeBefore   = stock->edgeCount();

    skill::sterilize::Input in;
    in.method       = "autoclave";
    in.dose_or_temp = 121.0;
    in.exposure_min = 30.0;

    auto out = skill::sterilize::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. Validate rejects bad input ─────────────────────────────────────────
TEST(SkillSterilize, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(20.0, 20.0, 3.0);

    // Unknown method.
    skill::sterilize::Input unknown;
    unknown.method       = "microwave";
    unknown.dose_or_temp = 100.0;
    unknown.exposure_min = 10.0;
    {
        auto r = skill::sterilize::validate(*stock, unknown);
        EXPECT_FALSE(r.passed);
        EXPECT_TRUE(hasFinding(r, "DFM-INPUT"));
    }
    EXPECT_THROW(skill::sterilize::apply(*stock, unknown), skill::SkillError);

    // Zero exposure.
    skill::sterilize::Input zeroExpo;
    zeroExpo.method       = "autoclave";
    zeroExpo.dose_or_temp = 121.0;
    zeroExpo.exposure_min = 0.0;
    {
        auto r = skill::sterilize::validate(*stock, zeroExpo);
        EXPECT_FALSE(r.passed);
        EXPECT_TRUE(hasFinding(r, "DFM-STERILIZE-EXPO"));
    }

    // Negative dose.
    skill::sterilize::Input negDose;
    negDose.method       = "gamma";
    negDose.dose_or_temp = -5.0;
    negDose.exposure_min = 60.0;
    {
        auto r = skill::sterilize::validate(*stock, negDose);
        EXPECT_FALSE(r.passed);
        EXPECT_TRUE(hasFinding(r, "DFM-STERILIZE-DOSE"));
    }
}

// ─── 3. Signature records kind + key params ────────────────────────────────
TEST(SkillSterilize, SignatureRecordsKindAndParams)
{
    auto stock = skill::createCuboidStock(15.0, 15.0, 3.0);

    skill::sterilize::Input in;
    in.method       = "gamma";
    in.dose_or_temp = 25.0;
    in.exposure_min = 120.0;

    auto out = skill::sterilize::apply(*stock, in);

    EXPECT_EQ(out.signature.skill_id, std::string("sterilize"));
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("sterilize"));
    EXPECT_EQ(out.signature.pattern["method"].get<std::string>(),
              std::string("gamma"));
    EXPECT_NEAR(out.signature.pattern["dose_or_temp"].get<double>(),
                25.0, 1e-9);
    EXPECT_NEAR(out.signature.pattern["exposure_min"].get<double>(),
                120.0, 1e-9);
    EXPECT_FALSE(out.signature.pattern["geometric_change"].get<bool>());
    EXPECT_EQ(out.signature.pattern["sal_target"].get<std::string>(),
              std::string("10e-6"));
}

// ─── 4. Recognize metadata replay ──────────────────────────────────────────
TEST(SkillSterilize, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(15.0, 15.0, 3.0);

    skill::sterilize::Input in;
    in.method       = "eto";
    in.dose_or_temp = 600.0;
    in.exposure_min = 240.0;

    auto out   = skill::sterilize::apply(*stock, in);
    auto cands = skill::sterilize::recognize(*out.workpiece);

    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["method"].get<std::string>(),
              std::string("eto"));
    EXPECT_NEAR(cands[0].recovered_params["exposure_min"].get<double>(),
                240.0, 1e-9);

    // Stripped of metadata → recognize empty.
    skill::Workpiece raw(out.workpiece->shape());
    auto cands2 = skill::sterilize::recognize(raw);
    EXPECT_TRUE(cands2.empty());
}

// ─── 5. Specific: tool_type matches method ─────────────────────────────────
TEST(SkillSterilize, ToolTypeMatchesMethod)
{
    auto stock = skill::createCuboidStock(10.0, 10.0, 2.0);

    skill::sterilize::Input gamma;
    gamma.method       = "gamma";
    gamma.dose_or_temp = 25.0;
    gamma.exposure_min = 60.0;
    auto outG = skill::sterilize::apply(*stock, gamma);
    EXPECT_EQ(outG.signature.tooling.tool_type,
              std::string("gamma_irradiator"));

    skill::sterilize::Input autocl;
    autocl.method       = "autoclave";
    autocl.dose_or_temp = 134.0;
    autocl.exposure_min = 15.0;
    auto outA = skill::sterilize::apply(*stock, autocl);
    EXPECT_EQ(outA.signature.tooling.tool_type,
              std::string("autoclave_chamber"));
}
