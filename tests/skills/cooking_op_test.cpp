// @lat: [[process/test-strategy#skill round-trip]]
//
// cooking_op — thermal cooking metadata stamp.  Metadata-only.
// Cases (5):
//   1. ApplyClonesGeometryUnchanged
//   2. ValidateRejectsBadInput
//   3. SignatureRecordsKindAndParams
//   4. RecognizeMetadataReplay
//   5. ToolTypeMatchesMethod

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/cooking_op.hpp"

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
TEST(SkillCookingOp, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(30.0, 30.0, 5.0);
    const double volBefore = volumeOf(stock->shape());
    const int faceBefore   = stock->faceCount();
    const int edgeBefore   = stock->edgeCount();

    skill::cooking_op::Input in;
    in.method                 = "bake";
    in.temp_c                 = 180.0;
    in.time_min               = 30.0;
    in.target_internal_temp_c = 75.0;

    auto out = skill::cooking_op::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. Validate rejects bad input ─────────────────────────────────────────
TEST(SkillCookingOp, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(20.0, 20.0, 3.0);

    // Unknown method.
    skill::cooking_op::Input badMethod;
    badMethod.method                 = "broil";
    badMethod.temp_c                 = 200.0;
    badMethod.time_min               = 20.0;
    badMethod.target_internal_temp_c = 70.0;
    {
        auto r = skill::cooking_op::validate(*stock, badMethod);
        EXPECT_FALSE(r.passed);
        EXPECT_TRUE(hasFinding(r, "DFM-INPUT"));
    }
    EXPECT_THROW(skill::cooking_op::apply(*stock, badMethod),
                 skill::SkillError);

    // Zero temp.
    skill::cooking_op::Input zeroTemp;
    zeroTemp.method                 = "bake";
    zeroTemp.temp_c                 = 0.0;
    zeroTemp.time_min               = 30.0;
    zeroTemp.target_internal_temp_c = 70.0;
    {
        auto r = skill::cooking_op::validate(*stock, zeroTemp);
        EXPECT_FALSE(r.passed);
        EXPECT_TRUE(hasFinding(r, "DFM-COOK-TEMP"));
    }

    // Internal too high.
    skill::cooking_op::Input badInt;
    badInt.method                 = "bake";
    badInt.temp_c                 = 180.0;
    badInt.time_min               = 30.0;
    badInt.target_internal_temp_c = 250.0;    // > 200
    {
        auto r = skill::cooking_op::validate(*stock, badInt);
        EXPECT_FALSE(r.passed);
        EXPECT_TRUE(hasFinding(r, "DFM-COOK-INTERNAL"));
    }
}

// ─── 3. Signature records kind + key params ────────────────────────────────
TEST(SkillCookingOp, SignatureRecordsKindAndParams)
{
    auto stock = skill::createCuboidStock(15.0, 15.0, 3.0);

    skill::cooking_op::Input in;
    in.method                 = "steam";
    in.temp_c                 = 100.0;
    in.time_min               = 12.0;
    in.target_internal_temp_c = 72.0;

    auto out = skill::cooking_op::apply(*stock, in);

    EXPECT_EQ(out.signature.skill_id, std::string("cooking_op"));
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("cooking_op"));
    EXPECT_EQ(out.signature.pattern["method"].get<std::string>(),
              std::string("steam"));
    EXPECT_NEAR(out.signature.pattern["temp_c"].get<double>(),
                100.0, 1e-9);
    EXPECT_NEAR(out.signature.pattern["time_min"].get<double>(),
                12.0, 1e-9);
    EXPECT_NEAR(
        out.signature.pattern["target_internal_temp_c"].get<double>(),
        72.0, 1e-9);
    EXPECT_FALSE(out.signature.pattern["geometric_change"].get<bool>());
}

// ─── 4. Recognize metadata replay ──────────────────────────────────────────
TEST(SkillCookingOp, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(15.0, 15.0, 3.0);

    skill::cooking_op::Input in;
    in.method                 = "fry";
    in.temp_c                 = 175.0;
    in.time_min               = 4.0;
    in.target_internal_temp_c = 74.0;

    auto out   = skill::cooking_op::apply(*stock, in);
    auto cands = skill::cooking_op::recognize(*out.workpiece);

    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["method"].get<std::string>(),
              std::string("fry"));
    EXPECT_NEAR(cands[0].recovered_params["temp_c"].get<double>(),
                175.0, 1e-9);

    // Stripped of metadata → recognize empty.
    skill::Workpiece raw(out.workpiece->shape());
    auto cands2 = skill::cooking_op::recognize(raw);
    EXPECT_TRUE(cands2.empty());
}

// ─── 5. Specific: tool_type matches method ─────────────────────────────────
TEST(SkillCookingOp, ToolTypeMatchesMethod)
{
    auto stock = skill::createCuboidStock(10.0, 10.0, 2.0);

    skill::cooking_op::Input retort;
    retort.method                 = "sterilize";
    retort.temp_c                 = 121.0;
    retort.time_min               = 20.0;
    retort.target_internal_temp_c = 121.0;
    auto outR = skill::cooking_op::apply(*stock, retort);
    EXPECT_EQ(outR.signature.tooling.tool_type,
              std::string("retort_sterilizer"));

    skill::cooking_op::Input fry;
    fry.method                 = "fry";
    fry.temp_c                 = 175.0;
    fry.time_min               = 3.0;
    fry.target_internal_temp_c = 70.0;
    auto outF = skill::cooking_op::apply(*stock, fry);
    EXPECT_EQ(outF.signature.tooling.tool_type,
              std::string("industrial_fryer"));
}
