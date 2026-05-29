// @lat: [[process/test-strategy#skill round-trip]]
//
// freezing_op — rapid food freezing metadata stamp.  Metadata-only.
// Cases (5):
//   1. ApplyClonesGeometryUnchanged
//   2. ValidateRejectsBadInput
//   3. SignatureRecordsKindAndParams
//   4. RecognizeMetadataReplay
//   5. SlowFreezingEmitsInfo

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/freezing_op.hpp"

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
TEST(SkillFreezingOp, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(30.0, 30.0, 5.0);
    const double volBefore = volumeOf(stock->shape());
    const int faceBefore   = stock->faceCount();
    const int edgeBefore   = stock->edgeCount();

    skill::freezing_op::Input in;
    in.final_temp_c      = -25.0;
    in.freezing_time_min = 30.0;

    auto out = skill::freezing_op::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. Validate rejects bad input ─────────────────────────────────────────
TEST(SkillFreezingOp, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(20.0, 20.0, 3.0);

    // Non-negative temperature.
    skill::freezing_op::Input warm;
    warm.final_temp_c      = 5.0;
    warm.freezing_time_min = 30.0;
    {
        auto r = skill::freezing_op::validate(*stock, warm);
        EXPECT_FALSE(r.passed);
        EXPECT_TRUE(hasFinding(r, "DFM-INPUT"));
    }
    EXPECT_THROW(skill::freezing_op::apply(*stock, warm), skill::SkillError);

    // Zero time.
    skill::freezing_op::Input zeroT;
    zeroT.final_temp_c      = -20.0;
    zeroT.freezing_time_min = 0.0;
    {
        auto r = skill::freezing_op::validate(*stock, zeroT);
        EXPECT_FALSE(r.passed);
        EXPECT_TRUE(hasFinding(r, "DFM-FREEZE-TIME"));
    }

    // Time out of range high.
    skill::freezing_op::Input tooLong;
    tooLong.final_temp_c      = -20.0;
    tooLong.freezing_time_min = 1000.0;   // > 720
    {
        auto r = skill::freezing_op::validate(*stock, tooLong);
        EXPECT_FALSE(r.passed);
        EXPECT_TRUE(hasFinding(r, "DFM-FREEZE-TIME"));
    }
}

// ─── 3. Signature records kind + key params ────────────────────────────────
TEST(SkillFreezingOp, SignatureRecordsKindAndParams)
{
    auto stock = skill::createCuboidStock(15.0, 15.0, 3.0);

    skill::freezing_op::Input in;
    in.final_temp_c      = -35.0;
    in.freezing_time_min = 18.0;

    auto out = skill::freezing_op::apply(*stock, in);

    EXPECT_EQ(out.signature.skill_id, std::string("freezing_op"));
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("freezing_op"));
    EXPECT_NEAR(out.signature.pattern["final_temp_c"].get<double>(),
                -35.0, 1e-9);
    EXPECT_NEAR(out.signature.pattern["freezing_time_min"].get<double>(),
                18.0, 1e-9);
    EXPECT_FALSE(out.signature.pattern["geometric_change"].get<bool>());
    EXPECT_EQ(out.signature.pattern["process_family"].get<std::string>(),
              std::string("food_processing"));
}

// ─── 4. Recognize metadata replay ──────────────────────────────────────────
TEST(SkillFreezingOp, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(15.0, 15.0, 3.0);

    skill::freezing_op::Input in;
    in.final_temp_c      = -40.0;
    in.freezing_time_min = 12.0;

    auto out   = skill::freezing_op::apply(*stock, in);
    auto cands = skill::freezing_op::recognize(*out.workpiece);

    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_NEAR(cands[0].recovered_params["final_temp_c"].get<double>(),
                -40.0, 1e-9);
    EXPECT_NEAR(
        cands[0].recovered_params["freezing_time_min"].get<double>(),
        12.0, 1e-9);

    // Stripped of metadata → recognize empty.
    skill::Workpiece raw(out.workpiece->shape());
    auto cands2 = skill::freezing_op::recognize(raw);
    EXPECT_TRUE(cands2.empty());
}

// ─── 5. Specific: slow freezing emits info, still proceeds ─────────────────
TEST(SkillFreezingOp, SlowFreezingEmitsInfo)
{
    auto stock = skill::createCuboidStock(10.0, 10.0, 2.0);

    skill::freezing_op::Input in;
    in.final_temp_c      = -18.0;
    in.freezing_time_min = 360.0;    // > 240 → info

    auto r = skill::freezing_op::validate(*stock, in);
    EXPECT_TRUE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-FREEZE-SLOW"));

    auto out = skill::freezing_op::apply(*stock, in);
    EXPECT_EQ(out.signature.tooling.tool_type,
              std::string("iqf_freezing_tunnel"));
    EXPECT_NEAR(out.signature.tooling.est_cycle_time_s,
                360.0 * 60.0, 1e-6);
}
