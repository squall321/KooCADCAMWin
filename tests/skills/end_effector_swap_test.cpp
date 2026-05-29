// @lat: [[process/test-strategy#skill round-trip]]
//
// end_effector_swap skill — metadata-only sweep.  Covers:
//   1. apply clones geometry unchanged
//   2. validate rejects bad input (empty new_effector_id)
//   3. signature records kind + key params
//   4. recognize via metadata replay
//   5. skill-specific: same id triggers DFM-EOAT-SAME info finding

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/end_effector_swap.hpp"

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
TEST(SkillEndEffectorSwap, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(50.0, 30.0, 12.0, "aluminum_6061");
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    skill::end_effector_swap::Input in;
    auto out = skill::end_effector_swap::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. Validate rejects bad input ────────────────────────────────────────
TEST(SkillEndEffectorSwap, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(50.0, 30.0, 12.0, "aluminum_6061");

    skill::end_effector_swap::Input in;
    in.new_effector_id = "";       // invalid

    auto r = skill::end_effector_swap::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_THROW(skill::end_effector_swap::apply(*stock, in), skill::SkillError);
}

// ─── 3. Signature records kind + key params ──────────────────────────────
TEST(SkillEndEffectorSwap, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCuboidStock(50.0, 30.0, 12.0, "aluminum_6061");

    skill::end_effector_swap::Input in;
    in.old_effector_id       = "gripper_2f";
    in.new_effector_id       = "magnet_pad";
    in.calibration_offset_mm = 0.4;

    auto out = skill::end_effector_swap::apply(*stock, in);
    EXPECT_EQ(out.signature.skill_id, std::string("end_effector_swap"));
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("end_effector_swap"));
    EXPECT_EQ(out.signature.pattern["old_effector_id"].get<std::string>(),
              std::string("gripper_2f"));
    EXPECT_EQ(out.signature.pattern["new_effector_id"].get<std::string>(),
              std::string("magnet_pad"));
    EXPECT_NEAR(out.signature.pattern["calibration_offset_mm"].get<double>(),
                0.4, 1e-9);
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
}

// ─── 4. Recognize via metadata replay ─────────────────────────────────────
TEST(SkillEndEffectorSwap, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(50.0, 30.0, 12.0, "aluminum_6061");

    skill::end_effector_swap::Input in;
    in.old_effector_id       = "vacuum_pad_4x";
    in.new_effector_id       = "gripper_2f";
    in.calibration_offset_mm = -0.2;
    auto out = skill::end_effector_swap::apply(*stock, in);

    auto cands = skill::end_effector_swap::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["new_effector_id"].get<std::string>(),
              std::string("gripper_2f"));

    skill::Workpiece raw(out.workpiece->shape());
    auto cands2 = skill::end_effector_swap::recognize(raw);
    EXPECT_TRUE(cands2.empty());
}

// ─── 5. Skill-specific: same id triggers info finding ────────────────────
TEST(SkillEndEffectorSwap, SameIdTriggersInfoFinding)
{
    auto stock = skill::createCuboidStock(50.0, 30.0, 12.0, "aluminum_6061");

    skill::end_effector_swap::Input in;
    in.old_effector_id = "gripper_2f";
    in.new_effector_id = "gripper_2f";   // no-op swap

    auto r = skill::end_effector_swap::validate(*stock, in);
    EXPECT_TRUE(r.passed)
        << "no-op swap is info-only, must not block";
    EXPECT_TRUE(hasFinding(r, "DFM-EOAT-SAME"));
}
