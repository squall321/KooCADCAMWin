// @lat: [[process/test-strategy#skill round-trip]]
//
// robot_calibrate skill — metadata-only sweep.  Covers:
//   1. apply clones geometry unchanged
//   2. validate rejects bad input (zero accuracy)
//   3. signature records kind + key params
//   4. recognize via metadata replay
//   5. skill-specific: repeatability > accuracy emits info finding

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/robot_calibrate.hpp"

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
TEST(SkillRobotCalibrate, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0, "aluminum_6061");
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    skill::robot_calibrate::Input in;
    auto out = skill::robot_calibrate::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. Validate rejects bad input ────────────────────────────────────────
TEST(SkillRobotCalibrate, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0, "aluminum_6061");

    skill::robot_calibrate::Input in;
    in.accuracy_mm = 0.0;       // invalid

    auto r = skill::robot_calibrate::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_THROW(skill::robot_calibrate::apply(*stock, in), skill::SkillError);
}

// ─── 3. Signature records kind + key params ──────────────────────────────
TEST(SkillRobotCalibrate, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0, "aluminum_6061");

    skill::robot_calibrate::Input in;
    in.robot_id         = "cell_B_robot_3";
    in.accuracy_mm      = 0.08;
    in.repeatability_mm = 0.02;

    auto out = skill::robot_calibrate::apply(*stock, in);
    EXPECT_EQ(out.signature.skill_id, std::string("robot_calibrate"));
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("robot_calibrate"));
    EXPECT_EQ(out.signature.pattern["robot_id"].get<std::string>(),
              std::string("cell_B_robot_3"));
    EXPECT_NEAR(out.signature.pattern["accuracy_mm"].get<double>(), 0.08, 1e-9);
    EXPECT_NEAR(out.signature.pattern["repeatability_mm"].get<double>(), 0.02, 1e-9);
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
}

// ─── 4. Recognize via metadata replay ─────────────────────────────────────
TEST(SkillRobotCalibrate, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0, "aluminum_6061");

    skill::robot_calibrate::Input in;
    in.robot_id         = "cell_C_robot_2";
    in.accuracy_mm      = 0.1;
    in.repeatability_mm = 0.05;
    auto out = skill::robot_calibrate::apply(*stock, in);

    auto cands = skill::robot_calibrate::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["robot_id"].get<std::string>(),
              std::string("cell_C_robot_2"));

    // Raw workpiece (no metadata) — recognize is empty.
    skill::Workpiece raw(out.workpiece->shape());
    auto cands2 = skill::robot_calibrate::recognize(raw);
    EXPECT_TRUE(cands2.empty());
}

// ─── 5. Skill-specific: repeatability > accuracy emits info finding ──────
TEST(SkillRobotCalibrate, RepeatabilityExceedsAccuracyEmitsInfo)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0, "aluminum_6061");

    skill::robot_calibrate::Input in;
    in.accuracy_mm      = 0.05;
    in.repeatability_mm = 0.20;        // > accuracy_mm — unusual

    auto r = skill::robot_calibrate::validate(*stock, in);
    EXPECT_TRUE(r.passed)
        << "info-level findings should not block";
    EXPECT_TRUE(hasFinding(r, "DFM-ROBOT-RANGE"));
}
