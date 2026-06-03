// @lat: [[process/test-strategy#compound crown_guard_shoulder_compound]]

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/crown_guard_shoulder_compound.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

#include <gp_Ax1.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>

#include <cmath>

using namespace koocadcam;

namespace {
double volumeOf(const TopoDS_Shape& s) {
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}

skill::crown_guard_shoulder_compound::Input goodInput() {
    skill::crown_guard_shoulder_compound::Input in;
    in.case_side_face         = skill::FaceByNormal{ gp_Dir(1, 0, 0) };
    in.case_axis              = gp_Ax1(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1));
    in.case_outer_radius_mm   = 20.0;
    in.guard_angle_deg        = 0.0;     // 3 o'clock side (= +X)
    in.guard_height_mm        = 4.0;
    in.guard_width_mm         = 6.0;
    in.guard_thickness_mm     = 2.0;
    in.crown_clearance_dia_mm = 2.5;
    in.crown_z_mm             = 4.0;     // mid-height
    return in;
}
}  // namespace

// ─── 1. Apply adds boss material and cuts clearance hole ─────────────────
TEST(SkillCrownGuardShoulder, ApplyAddsBossAndDrillsHole)
{
    auto stock = skill::createCylindricalStock(40.0, 8.0);

    auto in = goodInput();
    auto out = skill::crown_guard_shoulder_compound::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    // Net volume change should be positive (boss > hole), since boss is
    // larger than the clearance hole.
    const double bossVol = in.guard_thickness_mm * in.guard_width_mm *
                           in.guard_height_mm;
    const double v1 = volumeOf(stock->shape());
    const double v2 = volumeOf(out.workpiece->shape());
    EXPECT_GT(v2, v1);
    EXPECT_LT(v2 - v1, bossVol * 1.2);
    EXPECT_GT(out.workpiece->faceCount(), stock->faceCount());
}

// ─── 2. Validate rejects too-small clearance ─────────────────────────────
TEST(SkillCrownGuardShoulder, ValidateRejectsTinyClearance)
{
    auto stock = skill::createCylindricalStock(40.0, 8.0);

    auto in = goodInput();
    in.crown_clearance_dia_mm = 0.5;  // < 1.0

    auto r = skill::crown_guard_shoulder_compound::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-CROWN-CLEAR") { found = true; break; }
    EXPECT_TRUE(found);

    EXPECT_THROW(skill::crown_guard_shoulder_compound::apply(*stock, in),
                 skill::SkillError);
}

// ─── 3. Validate rejects too-thin guard wall ─────────────────────────────
TEST(SkillCrownGuardShoulder, ValidateRejectsThinGuard)
{
    auto stock = skill::createCylindricalStock(40.0, 8.0);

    auto in = goodInput();
    in.guard_thickness_mm = 0.3;   // < 0.5 mm

    auto r = skill::crown_guard_shoulder_compound::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-GUARD-THICK") { found = true; break; }
    EXPECT_TRUE(found);
}

// ─── 4. Signature records compound + crown shoulder type ─────────────────
TEST(SkillCrownGuardShoulder, SignatureRecordsCompoundShoulder)
{
    auto stock = skill::createCylindricalStock(40.0, 8.0);

    auto in = goodInput();
    auto out = skill::crown_guard_shoulder_compound::apply(*stock, in);

    EXPECT_EQ(out.signature.skill_id,
              std::string("crown_guard_shoulder_compound"));
    EXPECT_TRUE(out.signature.pattern.at("is_compound").get<bool>());
    EXPECT_TRUE(out.signature.pattern.at("is_watch_feature").get<bool>());
    EXPECT_EQ(out.signature.pattern.at("watch_feature_type").get<std::string>(),
              std::string("crown_shoulder_with_clearance"));
    EXPECT_TRUE(out.signature.pattern.at("boss_with_through_hole").get<bool>());
}

// ─── 5. Recognize via metadata replay ────────────────────────────────────
TEST(SkillCrownGuardShoulder, RecognizeMetadataReplay)
{
    auto stock = skill::createCylindricalStock(40.0, 8.0);

    auto in = goodInput();
    auto out = skill::crown_guard_shoulder_compound::apply(*stock, in);

    auto cands = skill::crown_guard_shoulder_compound::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_DOUBLE_EQ(cands[0].confidence, 1.0);
    EXPECT_NEAR(cands[0].recovered_params.at("guard_thickness_mm").get<double>(),
                2.0, 1e-6);
}
