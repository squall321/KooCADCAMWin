// @lat: [[process/test-strategy#compound feature]]
//
// torsion_spring_anchor_compound — 4 sub-feature chain:
//   pivot bore + arm-clearance slot + 2 anchor pilots.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/torsion_spring_anchor_compound.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

#include <cmath>

using namespace koocadcam;

namespace {
double volumeOf(const TopoDS_Shape& s) {
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}
}  // namespace

// ─── 1. apply produces a valid shape with multiple sub-features ───────────
TEST(SkillTorsionSpringAnchor, ApplyProducesValidShapeWithMultipleSubFeatures)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 25.0);

    skill::torsion_spring_anchor_compound::Input in;
    in.face_id           = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm     = 30.0;
    in.position_y_mm     = 30.0;
    in.axis_dir          = gp_Dir(0, 0, -1);
    in.pivot_bore        = 8.0;
    in.pivot_depth_mm    = 15.0;
    in.arm_slot_width    = 2.0;
    in.arm_slot_length   = 12.0;
    in.anchor_angle_1_deg = 45.0;
    in.anchor_angle_2_deg = -45.0;
    in.anchor_radius_mm  = 10.0;

    const double v0 = volumeOf(stock->shape());
    const int    n0 = stock->faceCount();
    auto out = skill::torsion_spring_anchor_compound::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    const double v1 = volumeOf(out.workpiece->shape());

    EXPECT_GT(v0 - v1, 0.0);
    // 4 sub-features → significant face count increase.
    EXPECT_GT(out.workpiece->faceCount(), n0);
}

// ─── 2. validate rejects bad input (anchor too close to pivot) ────────────
TEST(SkillTorsionSpringAnchor, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 25.0);

    skill::torsion_spring_anchor_compound::Input in;
    in.face_id           = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm     = 30.0;
    in.position_y_mm     = 30.0;
    in.pivot_bore        = 10.0;
    in.pivot_depth_mm    = 15.0;
    in.arm_slot_width    = 2.0;
    in.arm_slot_length   = 12.0;
    in.anchor_angle_1_deg = 45.0;
    in.anchor_angle_2_deg = -45.0;
    in.anchor_radius_mm  = 3.0;       // < pivot_bore/2 + pilot/2 = 5 + 0.8 + 0.5 = 6.3

    auto r = skill::torsion_spring_anchor_compound::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool foundAnchor = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-TORSION-ANCHOR") { foundAnchor = true; break; }
    EXPECT_TRUE(foundAnchor);

    EXPECT_THROW(skill::torsion_spring_anchor_compound::apply(*stock, in),
                 skill::SkillError);
}

// ─── 3. signature records is_compound + subfeature_count >= 2 ────────────
TEST(SkillTorsionSpringAnchor, SignatureRecordsCompoundKind)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 25.0);

    skill::torsion_spring_anchor_compound::Input in;
    in.face_id           = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm     = 30.0;
    in.position_y_mm     = 30.0;
    in.pivot_bore        = 6.0;
    in.pivot_depth_mm    = 10.0;
    in.arm_slot_width    = 1.5;
    in.arm_slot_length   = 8.0;
    in.anchor_angle_1_deg = 30.0;
    in.anchor_angle_2_deg = -30.0;
    in.anchor_radius_mm  = 8.0;

    auto out = skill::torsion_spring_anchor_compound::apply(*stock, in);

    EXPECT_EQ(out.signature.skill_id,
              std::string("torsion_spring_anchor_compound"));
    EXPECT_EQ(out.signature.pattern.at("kind"),
              std::string("torsion_spring_anchor_compound"));
    EXPECT_TRUE(out.signature.pattern.at("is_compound").get<bool>());
    EXPECT_GE(out.signature.pattern.at("subfeature_count").get<int>(), 2);
    EXPECT_EQ(out.signature.pattern.at("subfeature_count").get<int>(), 4);
    EXPECT_NEAR(out.signature.params.at("pivot_bore").get<double>(), 6.0, 1e-9);
    EXPECT_NEAR(out.signature.params.at("anchor_radius_mm").get<double>(),
                8.0, 1e-9);
}

// ─── 4. recognize() replays metadata at confidence 1.0 ───────────────────
TEST(SkillTorsionSpringAnchor, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 25.0);

    skill::torsion_spring_anchor_compound::Input in;
    in.face_id           = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm     = 30.0;
    in.position_y_mm     = 30.0;
    in.pivot_bore        = 5.0;
    in.pivot_depth_mm    = 10.0;
    in.arm_slot_width    = 1.2;
    in.arm_slot_length   = 7.0;
    in.anchor_angle_1_deg = 35.0;
    in.anchor_angle_2_deg = -35.0;
    in.anchor_radius_mm  = 6.0;

    auto out = skill::torsion_spring_anchor_compound::apply(*stock, in);
    auto cands = skill::torsion_spring_anchor_compound::recognize(*out.workpiece);

    ASSERT_EQ(cands.size(), 1u);
    EXPECT_EQ(cands[0].skill_id,
              std::string("torsion_spring_anchor_compound"));
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-9);
    EXPECT_NEAR(cands[0].recovered_params.at("pivot_bore").get<double>(),
                5.0, 1e-9);
    EXPECT_NEAR(cands[0].recovered_params.at("anchor_radius_mm").get<double>(),
                6.0, 1e-9);
}

// ─── 5. specific: anchor pilot dia = 0.8 × arm_slot_width ────────────────
TEST(SkillTorsionSpringAnchor, AnchorPilotDiaIs80PctOfArmSlotWidth)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 25.0);

    skill::torsion_spring_anchor_compound::Input in;
    in.face_id           = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm     = 30.0;
    in.position_y_mm     = 30.0;
    in.pivot_bore        = 8.0;
    in.pivot_depth_mm    = 12.0;
    in.arm_slot_width    = 2.0;
    in.arm_slot_length   = 10.0;
    in.anchor_angle_1_deg = 45.0;
    in.anchor_angle_2_deg = -45.0;
    in.anchor_radius_mm  = 9.0;

    auto out = skill::torsion_spring_anchor_compound::apply(*stock, in);

    // anchor pilot = arm_slot_width × 0.8 = 1.6 mm
    EXPECT_NEAR(out.signature.pattern.at("anchor_pilot_dia_mm").get<double>(),
                2.0 * 0.8, 1e-9);
    EXPECT_EQ(out.signature.pattern.at("anchor_pilot_count").get<int>(), 2);

    // Verify anchor angles are recorded.
    EXPECT_NEAR(out.signature.pattern.at("anchor_angle_1_deg").get<double>(),
                45.0, 1e-9);
    EXPECT_NEAR(out.signature.pattern.at("anchor_angle_2_deg").get<double>(),
                -45.0, 1e-9);

    // Anchor pilots must clear pivot bore wall.
    const double pivotR = out.signature.params.at("pivot_bore").get<double>() / 2.0;
    const double pilotR = out.signature.pattern.at("anchor_pilot_dia_mm").get<double>() / 2.0;
    const double anchorR = out.signature.params.at("anchor_radius_mm").get<double>();
    EXPECT_GT(anchorR, pivotR + pilotR);
}
