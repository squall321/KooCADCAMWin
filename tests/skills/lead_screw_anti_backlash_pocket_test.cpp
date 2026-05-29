// @lat: [[process/test-strategy#compound macro]]
//
// lead_screw_anti_backlash_pocket — COMPOUND macro: 2 stacked half-nut
// pockets + spring slot + adj-screw pilot.  Tests:
//   1. apply produces valid shape with multiple sub-features.
//   2. validate rejects bad input.
//   3. signature records compound kind, subfeature_count >= 2.
//   4. recognize metadata replay (confidence 1.0).
//   5. stack_height == 2 × half_nut_l + spring_slot_w (geometric invariant).

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/lead_screw_anti_backlash_pocket.hpp"

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

// ─── 1. Apply produces valid shape with multiple sub-features ─────────────
TEST(SkillLeadScrewAntiBacklashPocket, ApplyProducesValidShapeWithMultipleSubFeatures)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 80.0);

    skill::lead_screw_anti_backlash_pocket::Input in;
    in.entry_face        = skill::FaceByNormal{ gp_Dir(0, 0, 1), 5.0, "largest" };
    in.position_x_mm     = 30.0;
    in.position_y_mm     = 30.0;
    in.axis_dir          = gp_Dir(0, 0, -1);
    in.half_nut_od_mm    = 20.0;
    in.half_nut_l_mm     = 15.0;
    in.spring_slot_w_mm  = 1.5;

    auto out = skill::lead_screw_anti_backlash_pocket::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    const double vBefore = volumeOf(stock->shape());
    const double vAfter  = volumeOf(out.workpiece->shape());
    EXPECT_LT(vAfter, vBefore);
    EXPECT_GT(out.workpiece->faceCount(), stock->faceCount());

    EXPECT_EQ(out.signature.pattern.at("subfeature_count").get<int>(), 4);
}

// ─── 2. Validate rejects bad input ────────────────────────────────────────
TEST(SkillLeadScrewAntiBacklashPocket, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 80.0);

    skill::lead_screw_anti_backlash_pocket::Input in;
    in.entry_face        = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.half_nut_od_mm    = 20.0;
    in.half_nut_l_mm     = 0.0;     // bad
    in.spring_slot_w_mm  = 1.5;

    auto r = skill::lead_screw_anti_backlash_pocket::validate(*stock, in);
    EXPECT_FALSE(r.passed);

    EXPECT_THROW(skill::lead_screw_anti_backlash_pocket::apply(*stock, in),
                 skill::SkillError);
}

// ─── 3. Signature records compound kind ───────────────────────────────────
TEST(SkillLeadScrewAntiBacklashPocket, SignatureRecordsCompoundKind)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 80.0);

    skill::lead_screw_anti_backlash_pocket::Input in;
    in.entry_face        = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm     = 30.0;
    in.position_y_mm     = 30.0;
    in.half_nut_od_mm    = 20.0;
    in.half_nut_l_mm     = 15.0;
    in.spring_slot_w_mm  = 1.5;

    auto out = skill::lead_screw_anti_backlash_pocket::apply(*stock, in);

    EXPECT_EQ(out.signature.skill_id, std::string("lead_screw_anti_backlash_pocket"));
    EXPECT_EQ(out.signature.pattern.at("kind").get<std::string>(),
              std::string("lead_screw_anti_backlash_pocket"));
    EXPECT_TRUE(out.signature.pattern.at("is_compound").get<bool>());
    EXPECT_GE(out.signature.pattern.at("subfeature_count").get<int>(), 2);
}

// ─── 4. Recognize metadata replay ─────────────────────────────────────────
TEST(SkillLeadScrewAntiBacklashPocket, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 80.0);

    skill::lead_screw_anti_backlash_pocket::Input in;
    in.entry_face        = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm     = 30.0;
    in.position_y_mm     = 30.0;
    in.half_nut_od_mm    = 20.0;
    in.half_nut_l_mm     = 15.0;
    in.spring_slot_w_mm  = 1.5;

    auto out = skill::lead_screw_anti_backlash_pocket::apply(*stock, in);

    auto cands = skill::lead_screw_anti_backlash_pocket::recognize(*out.workpiece);
    ASSERT_GE(cands.size(), 1u);
    EXPECT_DOUBLE_EQ(cands[0].confidence, 1.0);
    EXPECT_DOUBLE_EQ(cands[0].recovered_params.at("half_nut_l_mm").get<double>(), 15.0);
}

// ─── 5. stack_height = 2 × half_nut_l + spring_slot_w (geometric invariant)
TEST(SkillLeadScrewAntiBacklashPocket, StackHeightMatchesPocketsPlusSlot)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 80.0);

    skill::lead_screw_anti_backlash_pocket::Input in;
    in.entry_face        = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm     = 30.0;
    in.position_y_mm     = 30.0;
    in.half_nut_od_mm    = 20.0;
    in.half_nut_l_mm     = 12.0;
    in.spring_slot_w_mm  = 2.0;

    auto out = skill::lead_screw_anti_backlash_pocket::apply(*stock, in);

    const double stackH = out.signature.pattern.at("stack_height_mm").get<double>();
    const double halfL  = out.signature.pattern.at("half_nut_l_mm").get<double>();
    const double slotW  = out.signature.pattern.at("spring_slot_w_mm").get<double>();
    EXPECT_NEAR(stackH, 2.0 * halfL + slotW, 1e-9);
}
