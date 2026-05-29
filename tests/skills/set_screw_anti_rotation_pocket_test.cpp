// @lat: [[process/test-strategy#compound macro]]
//
// set_screw_anti_rotation_pocket — 3 sub-features:
//   tapped set-screw hole + perpendicular shaft cross-drill + entry chamfer.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/set_screw_anti_rotation_pocket.hpp"

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

// ─── 1. apply produces a valid shape with sub-features ────────────────
TEST(SkillSetScrewAntiRotationPocket, ApplyProducesValidShapeWithMultipleSubFeatures)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 30.0);
    const int faces0 = stock->faceCount();

    skill::set_screw_anti_rotation_pocket::Input in;
    in.entry_face          = skill::FaceByNormal{ gp_Dir(0, 0, 1), 5.0, "largest" };
    in.position_x_mm       = 30.0;
    in.position_y_mm       = 30.0;
    in.axis_dir            = gp_Dir(0, 0, -1);
    in.shaft_axis_dir      = gp_Dir(1, 0, 0);
    in.set_screw_M         = "M4";
    in.shaft_intersect_dia = 8.0;
    in.chamfer_size_mm     = 0.5;
    in.boss_thickness_mm   = 10.0;

    const double v0 = volumeOf(stock->shape());
    auto out = skill::set_screw_anti_rotation_pocket::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    const double v1 = volumeOf(out.workpiece->shape());

    EXPECT_GT(v0 - v1, 0.0);
    // Cross-drill + pilot + chamfer adds at least 2 cylinders + a chamfer
    // → face count must rise significantly beyond a 6-face box.
    EXPECT_GT(out.workpiece->faceCount(), faces0);
}

// ─── 2. validate rejects bad input ────────────────────────────────────
TEST(SkillSetScrewAntiRotationPocket, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 30.0);

    skill::set_screw_anti_rotation_pocket::Input in;
    in.entry_face          = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm       = 30.0;
    in.position_y_mm       = 30.0;
    in.set_screw_M         = "M4";
    in.shaft_intersect_dia = 2.0;     // < pilot (3.3 mm) → DFM-SS-GEOM error
    in.boss_thickness_mm   = 10.0;

    auto r = skill::set_screw_anti_rotation_pocket::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-SS-GEOM") { found = true; break; }
    EXPECT_TRUE(found);
    EXPECT_THROW(skill::set_screw_anti_rotation_pocket::apply(*stock, in),
                 skill::SkillError);
}

// ─── 3. signature records compound kind ───────────────────────────────
TEST(SkillSetScrewAntiRotationPocket, SignatureRecordsCompoundKind)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 30.0);

    skill::set_screw_anti_rotation_pocket::Input in;
    in.entry_face          = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm       = 30.0;
    in.position_y_mm       = 30.0;
    in.shaft_axis_dir      = gp_Dir(0, 1, 0);
    in.set_screw_M         = "M6";
    in.shaft_intersect_dia = 12.0;
    in.boss_thickness_mm   = 12.0;

    auto out = skill::set_screw_anti_rotation_pocket::apply(*stock, in);
    EXPECT_EQ(out.signature.skill_id,
              std::string("set_screw_anti_rotation_pocket"));
    EXPECT_EQ(out.signature.pattern.at("kind"),
              std::string("set_screw_anti_rotation_pocket"));
    EXPECT_TRUE(out.signature.pattern.at("is_compound").get<bool>());
    EXPECT_GE(out.signature.pattern.at("subfeature_count").get<int>(), 2);
    EXPECT_EQ(out.signature.pattern.at("subfeature_count").get<int>(), 3);
    EXPECT_NEAR(out.signature.pattern.at("pilot_dia_mm").get<double>(), 5.0, 1e-6);
}

// ─── 4. recognize via metadata replay ─────────────────────────────────
TEST(SkillSetScrewAntiRotationPocket, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 30.0);

    skill::set_screw_anti_rotation_pocket::Input in;
    in.entry_face          = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm       = 30.0;
    in.position_y_mm       = 30.0;
    in.shaft_axis_dir      = gp_Dir(1, 0, 0);
    in.set_screw_M         = "M5";
    in.shaft_intersect_dia = 10.0;
    in.boss_thickness_mm   = 10.0;

    auto out = skill::set_screw_anti_rotation_pocket::apply(*stock, in);
    auto cands = skill::set_screw_anti_rotation_pocket::recognize(*out.workpiece);

    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-9);
    EXPECT_EQ(cands[0].recovered_params["set_screw_M"], "M5");
}

// ─── 5. SPECIFIC: shaft_intersect_dia > set-screw pilot dia ───────────
TEST(SkillSetScrewAntiRotationPocket, ShaftDiaExceedsPilotDia)
{
    EXPECT_NEAR(skill::set_screw_anti_rotation_pocket
                    ::setScrewPilotDiameterFor("M3"), 2.5, 1e-9);
    EXPECT_NEAR(skill::set_screw_anti_rotation_pocket
                    ::setScrewPilotDiameterFor("M8"), 6.8, 1e-9);

    auto stock = skill::createCuboidStock(60.0, 60.0, 30.0);

    skill::set_screw_anti_rotation_pocket::Input in;
    in.entry_face          = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm       = 30.0;
    in.position_y_mm       = 30.0;
    in.shaft_axis_dir      = gp_Dir(1, 0, 0);
    in.set_screw_M         = "M3";       // pilot 2.5
    in.shaft_intersect_dia = 6.0;
    in.boss_thickness_mm   = 8.0;

    auto out = skill::set_screw_anti_rotation_pocket::apply(*stock, in);
    const double pilotDia = out.signature.pattern.at("pilot_dia_mm").get<double>();
    const double shaftDia = out.signature.pattern.at("shaft_intersect_dia").get<double>();
    EXPECT_GT(shaftDia, pilotDia);
}
