// @lat: [[process/test-strategy#compound feature]]
//
// gas_spring_clevis_pocket — 3 sub-feature chain:
//   U-pocket + cross pin hole + retention grooves.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/gas_spring_clevis_pocket.hpp"

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
TEST(SkillGasSpringClevisPocket, ApplyProducesValidShapeWithMultipleSubFeatures)
{
    auto stock = skill::createCuboidStock(60.0, 40.0, 30.0);

    skill::gas_spring_clevis_pocket::Input in;
    in.face_id            = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm      = 30.0;
    in.position_y_mm      = 20.0;
    in.axis_dir           = gp_Dir(0, 0, -1);
    in.clevis_w           = 12.0;
    in.clevis_d           = 15.0;
    in.clevis_length      = 20.0;
    in.pin_dia            = 5.0;
    in.retention_groove_w = 1.0;

    const double v0 = volumeOf(stock->shape());
    const int    n0 = stock->faceCount();
    auto out = skill::gas_spring_clevis_pocket::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    const double v1 = volumeOf(out.workpiece->shape());
    EXPECT_GT(v0 - v1, 0.0);

    // 3 sub-features → significantly more faces than starting cuboid.
    EXPECT_GT(out.workpiece->faceCount(), n0);
}

// ─── 2. validate rejects bad input (pin too large) ────────────────────────
TEST(SkillGasSpringClevisPocket, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(60.0, 40.0, 30.0);

    skill::gas_spring_clevis_pocket::Input in;
    in.face_id            = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm      = 30.0;
    in.position_y_mm      = 20.0;
    in.clevis_w           = 12.0;
    in.clevis_d           = 15.0;
    in.clevis_length      = 20.0;
    in.pin_dia            = 10.0;     // > clevis_w / 2 = 6.0
    in.retention_groove_w = 1.0;

    auto r = skill::gas_spring_clevis_pocket::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool foundPin = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-CLEVIS-PIN") { foundPin = true; break; }
    EXPECT_TRUE(foundPin);

    EXPECT_THROW(skill::gas_spring_clevis_pocket::apply(*stock, in),
                 skill::SkillError);
}

// ─── 3. signature records is_compound + subfeature_count >= 2 ────────────
TEST(SkillGasSpringClevisPocket, SignatureRecordsCompoundKind)
{
    auto stock = skill::createCuboidStock(60.0, 40.0, 30.0);

    skill::gas_spring_clevis_pocket::Input in;
    in.face_id            = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm      = 30.0;
    in.position_y_mm      = 20.0;
    in.clevis_w           = 14.0;
    in.clevis_d           = 18.0;
    in.clevis_length      = 22.0;
    in.pin_dia            = 6.0;
    in.retention_groove_w = 1.0;

    auto out = skill::gas_spring_clevis_pocket::apply(*stock, in);

    EXPECT_EQ(out.signature.skill_id,
              std::string("gas_spring_clevis_pocket"));
    EXPECT_EQ(out.signature.pattern.at("kind"),
              std::string("gas_spring_clevis_pocket"));
    EXPECT_TRUE(out.signature.pattern.at("is_compound").get<bool>());
    EXPECT_GE(out.signature.pattern.at("subfeature_count").get<int>(), 2);
    EXPECT_EQ(out.signature.pattern.at("subfeature_count").get<int>(), 3);
    EXPECT_NEAR(out.signature.params.at("clevis_w").get<double>(), 14.0, 1e-9);
    EXPECT_NEAR(out.signature.params.at("pin_dia").get<double>(), 6.0, 1e-9);
}

// ─── 4. recognize() replays metadata at confidence 1.0 ───────────────────
TEST(SkillGasSpringClevisPocket, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(60.0, 40.0, 30.0);

    skill::gas_spring_clevis_pocket::Input in;
    in.face_id            = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm      = 30.0;
    in.position_y_mm      = 20.0;
    in.clevis_w           = 10.0;
    in.clevis_d           = 12.0;
    in.clevis_length      = 18.0;
    in.pin_dia            = 4.5;
    in.retention_groove_w = 1.0;

    auto out = skill::gas_spring_clevis_pocket::apply(*stock, in);
    auto cands = skill::gas_spring_clevis_pocket::recognize(*out.workpiece);

    ASSERT_EQ(cands.size(), 1u);
    EXPECT_EQ(cands[0].skill_id, std::string("gas_spring_clevis_pocket"));
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-9);
    EXPECT_NEAR(cands[0].recovered_params.at("clevis_w").get<double>(),
                10.0, 1e-9);
    EXPECT_NEAR(cands[0].recovered_params.at("pin_dia").get<double>(),
                4.5, 1e-9);
}

// ─── 5. specific: pin axis sits in middle of pocket (z = top - clevis_d/2) ─
TEST(SkillGasSpringClevisPocket, PinAxisSitsMidPocketDepth)
{
    auto stock = skill::createCuboidStock(60.0, 40.0, 30.0);
    const double topZ = 30.0;

    skill::gas_spring_clevis_pocket::Input in;
    in.face_id            = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm      = 30.0;
    in.position_y_mm      = 20.0;
    in.clevis_w           = 12.0;
    in.clevis_d           = 15.0;
    in.clevis_length      = 20.0;
    in.pin_dia            = 5.0;
    in.retention_groove_w = 1.0;

    auto out = skill::gas_spring_clevis_pocket::apply(*stock, in);
    const double pinZ = out.signature.pattern.at("pin_axis_z_mm").get<double>();
    // Pin axis = topZ - clevis_d / 2 = 30 - 7.5 = 22.5
    EXPECT_NEAR(pinZ, topZ - 15.0 / 2.0, 1e-9);

    // pin_dia must be < clevis_w / 2 (validation constraint)
    EXPECT_LT(out.signature.params.at("pin_dia").get<double>(),
              out.signature.params.at("clevis_w").get<double>() / 2.0);
}
