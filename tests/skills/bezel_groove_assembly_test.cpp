// @lat: [[process/test-strategy#compound bezel_groove_assembly]]
//
// bezel_groove_assembly — annular groove + chamfer + fillet compound.
//
// Test cases:
//   1. ApplyProducesValidShapeWithMultipleSubFeatures — volume + faces
//   2. ValidateRejectsBadInput — DIN 471 width-too-small
//   3. SignatureRecordsCompoundKind — pattern.is_compound, subfeature_count
//   4. RecognizeMetadataReplay — confidence 1.0 after apply
//   5. BezelInnerSmallerThanOuter — inner_dia < outer_dia assertion

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/bezel_groove_assembly.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using namespace koocadcam;

namespace {
double volumeOf(const TopoDS_Shape& s) {
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}
}  // namespace

// ─── 1. Apply produces valid shape with multiple sub-features ────────────
TEST(SkillBezelGroove, ApplyProducesValidShapeWithMultipleSubFeatures)
{
    auto stock = skill::createCylindricalStock(40.0, 12.0);

    skill::bezel_groove_assembly::Input in;
    in.case_face       = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.center_x_mm     = 0.0;
    in.center_y_mm     = 0.0;
    in.axis_dir        = gp_Dir(0, 0, -1);
    in.outer_dia_mm    = 32.0;
    in.inner_dia_mm    = 30.0;
    in.groove_depth_mm = 0.8;
    in.taper_deg       = 30.0;
    in.bottom_fillet_mm = 0.15;

    auto out = skill::bezel_groove_assembly::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    // Volume removed ≥ annular groove volume.
    const double rO = 16.0, rI = 15.0;
    const double annularVol = M_PI * (rO*rO - rI*rI) * 0.8;
    const double diff = volumeOf(stock->shape()) - volumeOf(out.workpiece->shape());
    EXPECT_GT(diff, annularVol * 0.6);   // primary + helper sub-features
    EXPECT_LT(diff, annularVol * 5.0);   // sanity

    // Face count must increase (groove introduces new walls).
    EXPECT_GT(out.workpiece->faceCount(), stock->faceCount());
}

// ─── 2. Validate rejects DIN 471 width-too-small groove ──────────────────
TEST(SkillBezelGroove, ValidateRejectsBadInput)
{
    auto stock = skill::createCylindricalStock(40.0, 12.0);

    skill::bezel_groove_assembly::Input in;
    in.case_face       = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.outer_dia_mm    = 30.2;
    in.inner_dia_mm    = 30.0;   // width = 0.1 mm — too narrow
    in.groove_depth_mm = 0.5;
    in.taper_deg       = 30.0;

    auto r = skill::bezel_groove_assembly::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-BEZEL-WIDTH") { found = true; break; }
    EXPECT_TRUE(found);

    EXPECT_THROW(skill::bezel_groove_assembly::apply(*stock, in),
                 skill::SkillError);
}

// ─── 3. Signature records compound kind ──────────────────────────────────
TEST(SkillBezelGroove, SignatureRecordsCompoundKind)
{
    auto stock = skill::createCylindricalStock(40.0, 12.0);

    skill::bezel_groove_assembly::Input in;
    in.case_face       = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.outer_dia_mm    = 32.0;
    in.inner_dia_mm    = 30.0;
    in.groove_depth_mm = 0.6;
    in.taper_deg       = 30.0;

    auto out = skill::bezel_groove_assembly::apply(*stock, in);

    EXPECT_EQ(out.signature.skill_id, std::string("bezel_groove_assembly"));
    EXPECT_EQ(out.signature.pattern.at("kind").get<std::string>(),
              std::string("bezel_groove_assembly"));
    EXPECT_TRUE(out.signature.pattern.at("is_compound").get<bool>());
    EXPECT_GE(out.signature.pattern.at("subfeature_count").get<int>(), 2);
    EXPECT_EQ(out.signature.pattern.at("subfeature_count").get<int>(), 4);
}

// ─── 4. Recognize via metadata replay returns confidence 1.0 ─────────────
TEST(SkillBezelGroove, RecognizeMetadataReplay)
{
    auto stock = skill::createCylindricalStock(40.0, 12.0);

    skill::bezel_groove_assembly::Input in;
    in.case_face       = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.outer_dia_mm    = 32.0;
    in.inner_dia_mm    = 30.0;
    in.groove_depth_mm = 0.8;
    in.taper_deg       = 30.0;

    auto out = skill::bezel_groove_assembly::apply(*stock, in);
    auto cands = skill::bezel_groove_assembly::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_DOUBLE_EQ(cands[0].confidence, 1.0);
    EXPECT_NEAR(cands[0].recovered_params.at("outer_dia_mm").get<double>(),
                32.0, 1e-6);
}

// ─── 5. Bezel groove inner Ø < outer Ø — geometric invariant ─────────────
TEST(SkillBezelGroove, BezelInnerSmallerThanOuter)
{
    auto stock = skill::createCylindricalStock(40.0, 12.0);

    skill::bezel_groove_assembly::Input in;
    in.case_face       = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.outer_dia_mm    = 32.0;
    in.inner_dia_mm    = 30.0;
    in.groove_depth_mm = 0.6;
    in.taper_deg       = 30.0;

    auto out = skill::bezel_groove_assembly::apply(*stock, in);
    const auto& pat = out.signature.pattern;

    const double outer = pat.at("outer_dia_mm").get<double>();
    const double inner = pat.at("inner_dia_mm").get<double>();
    EXPECT_LT(inner, outer);
    EXPECT_NEAR(pat.at("groove_width_mm").get<double>(),
                (outer - inner) / 2.0, 1e-6);
}
