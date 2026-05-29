// @lat: [[process/test-strategy#compound lug_with_spring_bar_holes]]
//
// lug_with_spring_bar_holes — additive lug + 2 coaxial spring-bar holes.
//
// Test cases:
//   1. ApplyProducesValidShapeWithMultipleSubFeatures
//   2. ValidateRejectsBadInput — non-catalog spring bar Ø
//   3. SignatureRecordsCompoundKind
//   4. RecognizeMetadataReplay
//   5. SpringBarStandardCatalogMatch — spring_bar_dia matches Bergeon table

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/lug_with_spring_bar_holes.hpp"

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

skill::lug_with_spring_bar_holes::Input goodInput() {
    skill::lug_with_spring_bar_holes::Input in;
    in.case_axis            = gp_Ax1(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1));
    in.case_outer_radius_mm = 20.0;
    in.lug_angle_deg        = 0.0;
    in.lug_attach_z_mm      = 5.0;
    in.lug_length_mm        = 8.0;
    in.lug_width_mm         = 5.0;
    in.lug_thickness_mm     = 2.5;
    in.corner_chamfer_mm    = 0.4;
    in.spring_bar_dia_mm    = 1.5;
    in.spring_bar_inset_mm  = 1.5;
    return in;
}
}  // namespace

// ─── 1. Apply produces valid shape with multiple sub-features ────────────
TEST(SkillLugSpringBar, ApplyProducesValidShapeWithMultipleSubFeatures)
{
    // Use a cylindrical case stock big enough for the lug to attach to.
    auto stock = skill::createCylindricalStock(40.0, 10.0);

    auto in = goodInput();
    auto out = skill::lug_with_spring_bar_holes::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    // Net volume = lug box fused on − two spring-bar through-holes.
    // Lug fuse is purely additive on the case OD; bar holes subtract.
    // We expect net > 0 (lug volume dwarfs hole volume).
    const double netDiff = volumeOf(out.workpiece->shape()) - volumeOf(stock->shape());
    EXPECT_GT(netDiff, 0.0);

    // Face count must increase (lug adds new faces + 2 hole cylinder walls).
    EXPECT_GT(out.workpiece->faceCount(), stock->faceCount());
}

// ─── 2. Validate rejects non-catalog spring bar ──────────────────────────
TEST(SkillLugSpringBar, ValidateRejectsBadInput)
{
    auto stock = skill::createCylindricalStock(40.0, 10.0);

    auto in = goodInput();
    in.spring_bar_dia_mm = 1.0;   // not in 1.5/1.6/1.8 catalog

    auto r = skill::lug_with_spring_bar_holes::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-SPRINGBAR-SIZE") { found = true; break; }
    EXPECT_TRUE(found);

    EXPECT_THROW(skill::lug_with_spring_bar_holes::apply(*stock, in),
                 skill::SkillError);
}

// ─── 3. Signature records compound kind ──────────────────────────────────
TEST(SkillLugSpringBar, SignatureRecordsCompoundKind)
{
    auto stock = skill::createCylindricalStock(40.0, 10.0);

    auto in = goodInput();
    auto out = skill::lug_with_spring_bar_holes::apply(*stock, in);

    EXPECT_EQ(out.signature.skill_id, std::string("lug_with_spring_bar_holes"));
    EXPECT_TRUE(out.signature.pattern.at("is_compound").get<bool>());
    EXPECT_GE(out.signature.pattern.at("subfeature_count").get<int>(), 2);
    EXPECT_EQ(out.signature.pattern.at("hole_count").get<int>(), 2);
}

// ─── 4. Recognize via metadata replay ────────────────────────────────────
TEST(SkillLugSpringBar, RecognizeMetadataReplay)
{
    auto stock = skill::createCylindricalStock(40.0, 10.0);

    auto in = goodInput();
    auto out = skill::lug_with_spring_bar_holes::apply(*stock, in);

    auto cands = skill::lug_with_spring_bar_holes::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_DOUBLE_EQ(cands[0].confidence, 1.0);
    EXPECT_NEAR(cands[0].recovered_params.at("spring_bar_dia_mm").get<double>(),
                1.5, 1e-6);
    EXPECT_NEAR(cands[0].recovered_params.at("lug_length_mm").get<double>(),
                8.0, 1e-6);
}

// ─── 5. Spring bar Ø matches Bergeon catalog ─────────────────────────────
TEST(SkillLugSpringBar, SpringBarStandardCatalogMatch)
{
    auto stock = skill::createCylindricalStock(40.0, 10.0);

    // Try each catalog value: 1.5, 1.6, 1.8 — all must apply successfully.
    for (double dia : { 1.5, 1.6, 1.8 }) {
        auto in = goodInput();
        in.spring_bar_dia_mm = dia;
        in.lug_length_mm     = std::max(8.0, 4.0 * dia + 0.2);
        in.lug_width_mm      = std::max(5.0, 2.0 * dia + 0.2);
        in.spring_bar_inset_mm = std::max(1.5, dia + 0.2);
        auto r = skill::lug_with_spring_bar_holes::validate(*stock, in);
        EXPECT_TRUE(r.passed) << "dia = " << dia;
    }

    // Non-catalog 2.0 mm must fail.
    auto in = goodInput();
    in.spring_bar_dia_mm = 2.0;
    auto r = skill::lug_with_spring_bar_holes::validate(*stock, in);
    EXPECT_FALSE(r.passed);
}
