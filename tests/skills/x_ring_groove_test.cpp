// @lat: [[process/test-strategy#compound macro]]
//
// x_ring_groove — compound: center groove + 2 sidewall chamfers (10°).
//
// Tests (5):
//   1. ApplyProducesValidShapeWithMultipleSubFeatures.
//   2. ValidateRejectsBadInput — unknown Q-code + insufficient wall.
//   3. SignatureRecordsCompoundKind.
//   4. RecognizeMetadataReplay.
//   5. GrooveWiderThanORingGroove — W_x > W_o for same CS (per Parker handbook).

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/x_ring_groove.hpp"

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
TEST(SkillXRingGroove, ApplyProducesValidShapeWithMultipleSubFeatures)
{
    auto stock = skill::createCylindricalStock(40.0, 30.0);

    skill::x_ring_groove::Input in;
    in.bore_or_shaft_dia_mm = 40.0;
    in.mean_dia_mm          = 35.0;
    in.position_z_mm        = 15.0;
    in.x_ring_size          = "Q3";    // CS 3.53

    const int facesBefore = stock->faceCount();
    auto out = skill::x_ring_groove::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    const int facesAfter = out.workpiece->faceCount();
    EXPECT_GT(facesAfter, facesBefore);

    const double cs      = 3.53;
    const double depth   = 0.78 * cs;
    const double width   = 1.70 * cs;
    const double shaftR  = 20.0;
    const double grooveR = shaftR - depth;
    const double rectVol = M_PI * (shaftR * shaftR - grooveR * grooveR) * width;
    const double removed = volumeOf(stock->shape()) - volumeOf(out.workpiece->shape());
    EXPECT_GT(removed, 0.85 * rectVol);
    EXPECT_LT(removed, 1.5  * rectVol);
}

// ─── 2. Validate rejects bad input ─────────────────────────────────────────
TEST(SkillXRingGroove, ValidateRejectsBadInput)
{
    auto stock = skill::createCylindricalStock(40.0, 30.0);

    skill::x_ring_groove::Input bad;
    bad.bore_or_shaft_dia_mm = 40.0;
    bad.mean_dia_mm          = 35.0;
    bad.position_z_mm        = 15.0;
    bad.x_ring_size          = "ZZ";    // unknown

    auto r1 = skill::x_ring_groove::validate(*stock, bad);
    EXPECT_FALSE(r1.passed);
    bool foundXR = false;
    for (const auto& f : r1.findings)
        if (f.code == "DFM-XRING") { foundXR = true; break; }
    EXPECT_TRUE(foundXR);

    EXPECT_THROW(skill::x_ring_groove::apply(*stock, bad), skill::SkillError);

    // Insufficient wall: large CS on a tiny shaft.
    auto tinyStock = skill::createCylindricalStock(4.0, 20.0);
    skill::x_ring_groove::Input thin;
    thin.bore_or_shaft_dia_mm = 4.0;
    thin.mean_dia_mm          = 3.5;
    thin.position_z_mm        = 10.0;
    thin.x_ring_size          = "Q4";   // CS 5.33 → depth ≈ 4.16 → wall negative
    auto r2 = skill::x_ring_groove::validate(*tinyStock, thin);
    EXPECT_FALSE(r2.passed);
}

// ─── 3. Signature records compound kind ────────────────────────────────────
TEST(SkillXRingGroove, SignatureRecordsCompoundKind)
{
    auto stock = skill::createCylindricalStock(40.0, 30.0);

    skill::x_ring_groove::Input in;
    in.bore_or_shaft_dia_mm = 40.0;
    in.mean_dia_mm          = 35.0;
    in.position_z_mm        = 15.0;
    in.x_ring_size          = "Q2";

    auto out = skill::x_ring_groove::apply(*stock, in);
    const auto& pat = out.signature.pattern;
    EXPECT_EQ(pat.at("kind").get<std::string>(), std::string("x_ring_groove"));
    EXPECT_EQ(pat.at("is_compound").get<bool>(), true);
    EXPECT_GE(pat.at("subfeature_count").get<int>(), 2);
    EXPECT_NEAR(pat.at("sidewall_angle_deg").get<double>(), 10.0, 1e-6);
}

// ─── 4. Recognize metadata replay ──────────────────────────────────────────
TEST(SkillXRingGroove, RecognizeMetadataReplay)
{
    auto stock = skill::createCylindricalStock(40.0, 30.0);

    skill::x_ring_groove::Input in;
    in.bore_or_shaft_dia_mm = 40.0;
    in.mean_dia_mm          = 35.0;
    in.position_z_mm        = 15.0;
    in.x_ring_size          = "Q3";

    auto out   = skill::x_ring_groove::apply(*stock, in);
    auto cands = skill::x_ring_groove::recognize(*out.workpiece);
    ASSERT_GE(cands.size(), 1u);
    EXPECT_EQ(cands[0].skill_id, std::string("x_ring_groove"));
    EXPECT_DOUBLE_EQ(cands[0].confidence, 1.0);
}

// ─── 5. X-ring groove is wider than O-ring groove for same CS ─────────────
TEST(SkillXRingGroove, GrooveWiderThanORingGroove)
{
    // Per Parker handbook: X-ring W = 1.70·CS, O-ring W = 1.45·CS.
    const double cs = 3.53;
    EXPECT_NEAR(skill::x_ring_groove::crossSectionForQ("Q3"), cs, 1e-6);

    const double w_x = skill::x_ring_groove::recommendedXWidthFor(cs);
    const double w_o = 1.45 * cs;
    EXPECT_GT(w_x, w_o);
    EXPECT_NEAR(w_x, 1.70 * cs, 1e-6);

    const double g_x = skill::x_ring_groove::recommendedXDepthFor(cs);
    const double g_o = 0.74 * cs;
    EXPECT_GT(g_x, g_o);   // X-ring also deeper

    // Verify signature carries the X-specific (wider) width.
    auto stock = skill::createCylindricalStock(40.0, 30.0);
    skill::x_ring_groove::Input in;
    in.bore_or_shaft_dia_mm = 40.0;
    in.mean_dia_mm          = 35.0;
    in.position_z_mm        = 15.0;
    in.x_ring_size          = "Q3";

    auto out = skill::x_ring_groove::apply(*stock, in);
    EXPECT_NEAR(out.signature.pattern.at("groove_width_mm").get<double>(),
                1.70 * cs, 1e-6);
}
