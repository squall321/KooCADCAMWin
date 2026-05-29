// @lat: [[process/test-strategy#compound caseback_o_ring_groove]]

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/caseback_o_ring_groove.hpp"

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

skill::caseback_o_ring_groove::Input goodInput() {
    skill::caseback_o_ring_groove::Input in;
    in.caseback_face       = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.center_x_mm         = 0.0;
    in.center_y_mm         = 0.0;
    in.axis_dir            = gp_Dir(0, 0, -1);
    in.mean_dia_mm         = 28.0;
    in.o_ring_size         = "-008";    // CS = 1.78 mm
    in.thread_pilot_od_mm  = 33.0;
    in.thread_pilot_depth_mm = 0.6;
    return in;
}
}  // namespace

// ─── 1. Apply produces valid shape with 2 sub-features ───────────────────
TEST(SkillCasebackORing, ApplyProducesValidShapeWithMultipleSubFeatures)
{
    auto stock = skill::createCylindricalStock(40.0, 5.0);

    auto in = goodInput();
    auto out = skill::caseback_o_ring_groove::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    // Expected groove vol ≈ π × (rO² − rI²) × 0.75 × 1.78.
    const double cs = 1.78;
    const double gW = 1.30 * cs;
    const double gD = 0.75 * cs;
    const double rO = 14.0 + gW / 2.0;
    const double rI = 14.0 - gW / 2.0;
    const double grooveVol = M_PI * (rO*rO - rI*rI) * gD;
    const double diff = volumeOf(stock->shape()) - volumeOf(out.workpiece->shape());
    EXPECT_GT(diff, grooveVol * 0.6);

    EXPECT_GT(out.workpiece->faceCount(), stock->faceCount());
}

// ─── 2. Validate rejects non-catalog o_ring_size ─────────────────────────
TEST(SkillCasebackORing, ValidateRejectsBadInput)
{
    auto stock = skill::createCylindricalStock(40.0, 5.0);

    auto in = goodInput();
    in.o_ring_size = "-999";   // not in AS568 face-seal catalog

    auto r = skill::caseback_o_ring_groove::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-ORING-SIZE") { found = true; break; }
    EXPECT_TRUE(found);

    EXPECT_THROW(skill::caseback_o_ring_groove::apply(*stock, in),
                 skill::SkillError);
}

// ─── 3. Signature records compound kind ──────────────────────────────────
TEST(SkillCasebackORing, SignatureRecordsCompoundKind)
{
    auto stock = skill::createCylindricalStock(40.0, 5.0);

    auto in = goodInput();
    auto out = skill::caseback_o_ring_groove::apply(*stock, in);

    EXPECT_EQ(out.signature.skill_id, std::string("caseback_o_ring_groove"));
    EXPECT_TRUE(out.signature.pattern.at("is_compound").get<bool>());
    EXPECT_GE(out.signature.pattern.at("subfeature_count").get<int>(), 2);
    EXPECT_EQ(out.signature.pattern.at("o_ring_size").get<std::string>(),
              std::string("-008"));
}

// ─── 4. Recognize via metadata replay ────────────────────────────────────
TEST(SkillCasebackORing, RecognizeMetadataReplay)
{
    auto stock = skill::createCylindricalStock(40.0, 5.0);

    auto in = goodInput();
    auto out = skill::caseback_o_ring_groove::apply(*stock, in);

    auto cands = skill::caseback_o_ring_groove::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_DOUBLE_EQ(cands[0].confidence, 1.0);
    EXPECT_EQ(cands[0].recovered_params.at("o_ring_size").get<std::string>(),
              std::string("-008"));
}

// ─── 5. AS568 cross-section table lookup ─────────────────────────────────
TEST(SkillCasebackORing, AS568CrossSectionLookup)
{
    EXPECT_NEAR(skill::caseback_o_ring_groove::aS568CrossSection("-008"),
                1.78, 1e-6);
    EXPECT_NEAR(skill::caseback_o_ring_groove::aS568CrossSection("-014"),
                1.78, 1e-6);
    EXPECT_NEAR(skill::caseback_o_ring_groove::aS568CrossSection("-020"),
                1.78, 1e-6);
    EXPECT_DOUBLE_EQ(skill::caseback_o_ring_groove::aS568CrossSection("-999"),
                     0.0);
    EXPECT_DOUBLE_EQ(skill::caseback_o_ring_groove::aS568CrossSection(""), 0.0);
}
