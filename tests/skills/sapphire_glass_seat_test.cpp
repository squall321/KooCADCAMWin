// @lat: [[process/test-strategy#compound sapphire_glass_seat]]

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/sapphire_glass_seat.hpp"

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

skill::sapphire_glass_seat::Input goodInput() {
    skill::sapphire_glass_seat::Input in;
    in.case_face        = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.center_x_mm      = 0.0;
    in.center_y_mm      = 0.0;
    in.axis_dir         = gp_Dir(0, 0, -1);
    in.glass_dia_mm     = 30.0;
    in.ledge_depth_mm   = 2.0;
    in.inner_bore_dia_mm = 28.0;
    in.inner_bore_depth_mm = 4.0;
    in.o_ring_cs_mm     = 1.0;
    in.o_ring_depth_mm  = 0.4;
    in.o_ring_z_offset_mm = 0.5;
    return in;
}
}  // namespace

// ─── 1. Apply produces valid shape with 3 sub-features ───────────────────
TEST(SkillSapphireSeat, ApplyProducesValidShapeWithMultipleSubFeatures)
{
    auto stock = skill::createCylindricalStock(40.0, 8.0);

    auto in = goodInput();
    auto out = skill::sapphire_glass_seat::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    const double rOuter = 15.025;
    const double rInner = 14.0;
    const double outerVol = M_PI * rOuter * rOuter * 2.0;
    const double innerVol = M_PI * rInner * rInner * 2.0;
    const double minExpected = outerVol * 0.95;
    const double diff = volumeOf(stock->shape()) - volumeOf(out.workpiece->shape());
    EXPECT_GT(diff, minExpected);
    EXPECT_GT(diff, outerVol + innerVol * 0.3);

    EXPECT_GT(out.workpiece->faceCount(), stock->faceCount());
}

// ─── 2. Validate rejects ledge too small ─────────────────────────────────
TEST(SkillSapphireSeat, ValidateRejectsBadInput)
{
    auto stock = skill::createCylindricalStock(40.0, 8.0);

    auto in = goodInput();
    in.inner_bore_dia_mm = 29.8;   // ledge width = 0.1 mm (per side) — too thin

    auto r = skill::sapphire_glass_seat::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-LEDGE-GEOM") { found = true; break; }
    EXPECT_TRUE(found);

    EXPECT_THROW(skill::sapphire_glass_seat::apply(*stock, in),
                 skill::SkillError);
}

// ─── 3. Signature records compound kind ──────────────────────────────────
TEST(SkillSapphireSeat, SignatureRecordsCompoundKind)
{
    auto stock = skill::createCylindricalStock(40.0, 8.0);

    auto in = goodInput();
    auto out = skill::sapphire_glass_seat::apply(*stock, in);

    EXPECT_EQ(out.signature.skill_id, std::string("sapphire_glass_seat"));
    EXPECT_TRUE(out.signature.pattern.at("is_compound").get<bool>());
    EXPECT_EQ(out.signature.pattern.at("subfeature_count").get<int>(), 3);
    EXPECT_GE(out.signature.pattern.at("subfeature_count").get<int>(), 2);
}

// ─── 4. Recognize via metadata replay ────────────────────────────────────
TEST(SkillSapphireSeat, RecognizeMetadataReplay)
{
    auto stock = skill::createCylindricalStock(40.0, 8.0);

    auto in = goodInput();
    auto out = skill::sapphire_glass_seat::apply(*stock, in);

    auto cands = skill::sapphire_glass_seat::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_DOUBLE_EQ(cands[0].confidence, 1.0);
    EXPECT_NEAR(cands[0].recovered_params.at("glass_dia_mm").get<double>(),
                30.0, 1e-6);
}

// ─── 5. Ledge width = (glass_dia - inner_bore_dia)/2 — invariant ─────────
TEST(SkillSapphireSeat, LedgeWidthInvariant)
{
    auto stock = skill::createCylindricalStock(40.0, 8.0);

    auto in = goodInput();
    auto out = skill::sapphire_glass_seat::apply(*stock, in);
    const auto& pat = out.signature.pattern;

    const double glassD     = pat.at("glass_dia_mm").get<double>();
    const double innerBoreD = pat.at("inner_bore_dia_mm").get<double>();
    const double ledgeW     = pat.at("ledge_width_mm").get<double>();
    EXPECT_LT(innerBoreD, glassD);
    EXPECT_NEAR(ledgeW, (glassD - innerBoreD) / 2.0, 1e-6);
    EXPECT_GT(ledgeW, 0.4);
}
