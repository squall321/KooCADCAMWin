// @lat: [[process/test-strategy#compound crown_stem_cavity_compound]]

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/crown_stem_cavity_compound.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

#include <gp_Ax1.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>

#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using namespace koocadcam;

namespace {
double volumeOf(const TopoDS_Shape& s) {
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}

skill::crown_stem_cavity_compound::Input goodInput() {
    skill::crown_stem_cavity_compound::Input in;
    in.case_axis            = gp_Ax1(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1));
    in.case_outer_radius_mm = 20.0;
    in.angle_deg            = 90.0;     // crown at 3-o'clock
    in.port_center_z_mm     = 5.0;
    in.cavity_dia_mm        = 3.0;
    in.cavity_depth_mm      = 2.5;
    in.stem_dia_mm          = 1.5;
    in.stem_total_length_mm = 8.0;
    in.o_ring_cs_mm         = 1.0;
    in.o_ring_depth_mm      = 0.4;
    in.o_ring_offset_mm     = 1.0;
    return in;
}
}  // namespace

// ─── 1. Apply produces valid shape with 3 sub-features ───────────────────
TEST(SkillCrownStemCavity, ApplyProducesValidShapeWithMultipleSubFeatures)
{
    auto stock = skill::createCylindricalStock(40.0, 10.0);

    auto in = goodInput();
    auto out = skill::crown_stem_cavity_compound::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    // Cavity + stem (beyond cavity) + groove all subtract.
    const double cavityVol = M_PI * (1.5 * 1.5) * 2.5;
    const double stemBeyond = M_PI * (0.75 * 0.75) * 5.5;
    const double minRemoved = cavityVol * 0.6;   // OCCT may approximate
    const double diff = volumeOf(stock->shape()) - volumeOf(out.workpiece->shape());
    EXPECT_GT(diff, minRemoved);
    EXPECT_GT(diff, cavityVol + stemBeyond * 0.3);

    EXPECT_GT(out.workpiece->faceCount(), stock->faceCount());
}

// ─── 2. Validate rejects insufficient radial shelf ───────────────────────
TEST(SkillCrownStemCavity, ValidateRejectsBadInput)
{
    auto stock = skill::createCylindricalStock(40.0, 10.0);

    auto in = goodInput();
    in.cavity_dia_mm = 1.6;   // stem = 1.5 → shelf = 0.05 mm (insufficient)

    auto r = skill::crown_stem_cavity_compound::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-CROWN-SHELF") { found = true; break; }
    EXPECT_TRUE(found);

    EXPECT_THROW(skill::crown_stem_cavity_compound::apply(*stock, in),
                 skill::SkillError);
}

// ─── 3. Signature records compound kind ──────────────────────────────────
TEST(SkillCrownStemCavity, SignatureRecordsCompoundKind)
{
    auto stock = skill::createCylindricalStock(40.0, 10.0);

    auto in = goodInput();
    auto out = skill::crown_stem_cavity_compound::apply(*stock, in);

    EXPECT_EQ(out.signature.skill_id, std::string("crown_stem_cavity_compound"));
    EXPECT_TRUE(out.signature.pattern.at("is_compound").get<bool>());
    EXPECT_EQ(out.signature.pattern.at("subfeature_count").get<int>(), 3);
    EXPECT_GE(out.signature.pattern.at("subfeature_count").get<int>(), 2);
}

// ─── 4. Recognize via metadata replay ────────────────────────────────────
TEST(SkillCrownStemCavity, RecognizeMetadataReplay)
{
    auto stock = skill::createCylindricalStock(40.0, 10.0);

    auto in = goodInput();
    auto out = skill::crown_stem_cavity_compound::apply(*stock, in);

    auto cands = skill::crown_stem_cavity_compound::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_DOUBLE_EQ(cands[0].confidence, 1.0);
    EXPECT_NEAR(cands[0].recovered_params.at("cavity_dia_mm").get<double>(),
                3.0, 1e-6);
    EXPECT_NEAR(cands[0].recovered_params.at("stem_dia_mm").get<double>(),
                1.5, 1e-6);
}

// ─── 5. Cavity dia > stem dia + 0.8 mm — radial shelf invariant ──────────
TEST(SkillCrownStemCavity, RadialShelfInvariant)
{
    auto stock = skill::createCylindricalStock(40.0, 10.0);

    auto in = goodInput();
    auto out = skill::crown_stem_cavity_compound::apply(*stock, in);
    const auto& pat = out.signature.pattern;

    const double cavityD = pat.at("cavity_dia_mm").get<double>();
    const double stemD   = pat.at("stem_dia_mm").get<double>();
    const double shelf   = pat.at("radial_shelf_mm").get<double>();
    EXPECT_GE(shelf, 0.4);
    EXPECT_GT(cavityD, stemD);
    EXPECT_NEAR(shelf, (cavityD - stemD) / 2.0, 1e-6);
}
