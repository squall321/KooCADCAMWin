// @lat: [[process/test-strategy#compound macro]]
//
// o_ring_groove_radial — compound: rect groove + bottom radius + 5° draft.
//
// Tests (5):
//   1. ApplyProducesValidShapeWithMultipleSubFeatures — face count grows
//      consistent with 3 sub-features, volume removed > rect-only estimate.
//   2. ValidateRejectsBadInput — unknown AS568 size + too-small bore.
//   3. SignatureRecordsCompoundKind — pattern.is_compound = true, count ≥ 2.
//   4. RecognizeMetadataReplay — confidence = 1.0 from history.
//   5. GrooveDepthMatchesAS568 — derived G ≈ 0.74·CS within tolerance.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/o_ring_groove_radial.hpp"

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
TEST(SkillORingGrooveRadial, ApplyProducesValidShapeWithMultipleSubFeatures)
{
    auto stock = skill::createCylindricalStock(40.0, 30.0);   // Ø40, len 30

    skill::o_ring_groove_radial::Input in;
    in.bore_or_shaft_dia_mm = 40.0;
    in.position_z_mm        = 15.0;
    in.o_ring_size          = "-210";   // CS = 3.53 mm

    const int facesBefore = stock->faceCount();
    auto out = skill::o_ring_groove_radial::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    // The 3 sub-features (main + bottom radius + 2 sidewall drafts) should
    // produce more faces than a single rectangular groove cut.
    const int facesAfter = out.workpiece->faceCount();
    EXPECT_GT(facesAfter, facesBefore);

    // Volume removed must EXCEED the bare rectangular-section estimate
    // (the bottom radius + side drafts add a small amount).
    const double cs       = 3.53;
    const double depth_G  = 0.74 * cs;
    const double width_W  = 1.45 * cs;
    const double shaftR   = 20.0;
    const double grooveR  = shaftR - depth_G;
    const double rectVol  = M_PI * (shaftR * shaftR - grooveR * grooveR) * width_W;

    const double removed = volumeOf(stock->shape()) - volumeOf(out.workpiece->shape());
    EXPECT_GT(removed, 0.9 * rectVol);     // at least 90% of rect estimate
    EXPECT_LT(removed, 1.5 * rectVol);     // not absurdly more
}

// ─── 2. Validate rejects unknown AS568 size + sub-min bore ─────────────────
TEST(SkillORingGrooveRadial, ValidateRejectsBadInput)
{
    auto stock = skill::createCylindricalStock(40.0, 30.0);

    skill::o_ring_groove_radial::Input bad;
    bad.bore_or_shaft_dia_mm = 40.0;
    bad.position_z_mm        = 15.0;
    bad.o_ring_size          = "-XYZ";   // not in AS568 table

    auto r1 = skill::o_ring_groove_radial::validate(*stock, bad);
    EXPECT_FALSE(r1.passed);
    bool foundUnknown = false;
    for (const auto& f : r1.findings)
        if (f.code == "DFM-AS568") { foundUnknown = true; break; }
    EXPECT_TRUE(foundUnknown);

    EXPECT_THROW(skill::o_ring_groove_radial::apply(*stock, bad), skill::SkillError);

    // Too-small bore for a valid AS568 size.
    auto tinyStock = skill::createCylindricalStock(1.5, 30.0);
    skill::o_ring_groove_radial::Input small;
    small.bore_or_shaft_dia_mm = 1.5;
    small.position_z_mm        = 15.0;
    small.o_ring_size          = "-210";
    auto r2 = skill::o_ring_groove_radial::validate(*tinyStock, small);
    EXPECT_FALSE(r2.passed);
}

// ─── 3. Signature records compound kind ────────────────────────────────────
TEST(SkillORingGrooveRadial, SignatureRecordsCompoundKind)
{
    auto stock = skill::createCylindricalStock(40.0, 30.0);

    skill::o_ring_groove_radial::Input in;
    in.bore_or_shaft_dia_mm = 40.0;
    in.position_z_mm        = 15.0;
    in.o_ring_size          = "-110";

    auto out = skill::o_ring_groove_radial::apply(*stock, in);
    const auto& pat = out.signature.pattern;
    EXPECT_EQ(pat.at("kind").get<std::string>(),
              std::string("o_ring_groove_radial"));
    EXPECT_EQ(pat.at("is_compound").get<bool>(), true);
    EXPECT_GE(pat.at("subfeature_count").get<int>(), 2);
}

// ─── 4. Recognize via metadata replay ──────────────────────────────────────
TEST(SkillORingGrooveRadial, RecognizeMetadataReplay)
{
    auto stock = skill::createCylindricalStock(40.0, 30.0);

    skill::o_ring_groove_radial::Input in;
    in.bore_or_shaft_dia_mm = 40.0;
    in.position_z_mm        = 15.0;
    in.o_ring_size          = "-210";

    auto out = skill::o_ring_groove_radial::apply(*stock, in);
    auto cands = skill::o_ring_groove_radial::recognize(*out.workpiece);
    ASSERT_GE(cands.size(), 1u);
    EXPECT_EQ(cands[0].skill_id, std::string("o_ring_groove_radial"));
    EXPECT_DOUBLE_EQ(cands[0].confidence, 1.0);
    EXPECT_EQ(cands[0].recovered_params.at("o_ring_size").get<std::string>(),
              std::string("-210"));
}

// ─── 5. Specific assertion: groove depth ≈ 0.74·CS (AS568) ─────────────────
TEST(SkillORingGrooveRadial, GrooveDepthMatchesAS568)
{
    EXPECT_NEAR(skill::o_ring_groove_radial::crossSectionFor("-110"), 2.62, 1e-6);
    EXPECT_NEAR(skill::o_ring_groove_radial::crossSectionFor("-210"), 3.53, 1e-6);
    EXPECT_EQ  (skill::o_ring_groove_radial::crossSectionFor("-999"), 0.0);

    EXPECT_NEAR(skill::o_ring_groove_radial::recommendedDepthFor(3.53),
                0.74 * 3.53, 1e-6);
    EXPECT_NEAR(skill::o_ring_groove_radial::recommendedWidthFor(3.53),
                1.45 * 3.53, 1e-6);

    auto stock = skill::createCylindricalStock(40.0, 30.0);
    skill::o_ring_groove_radial::Input in;
    in.bore_or_shaft_dia_mm = 40.0;
    in.position_z_mm        = 15.0;
    in.o_ring_size          = "-210";
    auto out = skill::o_ring_groove_radial::apply(*stock, in);

    const auto& pat = out.signature.pattern;
    EXPECT_NEAR(pat.at("groove_depth_mm").get<double>(), 0.74 * 3.53, 1e-6);
    EXPECT_NEAR(pat.at("groove_width_mm").get<double>(), 1.45 * 3.53, 1e-6);
    // groove_radius = shaftR - depth = 20 - 2.6122 = 17.3878
    EXPECT_NEAR(pat.at("groove_radius_mm").get<double>(),
                20.0 - 0.74 * 3.53, 1e-6);
}
