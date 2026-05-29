// @lat: [[process/test-strategy#compound bushing bore]]
//
// plain_bushing_bore_with_lube_groove — compound: bore + helical lube
// groove (HelicalSweep primitive) + entry chamfer.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/plain_bushing_bore_with_lube_groove.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

#include <cmath>

using namespace koocadcam;

namespace {
double volumeOf(const TopoDS_Shape& s)
{
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}
}  // namespace

// ─── 1. Apply produces valid shape with multiple sub-features ─────────────
TEST(SkillPlainBushingBoreLube, ApplyProducesValidShapeWithMultipleSubFeatures)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 25.0);

    skill::plain_bushing_bore_with_lube_groove::Input in;
    in.entry_face     = skill::FaceByNormal{ gp_Dir(0, 0, 1), 5.0, "largest" };
    in.position_x_mm  = 20.0;
    in.position_y_mm  = 20.0;
    in.axis_dir       = gp_Dir(0, 0, -1);
    in.bore_dia_mm    = 12.0;
    in.length_mm      = 20.0;
    in.groove_pitch_mm = 10.0;

    const double v0 = volumeOf(stock->shape());
    const int    f0 = stock->faceCount();

    auto out = skill::plain_bushing_bore_with_lube_groove::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    const double v1 = volumeOf(out.workpiece->shape());
    const int    f1 = out.workpiece->faceCount();

    // Bore volume = π × 6² × 20 = ~2262.  Groove + chamfer add a little.
    const double boreVol = M_PI * 6.0 * 6.0 * 20.0;
    const double removed = v0 - v1;
    EXPECT_GT(removed, boreVol * 0.95);
    EXPECT_LT(removed, boreVol * 1.5);

    EXPECT_GT(f1, f0);
}

// ─── 2. Validate rejects groove_pitch that would self-intersect ───────────
TEST(SkillPlainBushingBoreLube, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 25.0);
    skill::plain_bushing_bore_with_lube_groove::Input in;
    in.entry_face     = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm  = 20.0;
    in.position_y_mm  = 20.0;
    in.bore_dia_mm    = 12.0;
    in.length_mm      = 20.0;
    in.groove_pitch_mm = 3.0;   // < 0.5 × bore_dia (6.0) → self-intersect

    auto r = skill::plain_bushing_bore_with_lube_groove::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-GROOVE-PITCH") { found = true; break; }
    EXPECT_TRUE(found);

    EXPECT_THROW(skill::plain_bushing_bore_with_lube_groove::apply(*stock, in),
                 skill::SkillError);
}

// ─── 3. Signature records compound kind ──────────────────────────────────
TEST(SkillPlainBushingBoreLube, SignatureRecordsCompoundKind)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 25.0);
    skill::plain_bushing_bore_with_lube_groove::Input in;
    in.entry_face     = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm  = 20.0;
    in.position_y_mm  = 20.0;
    in.bore_dia_mm    = 12.0;
    in.length_mm      = 20.0;
    in.groove_pitch_mm = 10.0;
    auto out = skill::plain_bushing_bore_with_lube_groove::apply(*stock, in);

    EXPECT_TRUE(out.signature.pattern.at("is_compound").get<bool>());
    EXPECT_GE(out.signature.pattern.at("subfeature_count").get<int>(), 2);
    EXPECT_EQ(out.signature.pattern.at("kind").get<std::string>(),
              std::string("plain_bushing_bore_with_lube_groove"));
}

// ─── 4. Recognize via metadata replay ─────────────────────────────────────
TEST(SkillPlainBushingBoreLube, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 25.0);
    skill::plain_bushing_bore_with_lube_groove::Input in;
    in.entry_face     = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm  = 20.0;
    in.position_y_mm  = 20.0;
    in.bore_dia_mm    = 12.0;
    in.length_mm      = 20.0;
    in.groove_pitch_mm = 10.0;
    auto out = skill::plain_bushing_bore_with_lube_groove::apply(*stock, in);

    auto cands = skill::plain_bushing_bore_with_lube_groove::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-9);
    EXPECT_NEAR(cands[0].recovered_params["bore_dia_mm"].get<double>(),
                12.0, 1e-6);
}

// ─── 5. Helical profile tag is recorded for downstream CAPP ───────────────
TEST(SkillPlainBushingBoreLube, HelicalProfileTagRecorded)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 25.0);
    skill::plain_bushing_bore_with_lube_groove::Input in;
    in.entry_face     = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm  = 20.0;
    in.position_y_mm  = 20.0;
    in.bore_dia_mm    = 12.0;
    in.length_mm      = 20.0;
    in.groove_pitch_mm = 10.0;
    auto out = skill::plain_bushing_bore_with_lube_groove::apply(*stock, in);

    ASSERT_TRUE(out.signature.pattern.contains("helical_profile_used"));
    const std::string tag =
        out.signature.pattern.at("helical_profile_used").get<std::string>();
    // One of three documented tags from HelicalSweepProfile.
    EXPECT_TRUE(tag == "iso_60_with_radii" ||
                tag == "iso_60_simplified" ||
                tag == "annular_fallback");
}
