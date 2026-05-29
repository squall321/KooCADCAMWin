// @lat: [[process/test-strategy#compound locator_pin_set]]
//
// Verifies REAL multi-feature compound: fuse(shoulder, post) onto plate +
// cut chamfer band + cut E-clip groove ring.  Plate volume INCREASES by
// pin material minus groove removed.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/locator_pin_set.hpp"

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

// ─── 1. apply: volume delta matches added pin material ───────────────────
TEST(SkillLocatorPinSet, ApplyAddsPinMaterialAndCutsGroove)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 8.0);
    const int    stockFaces = stock->faceCount();
    const double stockVol   = volumeOf(stock->shape());

    skill::locator_pin_set::Input in;
    in.base_face          = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm      = 30.0;
    in.position_y_mm      = 30.0;
    in.axis_dir           = gp_Dir(0, 0, 1);
    in.shoulder_dia_mm    = 14.0;
    in.shoulder_height_mm = 3.0;
    in.post_dia_mm        = 8.0;
    in.post_height_mm     = 20.0;
    in.lead_in_chamfer_mm = 1.0;
    in.groove_dia_mm      = 6.5;
    in.groove_width_mm    = 1.5;
    in.groove_z_offset_mm = 1.0;

    auto out = skill::locator_pin_set::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    // Pin material added:
    //   shoulder vol = π·7² · 3                = 461.8
    //   post vol     = π·4² · 20               = 1005.3
    // Minus removed:
    //   chamfer cone cap (small)
    //   groove ring  = π·(4² − 3.25²) · 1.5    ≈ 25.6
    const double rPost = 4.0, rShoulder = 7.0, rGroove = 3.25;
    const double added =
        M_PI * rShoulder * rShoulder * 3.0 +
        M_PI * rPost     * rPost     * 20.0;
    const double removedGroove =
        M_PI * (rPost * rPost - rGroove * rGroove) * 1.5;
    // The chamfer removes ≲ π · rPost² · chamfer ≈ 50 mm³; treat it as a
    // bounded extra in the tolerance band rather than computing exactly.
    const double approx = added - removedGroove;
    const double delta  = volumeOf(out.workpiece->shape()) - stockVol;
    // Tolerance generous enough to bracket chamfer cone interior overlap.
    EXPECT_NEAR(delta, approx, approx * 0.10);

    EXPECT_GT(out.workpiece->faceCount(), stockFaces);
}

// ─── 2. validate rejects bad geometry + apply throws ─────────────────────
TEST(SkillLocatorPinSet, ValidateRejectsBadGeometry)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 8.0);

    // (a) shoulder ≤ post
    {
        skill::locator_pin_set::Input in;
        in.base_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
        in.position_x_mm = 30.0; in.position_y_mm = 30.0;
        in.shoulder_dia_mm = 8.0;        // = post_dia → no shoulder
        in.shoulder_height_mm = 3.0;
        in.post_dia_mm = 8.0;
        in.post_height_mm = 20.0;
        in.lead_in_chamfer_mm = 1.0;
        in.groove_dia_mm = 6.5; in.groove_width_mm = 1.5; in.groove_z_offset_mm = 1.0;
        auto r = skill::locator_pin_set::validate(*stock, in);
        EXPECT_FALSE(r.passed);
        bool found = false;
        for (const auto& f : r.findings)
            if (f.code == "DFM-PIN-SHOULDER") { found = true; break; }
        EXPECT_TRUE(found);
        EXPECT_THROW(skill::locator_pin_set::apply(*stock, in), skill::SkillError);
    }
    // (b) DIN 6321 slenderness > 6
    {
        skill::locator_pin_set::Input in;
        in.base_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
        in.position_x_mm = 30.0; in.position_y_mm = 30.0;
        in.shoulder_dia_mm = 14.0; in.shoulder_height_mm = 3.0;
        in.post_dia_mm = 4.0;        // L/D = 50/4 = 12.5 > 6
        in.post_height_mm = 50.0;
        in.lead_in_chamfer_mm = 1.0;
        in.groove_dia_mm = 3.0; in.groove_width_mm = 1.5; in.groove_z_offset_mm = 1.0;
        auto r = skill::locator_pin_set::validate(*stock, in);
        EXPECT_FALSE(r.passed);
        bool found = false;
        for (const auto& f : r.findings)
            if (f.code == "DFM-PIN-SLENDER") { found = true; break; }
        EXPECT_TRUE(found);
    }
}

// ─── 3. signature compound + params round-trip ───────────────────────────
TEST(SkillLocatorPinSet, SignatureCompoundRoundTrip)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 8.0);
    skill::locator_pin_set::Input in;
    in.base_face       = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm   = 30.0; in.position_y_mm = 30.0;
    in.axis_dir        = gp_Dir(0, 0, 1);
    in.shoulder_dia_mm = 14.0; in.shoulder_height_mm = 3.0;
    in.post_dia_mm     = 8.0;  in.post_height_mm    = 20.0;
    in.lead_in_chamfer_mm = 1.0;
    in.groove_dia_mm   = 6.5;  in.groove_width_mm = 1.5;
    in.groove_z_offset_mm = 1.0;

    auto out = skill::locator_pin_set::apply(*stock, in);
    EXPECT_TRUE(out.signature.pattern.at("is_compound").get<bool>());
    EXPECT_GE(out.signature.pattern.at("subfeature_count").get<int>(), 2);
    EXPECT_EQ(out.signature.pattern.at("subfeature_count").get<int>(), 4);
    EXPECT_NEAR(out.signature.params.at("post_dia_mm").get<double>(),
                in.post_dia_mm, 1e-6);
    EXPECT_NEAR(out.signature.params.at("groove_width_mm").get<double>(),
                in.groove_width_mm, 1e-6);
}

// ─── 4. recognize: metadata replay returns 1.0 confidence ────────────────
TEST(SkillLocatorPinSet, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 8.0);
    skill::locator_pin_set::Input in;
    in.base_face       = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm   = 30.0; in.position_y_mm = 30.0;
    in.axis_dir        = gp_Dir(0, 0, 1);
    in.shoulder_dia_mm = 14.0; in.shoulder_height_mm = 3.0;
    in.post_dia_mm     = 8.0;  in.post_height_mm    = 20.0;
    in.lead_in_chamfer_mm = 1.0;
    in.groove_dia_mm   = 6.5;  in.groove_width_mm = 1.5;
    in.groove_z_offset_mm = 1.0;

    auto out = skill::locator_pin_set::apply(*stock, in);
    auto cands = skill::locator_pin_set::recognize(*out.workpiece);
    ASSERT_FALSE(cands.empty());
    EXPECT_DOUBLE_EQ(cands.front().confidence, 1.0);
    EXPECT_EQ(cands.front().skill_id, std::string("locator_pin_set"));
}

// ─── 5. feature-specific: groove geometry valid (groove < post < shoulder)
TEST(SkillLocatorPinSet, GeometryHierarchy)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 8.0);
    skill::locator_pin_set::Input in;
    in.base_face       = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm   = 30.0; in.position_y_mm = 30.0;
    in.axis_dir        = gp_Dir(0, 0, 1);
    in.shoulder_dia_mm = 14.0; in.shoulder_height_mm = 3.0;
    in.post_dia_mm     = 8.0;  in.post_height_mm    = 20.0;
    in.lead_in_chamfer_mm = 1.0;
    in.groove_dia_mm   = 6.5;  in.groove_width_mm = 1.5;
    in.groove_z_offset_mm = 1.0;

    auto out = skill::locator_pin_set::apply(*stock, in);
    const double groove = out.signature.pattern.at("groove_dia_mm").get<double>();
    const double post   = out.signature.pattern.at("post_dia_mm").get<double>();
    const double shoulder = out.signature.pattern.at("shoulder_dia_mm").get<double>();
    EXPECT_LT(groove,   post);
    EXPECT_LT(post,     shoulder);
}
