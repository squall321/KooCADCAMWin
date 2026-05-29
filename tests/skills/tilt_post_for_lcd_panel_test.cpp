// @lat: [[process/test-strategy#compound tilt_post_for_lcd_panel]]
//
// Verifies REAL multi-cut + multi-fuse geometry: pocket CUT + groove CUT
// + pillar FUSE, optional auto-extend plate FUSE.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/tilt_post_for_lcd_panel.hpp"

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

// ─── T1. apply: net volume = pillar_FUSED − pocket_CUT (±20%) ───────────
TEST(SkillTiltPost, ApplyRealGeometryNetVolumeAndFaces)
{
    auto stock = skill::createCuboidStock(80.0, 80.0, 5.0);
    const int    stockFaces = stock->faceCount();
    const double stockVol   = volumeOf(stock->shape());

    skill::tilt_post_for_lcd_panel::Input in;
    in.base_face    = skill::FaceByNormal{ gp_Dir(0, 0, 1), 5.0, "largest" };
    in.axis_dir     = gp_Dir(0, 0, -1);
    in.position_x_mm = 40.0;
    in.position_y_mm = 30.0;   // well inside the 80×80 footprint
    in.panel_size    = "PANEL-10";

    auto out = skill::tilt_post_for_lcd_panel::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    // PANEL-10: pin Ø 4 → r=2; pocket 8 mm deep; pillar 10×8×16
    // Pocket CUT vol = π·4·8 = 100.5
    // Groove (r=2.4)²−(r=2)² = 1.76; × π × 0.5 ≈ 2.76
    // Pillar FUSE vol = 10·8·16 = 1280
    // Net added = 1280 − (100.5 + 2.76) ≈ 1176.7
    const double rPin = 2.0;
    const double rGroove = 2.4;
    const double pocketVol = M_PI * rPin * rPin * 8.0;
    const double grooveVol = M_PI * (rGroove * rGroove - rPin * rPin) * 0.5;
    const double pillarVol = 10.0 * 8.0 * 16.0;
    const double expectedNet = pillarVol - pocketVol - grooveVol;
    const double netAdded = volumeOf(out.workpiece->shape()) - stockVol;
    EXPECT_NEAR(netAdded, expectedNet, expectedNet * 0.20);

    EXPECT_GT(out.workpiece->faceCount(), stockFaces);
}

// ─── T2. validate rejects unknown panel + apply throws ──────────────────
TEST(SkillTiltPost, ValidateRejectsUnknownSpec)
{
    auto stock = skill::createCuboidStock(80.0, 80.0, 5.0);

    skill::tilt_post_for_lcd_panel::Input in;
    in.base_face     = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 40.0;
    in.position_y_mm = 30.0;
    in.panel_size    = "PANEL-9999";  // unknown

    auto r = skill::tilt_post_for_lcd_panel::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool foundSpec = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-CONN-SIZE") { foundSpec = true; break; }
    EXPECT_TRUE(foundSpec);

    EXPECT_THROW(skill::tilt_post_for_lcd_panel::apply(*stock, in),
                 skill::SkillError);
}

// ─── T3. signature carries spec key + is_compound + ≥3 subs ─────────────
TEST(SkillTiltPost, SignatureCarriesSpecAndCompoundFlag)
{
    auto stock = skill::createCuboidStock(80.0, 80.0, 5.0);

    skill::tilt_post_for_lcd_panel::Input in;
    in.base_face     = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.axis_dir      = gp_Dir(0, 0, -1);
    in.position_x_mm = 40.0;
    in.position_y_mm = 30.0;
    in.panel_size    = "PANEL-13";

    auto out = skill::tilt_post_for_lcd_panel::apply(*stock, in);
    EXPECT_TRUE(out.signature.pattern.at("is_compound").get<bool>());
    EXPECT_EQ(out.signature.pattern.at("kind").get<std::string>(),
              std::string("tilt_post_for_lcd_panel"));
    EXPECT_EQ(out.signature.pattern.at("panel_size").get<std::string>(),
              std::string("PANEL-13"));
    EXPECT_GE(out.signature.pattern.at("subfeature_count").get<int>(), 3);
}

// ─── T4. recognize returns metadata-replay candidate ────────────────────
TEST(SkillTiltPost, RecognizeReturnsCandidate)
{
    auto stock = skill::createCuboidStock(80.0, 80.0, 5.0);

    skill::tilt_post_for_lcd_panel::Input in;
    in.base_face     = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.axis_dir      = gp_Dir(0, 0, -1);
    in.position_x_mm = 40.0;
    in.position_y_mm = 30.0;
    in.panel_size    = "PANEL-15";

    auto out = skill::tilt_post_for_lcd_panel::apply(*stock, in);
    auto cands = skill::tilt_post_for_lcd_panel::recognize(*out.workpiece);
    ASSERT_FALSE(cands.empty());
    EXPECT_DOUBLE_EQ(cands.front().confidence, 1.0);
    EXPECT_EQ(cands.front().skill_id,
              std::string("tilt_post_for_lcd_panel"));
    EXPECT_EQ(
        cands.front().recovered_params.at("panel_size").get<std::string>(),
        std::string("PANEL-15"));
}

// ─── T5. CONTEXT: pillar past +Y edge triggers auto-extend ──────────────
TEST(SkillTiltPost, AutoExtendsBasePlateWhenPillarExceedsBackEdge)
{
    // Use a small 40 × 30 plate, place the pivot near the +Y edge so the
    // pillar extends past yMax = 30.
    auto stock = skill::createCuboidStock(40.0, 30.0, 5.0);

    skill::tilt_post_for_lcd_panel::Input in;
    in.base_face     = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.axis_dir      = gp_Dir(0, 0, -1);
    in.position_x_mm = 20.0;
    in.position_y_mm = 25.0;   // pivot 5 mm from +Y edge; pillar reaches ~33
    in.panel_size    = "PANEL-10";  // pillar_d_mm = 8

    auto out = skill::tilt_post_for_lcd_panel::apply(*stock, in);
    EXPECT_TRUE(out.signature.params.at("auto_extended_plate").get<bool>());
    EXPECT_TRUE(out.signature.pattern.at("auto_extended_plate").get<bool>());

    // Verify the bbox actually grew along +Y.
    double xMin, yMin, zMin, xMax, yMax, zMax;
    out.workpiece->boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    EXPECT_GT(yMax, 30.0);
}
