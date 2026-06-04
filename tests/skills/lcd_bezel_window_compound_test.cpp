// @lat: [[process/test-strategy#lcd_bezel_window_compound]]

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/lcd_bezel_window_compound.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

using namespace koocadcam;

namespace {
double volumeOf(const TopoDS_Shape& s) {
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}
}  // namespace

// ─── 1. apply REMOVES the recess volume (no vents) ──────────────────────
TEST(SkillLcdBezel, ApplyRemovesRecessVolume)
{
    auto stock = skill::createCuboidStock(120.0, 80.0, 5.0);
    const double v0 = volumeOf(stock->shape());

    skill::lcd_bezel_window_compound::Input in;
    in.face_id                     = 0;
    in.center_x_mm                 = 60.0;
    in.center_y_mm                 = 40.0;
    in.lcd_window_w_mm             = 60.0;
    in.lcd_window_h_mm             = 30.0;
    in.lcd_window_corner_radius_mm = 2.0;
    in.lcd_recess_depth_mm         = 2.0;
    in.vent_slot_count             = 0;

    auto out = skill::lcd_bezel_window_compound::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    const double v1 = volumeOf(out.workpiece->shape());

    // expected ≈ 60 × 30 × 2 = 3600 mm³ (minus a small rounded-corner factor).
    const double rectVol = 60.0 * 30.0 * 2.0;
    const double removed = v0 - v1;
    EXPECT_GT(removed, rectVol * 0.95);
    EXPECT_LT(removed, rectVol * 1.05);

    EXPECT_EQ(out.signature.skill_id,
              std::string("lcd_bezel_window_compound"));
}

// ─── 2. validate rejects bad corner radius / vent count ─────────────────
TEST(SkillLcdBezel, ValidateRejectsBadInputs)
{
    auto stock = skill::createCuboidStock(120.0, 80.0, 5.0);

    {   // (a) corner radius too small
        skill::lcd_bezel_window_compound::Input in;
        in.lcd_window_w_mm = 60.0; in.lcd_window_h_mm = 30.0;
        in.lcd_window_corner_radius_mm = 0.2;
        in.lcd_recess_depth_mm = 1.0; in.vent_slot_count = 0;
        auto r = skill::lcd_bezel_window_compound::validate(*stock, in);
        EXPECT_FALSE(r.passed);
        bool found = false;
        for (const auto& f : r.findings)
            if (f.code == "DFM-LCD-CORNER") found = true;
        EXPECT_TRUE(found);
        EXPECT_THROW(skill::lcd_bezel_window_compound::apply(*stock, in),
                     skill::SkillError);
    }
    {   // (b) corner radius too big
        skill::lcd_bezel_window_compound::Input in;
        in.lcd_window_w_mm = 20.0; in.lcd_window_h_mm = 10.0;
        in.lcd_window_corner_radius_mm = 6.0;   // > h/2
        in.lcd_recess_depth_mm = 1.0; in.vent_slot_count = 0;
        auto r = skill::lcd_bezel_window_compound::validate(*stock, in);
        EXPECT_FALSE(r.passed);
        bool found = false;
        for (const auto& f : r.findings)
            if (f.code == "DFM-LCD-CORNER-MAX") found = true;
        EXPECT_TRUE(found);
    }
    {   // (c) bad vent count
        skill::lcd_bezel_window_compound::Input in;
        in.lcd_window_w_mm = 60.0; in.lcd_window_h_mm = 30.0;
        in.lcd_window_corner_radius_mm = 2.0;
        in.lcd_recess_depth_mm = 1.0; in.vent_slot_count = 6;
        auto r = skill::lcd_bezel_window_compound::validate(*stock, in);
        EXPECT_FALSE(r.passed);
        bool found = false;
        for (const auto& f : r.findings)
            if (f.code == "DFM-LCD-VENTS") found = true;
        EXPECT_TRUE(found);
    }
}

// ─── 3. signature carries compound + dimensions ─────────────────────────
TEST(SkillLcdBezel, SignatureCarriesCompound)
{
    auto stock = skill::createCuboidStock(120.0, 100.0, 5.0);
    skill::lcd_bezel_window_compound::Input in;
    in.center_x_mm = 60.0; in.center_y_mm = 40.0;
    in.lcd_window_w_mm = 80.0; in.lcd_window_h_mm = 40.0;
    in.lcd_window_corner_radius_mm = 3.0;
    in.lcd_recess_depth_mm = 2.0;
    in.vent_slot_count = 4;
    auto out = skill::lcd_bezel_window_compound::apply(*stock, in);

    EXPECT_TRUE(out.signature.pattern.at("is_compound").get<bool>());
    EXPECT_EQ(out.signature.pattern.at("electronics_feature_type").get<std::string>(),
              std::string("lcd_bezel_window"));
    EXPECT_EQ(out.signature.pattern.at("vent_slot_count").get<int>(), 4);
    EXPECT_EQ(out.signature.pattern.at("subfeature_count").get<int>(), 5);  // recess + 4 vents
    EXPECT_NEAR(out.signature.pattern.at("lcd_window_w_mm").get<double>(),
                80.0, 1e-6);
    EXPECT_NEAR(out.signature.pattern.at("lcd_window_corner_radius_mm").get<double>(),
                3.0, 1e-6);
}

// ─── 4. recognize from metadata ──────────────────────────────────────────
TEST(SkillLcdBezel, RecognizeFromMetadata)
{
    auto stock = skill::createCuboidStock(120.0, 80.0, 5.0);
    skill::lcd_bezel_window_compound::Input in;
    in.center_x_mm = 60.0; in.center_y_mm = 40.0;
    in.lcd_window_w_mm = 60.0; in.lcd_window_h_mm = 30.0;
    in.lcd_window_corner_radius_mm = 2.0;
    in.lcd_recess_depth_mm = 1.5;
    in.vent_slot_count = 0;
    auto out = skill::lcd_bezel_window_compound::apply(*stock, in);

    auto cands = skill::lcd_bezel_window_compound::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_DOUBLE_EQ(cands[0].confidence, 1.0);
    EXPECT_NEAR(cands[0].recovered_params.at("lcd_window_w_mm").get<double>(),
                60.0, 1e-6);
}

// ─── 5. vents option ADDS more removed volume ───────────────────────────
TEST(SkillLcdBezel, VentSlotsAddRemovedVolume)
{
    auto stock = skill::createCuboidStock(160.0, 120.0, 5.0);
    const double v0 = volumeOf(stock->shape());

    skill::lcd_bezel_window_compound::Input no_vents;
    no_vents.center_x_mm = 80.0; no_vents.center_y_mm = 50.0;
    no_vents.lcd_window_w_mm = 80.0; no_vents.lcd_window_h_mm = 40.0;
    no_vents.lcd_window_corner_radius_mm = 2.0;
    no_vents.lcd_recess_depth_mm = 2.0;
    no_vents.vent_slot_count = 0;
    auto out_a = skill::lcd_bezel_window_compound::apply(*stock, no_vents);
    const double removed_a = v0 - volumeOf(out_a.workpiece->shape());

    skill::lcd_bezel_window_compound::Input with_vents = no_vents;
    with_vents.vent_slot_count = 4;
    auto stock_b = skill::createCuboidStock(160.0, 120.0, 5.0);
    auto out_b = skill::lcd_bezel_window_compound::apply(*stock_b, with_vents);
    const double removed_b = v0 - volumeOf(out_b.workpiece->shape());

    EXPECT_GT(removed_b, removed_a + 50.0)
        << "vented case must remove strictly more volume than non-vented";
}
