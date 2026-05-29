// @lat: [[process/test-strategy#skill round-trip]]
//
// linear_slider_track tests — verify compound chain behavior with real
// volume-delta assertions and spec-table replay.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/linear_slider_track.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

#include <cmath>

using namespace koocadcam;

namespace {
double volumeOf(const TopoDS_Shape& s) {
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}
}

// ─── T1: apply produces non-null shape with volume delta within ±20 % ────
//
// Volume math (Test 1 expected):
//   slot box volume = effLen × slot_w × slot_d
//     where effLen = slotLen − 2 × end_stop_thk
//   rib added back  = effLen × (0.4 × slot_w) × rib_h     (per spec GENERAL_H8 → 1.0 mm)
//   net removed     = slot_vol − rib_vol
//
// For slotLen=40, end_stop_thk=2, slot_w=6, slot_d=4:
//   effLen = 40 − 4 = 36
//   slot_vol = 36 × 6 × 4   = 864 mm³
//   rib_vol  = 36 × 2.4 × 1 = 86.4 mm³
//   approx_removed = 864 − 86.4 = 777.6 mm³
TEST(SkillLinearSliderTrack, ApplyRemovesExpectedVolume)
{
    auto stock = skill::createCuboidStock(80.0, 40.0, 10.0);

    skill::linear_slider_track::Input in;
    in.entry_face       = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.start_x_mm  = 20.0; in.start_y_mm = 20.0;
    in.end_x_mm    = 60.0; in.end_y_mm   = 20.0;
    in.slot_width_mm    = 6.0;
    in.slot_depth_mm    = 4.0;
    in.end_stop_thk_mm  = 2.0;
    in.track_class      = "GENERAL_H8";

    auto out = skill::linear_slider_track::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    const double effLen   = 40.0 - 2.0 * 2.0;          // 36
    const double slot_vol = effLen * 6.0 * 4.0;        // 864
    const double rib_vol  = effLen * (6.0 * 0.4) * 1.0;// 86.4 (rib_h = 1.0 for GENERAL_H8)
    const double approx   = slot_vol - rib_vol;        // 777.6

    const double removed = volumeOf(stock->shape()) - volumeOf(out.workpiece->shape());
    EXPECT_NEAR(removed, approx, approx * 0.20);

    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_EQ(out.signature.skill_id, std::string("linear_slider_track"));
}

// ─── T2: validate rejects unknown spec key + apply throws ────────────────
TEST(SkillLinearSliderTrack, RejectsUnknownSpecKey)
{
    auto stock = skill::createCuboidStock(50.0, 30.0, 10.0);

    skill::linear_slider_track::Input in;
    in.entry_face       = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.start_x_mm  = 10.0; in.start_y_mm = 15.0;
    in.end_x_mm    = 40.0; in.end_y_mm   = 15.0;
    in.slot_width_mm    = 5.0;
    in.slot_depth_mm    = 3.0;
    in.end_stop_thk_mm  = 1.5;
    in.track_class      = "UNKNOWN_XYZ";

    auto r = skill::linear_slider_track::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-SPEC") { found = true; break; }
    EXPECT_TRUE(found);
    EXPECT_THROW(skill::linear_slider_track::apply(*stock, in), skill::SkillError);
}

// ─── T3: signature carries spec key + is_compound=true ───────────────────
TEST(SkillLinearSliderTrack, SignatureHasCompoundAndSpecKey)
{
    auto stock = skill::createCuboidStock(80.0, 40.0, 10.0);

    skill::linear_slider_track::Input in;
    in.entry_face       = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.start_x_mm  = 20.0; in.start_y_mm = 20.0;
    in.end_x_mm    = 60.0; in.end_y_mm   = 20.0;
    in.slot_width_mm    = 5.0;
    in.slot_depth_mm    = 3.0;
    in.end_stop_thk_mm  = 1.5;
    in.track_class      = "PRECISION_H7";

    auto out = skill::linear_slider_track::apply(*stock, in);
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("linear_slider_track"));
    EXPECT_TRUE(out.signature.pattern["is_compound"].get<bool>());
    EXPECT_EQ(out.signature.pattern["track_class"].get<std::string>(),
              std::string("PRECISION_H7"));
    EXPECT_EQ(out.signature.pattern["subfeature_count"].get<int>(), 5);
}

// ─── T4: recognize returns ≥ 1 candidate with right spec key ─────────────
TEST(SkillLinearSliderTrack, RecognizeFindsTrack)
{
    auto stock = skill::createCuboidStock(80.0, 40.0, 10.0);

    skill::linear_slider_track::Input in;
    in.entry_face       = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.start_x_mm  = 20.0; in.start_y_mm = 20.0;
    in.end_x_mm    = 60.0; in.end_y_mm   = 20.0;
    in.slot_width_mm    = 6.0;
    in.slot_depth_mm    = 4.0;
    in.end_stop_thk_mm  = 2.0;
    in.track_class      = "GENERAL_H8";

    auto out   = skill::linear_slider_track::apply(*stock, in);
    auto cands = skill::linear_slider_track::recognize(*out.workpiece);
    ASSERT_FALSE(cands.empty());
    EXPECT_EQ(cands[0].skill_id, std::string("linear_slider_track"));
    EXPECT_EQ(cands[0].recovered_params["track_class"].get<std::string>(),
              std::string("GENERAL_H8"));
}

// ─── T5: spec table lookup returns correct rib_height per class ──────────
TEST(SkillLinearSliderTrack, SpecTableMatchesY14_5Fits)
{
    const auto* p = skill::linear_slider_track::lookupTrackSpec("PRECISION_H7");
    ASSERT_NE(p, nullptr);
    EXPECT_NEAR(p->slot_clearance_mm, 0.020, 1e-6);
    EXPECT_NEAR(p->rib_height_mm,     1.5,   1e-6);

    const auto* g = skill::linear_slider_track::lookupTrackSpec("GENERAL_H8");
    ASSERT_NE(g, nullptr);
    EXPECT_NEAR(g->rib_height_mm,     1.0,   1e-6);

    const auto* l = skill::linear_slider_track::lookupTrackSpec("LOOSE_H9");
    ASSERT_NE(l, nullptr);
    EXPECT_NEAR(l->slot_clearance_mm, 0.080, 1e-6);

    EXPECT_EQ(skill::linear_slider_track::lookupTrackSpec("BOGUS"), nullptr);
}
