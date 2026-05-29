// @lat: [[process/test-strategy#skill round-trip]]

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/detented_position_slider.hpp"

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
// Volume math (Test 1, detent_class = "STD_5"):
//   spec: detent_count=5, dimple_dia=2.0, dimple_depth=0.6
//   slot_len = 50, slot_w = 3, slot_d = 4
//
//   linear slot:   50 × 3 × 4 = 600 mm³
//   5 dimples:     5 × (π × 1.0² × 0.6) ≈ 5 × 1.884 = 9.42 mm³
//   approx total:  ≈ 609.4 mm³
TEST(SkillDetentedPositionSlider, ApplyRemovesExpectedVolume)
{
    auto stock = skill::createCuboidStock(100.0, 30.0, 15.0);

    skill::detented_position_slider::Input in;
    in.entry_face      = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.slot_start_x_mm = 25.0; in.slot_start_y_mm = 15.0;
    in.slot_end_x_mm   = 75.0; in.slot_end_y_mm   = 15.0;
    in.slot_width_mm   = 3.0;
    in.slot_depth_mm   = 4.0;
    in.detent_class    = "STD_5";

    auto out = skill::detented_position_slider::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    const double Vslot   = 50.0 * 3.0 * 4.0;        // 600
    const double Vdimple = 5.0 * (M_PI * 1.0 * 1.0 * 0.6);  // ~9.42
    const double approx  = Vslot + Vdimple;

    const double removed = volumeOf(stock->shape()) - volumeOf(out.workpiece->shape());
    EXPECT_NEAR(removed, approx, approx * 0.20);

    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_EQ(out.signature.skill_id, std::string("detented_position_slider"));
}

// ─── T2: validate rejects unknown spec key + apply throws ────────────────
TEST(SkillDetentedPositionSlider, RejectsUnknownSpecKey)
{
    auto stock = skill::createCuboidStock(60.0, 30.0, 15.0);

    skill::detented_position_slider::Input in;
    in.entry_face      = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.slot_start_x_mm = 10.0; in.slot_start_y_mm = 15.0;
    in.slot_end_x_mm   = 50.0; in.slot_end_y_mm   = 15.0;
    in.slot_width_mm   = 3.0;
    in.slot_depth_mm   = 3.0;
    in.detent_class    = "SUPER_FINE_99";

    auto r = skill::detented_position_slider::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-SPEC") { found = true; break; }
    EXPECT_TRUE(found);
    EXPECT_THROW(skill::detented_position_slider::apply(*stock, in), skill::SkillError);
}

// ─── T3: signature carries spec key + is_compound=true ───────────────────
TEST(SkillDetentedPositionSlider, SignatureHasCompoundAndSpecKey)
{
    auto stock = skill::createCuboidStock(120.0, 30.0, 15.0);

    skill::detented_position_slider::Input in;
    in.entry_face      = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.slot_start_x_mm = 20.0; in.slot_start_y_mm = 15.0;
    in.slot_end_x_mm   = 100.0; in.slot_end_y_mm  = 15.0;
    in.slot_width_mm   = 4.0;
    in.slot_depth_mm   = 4.0;
    in.detent_class    = "COARSE_10";

    auto out = skill::detented_position_slider::apply(*stock, in);
    EXPECT_TRUE(out.signature.pattern["is_compound"].get<bool>());
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("detented_position_slider"));
    EXPECT_EQ(out.signature.pattern["detent_class"].get<std::string>(),
              std::string("COARSE_10"));
    EXPECT_EQ(out.signature.pattern["detent_count"].get<int>(), 10);
    // subfeature_count = 1 slot + 10 dimples = 11
    EXPECT_EQ(out.signature.pattern["subfeature_count"].get<int>(), 11);
}

// ─── T4: recognize returns ≥ 1 candidate (metadata replay) ───────────────
TEST(SkillDetentedPositionSlider, RecognizeFindsDetentedSlider)
{
    auto stock = skill::createCuboidStock(100.0, 30.0, 15.0);

    skill::detented_position_slider::Input in;
    in.entry_face      = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.slot_start_x_mm = 25.0; in.slot_start_y_mm = 15.0;
    in.slot_end_x_mm   = 75.0; in.slot_end_y_mm   = 15.0;
    in.slot_width_mm   = 3.0;
    in.slot_depth_mm   = 4.0;
    in.detent_class    = "STD_5";

    auto out   = skill::detented_position_slider::apply(*stock, in);
    auto cands = skill::detented_position_slider::recognize(*out.workpiece);
    ASSERT_FALSE(cands.empty());
    EXPECT_EQ(cands[0].recovered_params["detent_class"].get<std::string>(),
              std::string("STD_5"));
}

// ─── T5: DFM-DIM-PITCH fires when slot is too short for the detent class
TEST(SkillDetentedPositionSlider, DfmDimplePitchFires)
{
    // FINE_3 has dimple_dia 1.5; pitch must be ≥ 3.0 mm.
    // With detent_count = 3, slot_len needs to be ≥ 6 mm.
    // We use slot_len = 4 to trigger DFM-DIM-PITCH.
    auto stock = skill::createCuboidStock(20.0, 20.0, 10.0);

    skill::detented_position_slider::Input in;
    in.entry_face      = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.slot_start_x_mm = 8.0;  in.slot_start_y_mm = 10.0;
    in.slot_end_x_mm   = 12.0; in.slot_end_y_mm   = 10.0;   // slot_len = 4
    in.slot_width_mm   = 2.0;
    in.slot_depth_mm   = 2.0;
    in.detent_class    = "FINE_3";

    auto r = skill::detented_position_slider::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-DIM-PITCH") { found = true; break; }
    EXPECT_TRUE(found);

    // Spec table integrity:
    const auto* p = skill::detented_position_slider::lookupDetentSpec("FINE_3");
    ASSERT_NE(p, nullptr);
    EXPECT_EQ(p->detent_count,        3);
    EXPECT_NEAR(p->dimple_dia_mm,    1.5, 1e-6);
    EXPECT_NEAR(p->dimple_depth_mm,  0.4, 1e-6);
}
