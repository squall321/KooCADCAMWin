// @lat: [[process/test-strategy#skill round-trip]]

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/cam_actuated_slider.hpp"

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
// Volume math (Test 1, cam_class = "STD_M5"):
//   spec: slot_w=5, pivot_dia=5, notch_w=2.5, throw=8
//   slot_len = 40, slot_depth = 5
//
//   linear slot:   slot_len × slot_w × slot_depth   = 40 × 5 × 5 = 1000 mm³
//   pivot bore:    π × 2.5² × 5 ≈ 98.2 mm³
//   cam notch:     notch_w × (slot_w × 1.5) × (slot_depth × 1.2)
//                = 2.5 × 7.5 × 6 = 112.5 mm³
//   approx total: ≈ 1210.7 mm³
TEST(SkillCamActuatedSlider, ApplyRemovesExpectedVolume)
{
    auto stock = skill::createCuboidStock(80.0, 60.0, 20.0);

    skill::cam_actuated_slider::Input in;
    in.entry_face      = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.slot_start_x_mm = 20.0; in.slot_start_y_mm = 30.0;
    in.slot_end_x_mm   = 60.0; in.slot_end_y_mm   = 30.0;
    in.slot_depth_mm   = 5.0;
    in.pivot_offset_mm = 8.0;   // > (5+5)/2 + 0.5 = 5.5, OK
    in.cam_class       = "STD_M5";

    auto out = skill::cam_actuated_slider::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    const double Vslot  = 40.0 * 5.0 * 5.0;                  // 1000
    const double Vpivot = M_PI * 2.5 * 2.5 * 5.0;            // ~98.2
    const double Vnotch = 2.5 * 7.5 * 6.0;                   // 112.5
    const double approx = Vslot + Vpivot + Vnotch;

    const double removed = volumeOf(stock->shape()) - volumeOf(out.workpiece->shape());
    EXPECT_NEAR(removed, approx, approx * 0.20);

    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_EQ(out.signature.skill_id, std::string("cam_actuated_slider"));
}

// ─── T2: validate rejects unknown spec key + apply throws ────────────────
TEST(SkillCamActuatedSlider, RejectsUnknownSpecKey)
{
    auto stock = skill::createCuboidStock(50.0, 30.0, 15.0);

    skill::cam_actuated_slider::Input in;
    in.entry_face      = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.slot_start_x_mm = 10.0; in.slot_start_y_mm = 15.0;
    in.slot_end_x_mm   = 40.0; in.slot_end_y_mm   = 15.0;
    in.slot_depth_mm   = 3.0;
    in.pivot_offset_mm = 8.0;
    in.cam_class       = "GIANT_M999";

    auto r = skill::cam_actuated_slider::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-SPEC") { found = true; break; }
    EXPECT_TRUE(found);
    EXPECT_THROW(skill::cam_actuated_slider::apply(*stock, in), skill::SkillError);
}

// ─── T3: signature carries spec key + is_compound=true ───────────────────
TEST(SkillCamActuatedSlider, SignatureHasCompoundAndSpecKey)
{
    auto stock = skill::createCuboidStock(80.0, 60.0, 20.0);

    skill::cam_actuated_slider::Input in;
    in.entry_face      = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.slot_start_x_mm = 20.0; in.slot_start_y_mm = 30.0;
    in.slot_end_x_mm   = 60.0; in.slot_end_y_mm   = 30.0;
    in.slot_depth_mm   = 5.0;
    in.pivot_offset_mm = 12.0;
    in.cam_class       = "HEAVY_M8";

    auto out = skill::cam_actuated_slider::apply(*stock, in);
    EXPECT_TRUE(out.signature.pattern["is_compound"].get<bool>());
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("cam_actuated_slider"));
    EXPECT_EQ(out.signature.pattern["cam_class"].get<std::string>(),
              std::string("HEAVY_M8"));
    EXPECT_EQ(out.signature.pattern["subfeature_count"].get<int>(), 3);
}

// ─── T4: recognize returns ≥ 1 candidate (metadata replay) ───────────────
TEST(SkillCamActuatedSlider, RecognizeFindsSlider)
{
    auto stock = skill::createCuboidStock(80.0, 60.0, 20.0);

    skill::cam_actuated_slider::Input in;
    in.entry_face      = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.slot_start_x_mm = 20.0; in.slot_start_y_mm = 30.0;
    in.slot_end_x_mm   = 60.0; in.slot_end_y_mm   = 30.0;
    in.slot_depth_mm   = 5.0;
    in.pivot_offset_mm = 8.0;
    in.cam_class       = "STD_M5";

    auto out   = skill::cam_actuated_slider::apply(*stock, in);
    auto cands = skill::cam_actuated_slider::recognize(*out.workpiece);
    ASSERT_FALSE(cands.empty());
    EXPECT_EQ(cands[0].recovered_params["cam_class"].get<std::string>(),
              std::string("STD_M5"));
}

// ─── T5: DFM-CAM-CLR fires when pivot too close to slot wall ─────────────
TEST(SkillCamActuatedSlider, DfmCamClearanceFires)
{
    auto stock = skill::createCuboidStock(80.0, 60.0, 20.0);

    skill::cam_actuated_slider::Input in;
    in.entry_face      = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.slot_start_x_mm = 20.0; in.slot_start_y_mm = 30.0;
    in.slot_end_x_mm   = 60.0; in.slot_end_y_mm   = 30.0;
    in.slot_depth_mm   = 5.0;
    in.pivot_offset_mm = 2.0;             // too small — (5+5)/2 + 0.5 = 5.5 required
    in.cam_class       = "STD_M5";

    auto r = skill::cam_actuated_slider::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-CAM-CLR") { found = true; break; }
    EXPECT_TRUE(found);

    // Spec lookup still works.
    const auto* p = skill::cam_actuated_slider::lookupCamSpec("STD_M5");
    ASSERT_NE(p, nullptr);
    EXPECT_NEAR(p->pivot_dia_mm, 5.0, 1e-6);
}
