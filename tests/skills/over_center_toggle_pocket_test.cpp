// @lat: [[process/test-strategy#skill round-trip]]

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/over_center_toggle_pocket.hpp"

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
// Volume math (Test 1, toggle_class = "TC-A40"):
//   spec: pivot_dia=5, slot_length=18, slot_width=3, anchor_dia=3, stop_width=1.5
//   pocket_depth = 10 mm
//
//   pivot bore:   π × 2.5² × 10  ≈ 196.3 mm³
//   cam slot:     18 × 3 × 10    = 540   mm³
//   over-center stop: stopL × (stop_width × 3) × (pocket_depth × 0.6)
//                    = (slot_width × 1.5) × (stop_width × 3) × 6
//                    = 4.5 × 4.5 × 6 = 121.5 mm³
//   spring anchor: π × 1.5² × 10 ≈ 70.7 mm³
//   approx total: ≈ 928.5 mm³
TEST(SkillOverCenterTogglePocket, ApplyRemovesExpectedVolume)
{
    auto stock = skill::createCuboidStock(80.0, 60.0, 30.0);

    skill::over_center_toggle_pocket::Input in;
    in.entry_face      = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.pivot_x_mm      = 30.0;
    in.pivot_y_mm      = 30.0;
    in.pocket_depth_mm = 10.0;
    in.slot_angle_deg  = 0.0;
    in.toggle_class    = "TC-A40";

    auto out = skill::over_center_toggle_pocket::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    const double Vpivot  = M_PI * 2.5 * 2.5 * 10.0;  // ~196.3
    const double Vslot   = 18.0 * 3.0 * 10.0;        // 540
    const double Vstop   = (3.0 * 1.5) * (1.5 * 3.0) * (10.0 * 0.6);  // 121.5
    const double Vanchor = M_PI * 1.5 * 1.5 * 10.0;  // ~70.7
    const double approx  = Vpivot + Vslot + Vstop + Vanchor;

    const double removed = volumeOf(stock->shape()) - volumeOf(out.workpiece->shape());
    EXPECT_NEAR(removed, approx, approx * 0.20);

    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_EQ(out.signature.skill_id, std::string("over_center_toggle_pocket"));
}

// ─── T2: validate rejects unknown spec key + apply throws ────────────────
TEST(SkillOverCenterTogglePocket, RejectsUnknownSpecKey)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 20.0);

    skill::over_center_toggle_pocket::Input in;
    in.entry_face      = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.pivot_x_mm      = 25.0;
    in.pivot_y_mm      = 25.0;
    in.pocket_depth_mm = 10.0;
    in.toggle_class    = "TC-WHATEVER";

    auto r = skill::over_center_toggle_pocket::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-SPEC") { found = true; break; }
    EXPECT_TRUE(found);
    EXPECT_THROW(skill::over_center_toggle_pocket::apply(*stock, in), skill::SkillError);
}

// ─── T3: signature carries spec key + is_compound=true ───────────────────
TEST(SkillOverCenterTogglePocket, SignatureHasCompoundAndSpecKey)
{
    auto stock = skill::createCuboidStock(80.0, 60.0, 30.0);

    skill::over_center_toggle_pocket::Input in;
    in.entry_face      = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.pivot_x_mm      = 30.0;
    in.pivot_y_mm      = 30.0;
    in.pocket_depth_mm = 15.0;
    in.toggle_class    = "TC-A80";

    auto out = skill::over_center_toggle_pocket::apply(*stock, in);
    EXPECT_TRUE(out.signature.pattern["is_compound"].get<bool>());
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("over_center_toggle_pocket"));
    EXPECT_EQ(out.signature.pattern["toggle_class"].get<std::string>(),
              std::string("TC-A80"));
    EXPECT_EQ(out.signature.pattern["subfeature_count"].get<int>(), 4);
}

// ─── T4: recognize returns ≥ 1 candidate (metadata replay) ───────────────
TEST(SkillOverCenterTogglePocket, RecognizeFindsToggle)
{
    auto stock = skill::createCuboidStock(80.0, 60.0, 30.0);

    skill::over_center_toggle_pocket::Input in;
    in.entry_face      = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.pivot_x_mm      = 30.0;
    in.pivot_y_mm      = 30.0;
    in.pocket_depth_mm = 10.0;
    in.toggle_class    = "TC-A40";

    auto out   = skill::over_center_toggle_pocket::apply(*stock, in);
    auto cands = skill::over_center_toggle_pocket::recognize(*out.workpiece);
    ASSERT_FALSE(cands.empty());
    EXPECT_EQ(cands[0].recovered_params["toggle_class"].get<std::string>(),
              std::string("TC-A40"));
}

// ─── T5: DFM-PIVOT depth rule per DIN 6840 (depth ≥ 1.5 × pivot_dia) ─────
TEST(SkillOverCenterTogglePocket, DfmPivotDepthRule)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 20.0);

    skill::over_center_toggle_pocket::Input in;
    in.entry_face      = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.pivot_x_mm      = 25.0;
    in.pivot_y_mm      = 25.0;
    in.pocket_depth_mm = 3.0;       // too shallow for TC-A80 (needs ≥ 12.0)
    in.toggle_class    = "TC-A80";

    auto r = skill::over_center_toggle_pocket::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-PIVOT") { found = true; break; }
    EXPECT_TRUE(found);

    // Spec lookup
    const auto* p = skill::over_center_toggle_pocket::lookupToggleSpec("TC-A80");
    ASSERT_NE(p, nullptr);
    EXPECT_NEAR(p->pivot_dia_mm,    8.0,  1e-6);
    EXPECT_NEAR(p->slot_length_mm, 30.0,  1e-6);
}
