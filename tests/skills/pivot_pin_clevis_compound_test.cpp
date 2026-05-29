// @lat: [[process/test-strategy#skill round-trip]]

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/pivot_pin_clevis_compound.hpp"

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
// Volume math (Test 1, clevis_size = "CL-06"):
//   spec: u_width=6, pin_dia=5, ear_thk=4, cotter=1.2, seat=7
//   earSpan = u_width × 3 = 18 mm
//
//   U pocket:    6 × 18 × u_depth = 6 × 18 × 8 = 864 mm³
//   pin bore:    π × (2.5)² × 18    ≈ 353.4 mm³
//   cotter slot: (pin_dia × 2) × cotter × cotter = 10 × 1.2 × 1.2 = 14.4 mm³
//   bushing seat annulus: π × (3.5² − 2.5²) × seat_depth (≈ear_thk × 0.3 = 1.2 mm)
//                      = π × 6.0 × 1.2 ≈ 22.6 mm³
//   approx removed total ≈ 864 + 353.4 + 14.4 + 22.6 ≈ 1254 mm³
//
// (Since cuts overlap somewhat near the pin/U intersection, the fuse-then-cut
// pipeline will produce a slightly LOWER total — within the ±20 % band.)
TEST(SkillPivotPinClevisCompound, ApplyRemovesExpectedVolume)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 30.0);

    skill::pivot_pin_clevis_compound::Input in;
    in.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.center_x_mm  = 30.0;
    in.center_y_mm  = 30.0;
    in.u_axis       = gp_Dir(0, 0, -1);
    in.pin_axis     = gp_Dir(0, 1, 0);
    in.u_depth_mm   = 8.0;
    in.clevis_size  = "CL-06";

    auto out = skill::pivot_pin_clevis_compound::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    const double earSpan = 6.0 * 3.0;                       // 18
    const double Vu     = 6.0 * earSpan * 8.0;              // 864
    const double Vpin   = M_PI * 2.5 * 2.5 * earSpan;       // ~353.4
    const double Vcot   = (5.0 * 2.0) * 1.2 * 1.2;          // 14.4
    const double Vseat  = M_PI * (3.5*3.5 - 2.5*2.5) * 1.2; // ~22.6
    const double approx = Vu + Vpin + Vcot + Vseat;

    const double removed = volumeOf(stock->shape()) - volumeOf(out.workpiece->shape());
    EXPECT_NEAR(removed, approx, approx * 0.20);

    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_EQ(out.signature.skill_id, std::string("pivot_pin_clevis_compound"));
}

// ─── T2: validate rejects unknown spec key + apply throws ────────────────
TEST(SkillPivotPinClevisCompound, RejectsUnknownSpecKey)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 20.0);

    skill::pivot_pin_clevis_compound::Input in;
    in.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.center_x_mm  = 25.0;
    in.center_y_mm  = 25.0;
    in.u_depth_mm   = 8.0;
    in.clevis_size  = "CL-99";   // unknown

    auto r = skill::pivot_pin_clevis_compound::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-SPEC") { found = true; break; }
    EXPECT_TRUE(found);
    EXPECT_THROW(skill::pivot_pin_clevis_compound::apply(*stock, in), skill::SkillError);
}

// ─── T3: signature carries spec key + is_compound=true ───────────────────
TEST(SkillPivotPinClevisCompound, SignatureHasCompoundAndSpecKey)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 30.0);

    skill::pivot_pin_clevis_compound::Input in;
    in.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.center_x_mm  = 30.0;
    in.center_y_mm  = 30.0;
    in.u_depth_mm   = 10.0;
    in.clevis_size  = "CL-08";

    auto out = skill::pivot_pin_clevis_compound::apply(*stock, in);
    EXPECT_TRUE(out.signature.pattern["is_compound"].get<bool>());
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("pivot_pin_clevis_compound"));
    EXPECT_EQ(out.signature.pattern["clevis_size"].get<std::string>(),
              std::string("CL-08"));
    EXPECT_EQ(out.signature.pattern["subfeature_count"].get<int>(), 4);
}

// ─── T4: recognize returns ≥ 1 candidate (via metadata replay) ───────────
TEST(SkillPivotPinClevisCompound, RecognizeFindsClevis)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 30.0);

    skill::pivot_pin_clevis_compound::Input in;
    in.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.center_x_mm  = 30.0;
    in.center_y_mm  = 30.0;
    in.u_depth_mm   = 8.0;
    in.clevis_size  = "CL-06";

    auto out   = skill::pivot_pin_clevis_compound::apply(*stock, in);
    auto cands = skill::pivot_pin_clevis_compound::recognize(*out.workpiece);
    ASSERT_FALSE(cands.empty());
    EXPECT_EQ(cands[0].recovered_params["clevis_size"].get<std::string>(),
              std::string("CL-06"));
}

// ─── T5: spec-table assertion — CL-08 dims per MIL-S-5556 ────────────────
TEST(SkillPivotPinClevisCompound, SpecTableMatchesMilS5556)
{
    const auto* p = skill::pivot_pin_clevis_compound::lookupClevisSpec("CL-08");
    ASSERT_NE(p, nullptr);
    EXPECT_NEAR(p->u_width_mm,      8.0,  1e-6);
    EXPECT_NEAR(p->pin_dia_mm,      6.0,  1e-6);
    EXPECT_NEAR(p->ear_thk_mm,      5.0,  1e-6);
    EXPECT_NEAR(p->cotter_slot_mm,  1.6,  1e-6);
    EXPECT_NEAR(p->seat_dia_mm,     10.0, 1e-6);
    // Ear-shear margin per MIL-S-5556 §4.3.2 (ear_thk ≥ pin_dia × 0.5):
    EXPECT_GE(p->ear_thk_mm, p->pin_dia_mm * 0.5);

    EXPECT_EQ(skill::pivot_pin_clevis_compound::lookupClevisSpec("CL-99"), nullptr);
}
