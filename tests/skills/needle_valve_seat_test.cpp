// @lat: [[process/test-strategy#compound macro]]
//
// needle_valve_seat — 4 sub-feature compound, ISA 75.11 small-Cv.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/needle_valve_seat.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using namespace koocadcam;

namespace {
double volumeOf(const TopoDS_Shape& s) {
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}
}  // namespace

// ─── T1: apply produces non-null shape with expected volume delta ────────
TEST(SkillNeedleValveSeat, ApplyProducesValidVolumeDelta)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 30.0);
    skill::needle_valve_seat::Input in;
    in.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1), 5.0, "largest" };
    in.position_x_mm = 20.0;
    in.position_y_mm = 20.0;
    in.pocket_entry_dia_mm  = 6.0;
    in.pocket_taper_deg     = 30.0;
    in.pocket_length_mm     = 8.0;
    in.control_bore_dia_mm  = 0.6;
    in.control_bore_depth_mm = 6.0;
    in.stem_boss_dia_mm     = 12.0;
    in.stem_boss_depth_mm   = 5.0;
    in.thread_relief_dia_mm = 7.0;
    in.thread_relief_depth_mm = 2.0;

    const double v0 = volumeOf(stock->shape());
    auto out = skill::needle_valve_seat::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    const double v1 = volumeOf(out.workpiece->shape());

    // Expected (mm³):
    //   coneV (frustum r1=3, r2=0.3, h=8):
    //     π × 8/3 × (9 + 0.9 + 0.09) ≈ 83.6
    //   controlBoreV: π × 0.3² × 6 ≈ 1.70
    //   stemBossV:    π × 6² × 5 ≈ 565.5
    //   reliefV:      π × 3.5² × 2 ≈ 76.97
    // Overlaps: relief inside boss column → ~ -77 net effect
    const double approx = 83.6 + 1.7 + 565.5 + 76.97 - 50.0;
    const double diff = v0 - v1;
    EXPECT_GT(diff, 0.0);
    EXPECT_NEAR(diff, approx, approx * 0.25)
        << "removed=" << diff << " expected~" << approx;
}

// ─── T2: validate rejects bad control bore + apply throws ────────────────
TEST(SkillNeedleValveSeat, ValidateRejectsBadControlBore)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 30.0);
    skill::needle_valve_seat::Input in;
    in.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 20.0;
    in.position_y_mm = 20.0;
    in.pocket_entry_dia_mm  = 6.0;
    in.pocket_taper_deg     = 30.0;
    in.pocket_length_mm     = 8.0;
    in.control_bore_dia_mm  = 2.0;    // > 1 mm — violates ISA 75.11 needle class
    in.control_bore_depth_mm = 6.0;
    in.stem_boss_dia_mm     = 12.0;
    in.stem_boss_depth_mm   = 5.0;
    in.thread_relief_dia_mm = 7.0;
    in.thread_relief_depth_mm = 2.0;

    auto r = skill::needle_valve_seat::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-NEEDLE-BORE-DIA") { found = true; break; }
    EXPECT_TRUE(found);
    EXPECT_THROW(skill::needle_valve_seat::apply(*stock, in),
                 skill::SkillError);
}

// ─── T3: signature carries compound info ─────────────────────────────────
TEST(SkillNeedleValveSeat, SignatureRecordsCompound)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 30.0);
    skill::needle_valve_seat::Input in;
    in.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 20.0;
    in.position_y_mm = 20.0;
    in.pocket_entry_dia_mm  = 6.0;
    in.pocket_taper_deg     = 30.0;
    in.pocket_length_mm     = 8.0;
    in.control_bore_dia_mm  = 0.6;
    in.control_bore_depth_mm = 6.0;
    in.stem_boss_dia_mm     = 12.0;
    in.stem_boss_depth_mm   = 5.0;
    in.thread_relief_dia_mm = 7.0;
    in.thread_relief_depth_mm = 2.0;

    auto out = skill::needle_valve_seat::apply(*stock, in);
    EXPECT_EQ(out.signature.skill_id, std::string("needle_valve_seat"));
    EXPECT_TRUE(out.signature.pattern.at("is_compound").get<bool>());
    EXPECT_EQ(out.signature.pattern.at("subfeature_count").get<int>(), 4);
    EXPECT_NEAR(out.signature.pattern.at("pocket_taper_deg").get<double>(),
                30.0, 1e-6);
    EXPECT_NEAR(out.signature.pattern.at("control_bore_dia_mm").get<double>(),
                0.6, 1e-6);
}

// ─── T4: recognize returns ≥1 candidate via metadata replay ──────────────
TEST(SkillNeedleValveSeat, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 30.0);
    skill::needle_valve_seat::Input in;
    in.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 20.0;
    in.position_y_mm = 20.0;
    in.pocket_entry_dia_mm  = 6.0;
    in.pocket_taper_deg     = 30.0;
    in.pocket_length_mm     = 8.0;
    in.control_bore_dia_mm  = 0.6;
    in.control_bore_depth_mm = 6.0;
    in.stem_boss_dia_mm     = 12.0;
    in.stem_boss_depth_mm   = 5.0;
    in.thread_relief_dia_mm = 7.0;
    in.thread_relief_depth_mm = 2.0;

    auto out = skill::needle_valve_seat::apply(*stock, in);
    auto cands = skill::needle_valve_seat::recognize(*out.workpiece);

    ASSERT_GE(cands.size(), 1u);
    EXPECT_EQ(cands.front().skill_id, std::string("needle_valve_seat"));
    EXPECT_NEAR(cands.front().confidence, 1.0, 1e-9);
    EXPECT_NEAR(cands.front().recovered_params.at("pocket_taper_deg")
                .get<double>(), 30.0, 1e-6);
}

// ─── T5: precision orifice flag — control_bore_dia < 1 mm sets is_precision_orifice ──
TEST(SkillNeedleValveSeat, PrecisionOrificeFlagged)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 30.0);
    skill::needle_valve_seat::Input in;
    in.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 20.0;
    in.position_y_mm = 20.0;
    in.pocket_entry_dia_mm  = 6.0;
    in.pocket_taper_deg     = 30.0;
    in.pocket_length_mm     = 8.0;
    in.control_bore_dia_mm  = 0.6;   // < 1.0 → precision
    in.control_bore_depth_mm = 6.0;
    in.stem_boss_dia_mm     = 12.0;
    in.stem_boss_depth_mm   = 5.0;
    in.thread_relief_dia_mm = 7.0;
    in.thread_relief_depth_mm = 2.0;

    auto out = skill::needle_valve_seat::apply(*stock, in);
    EXPECT_TRUE(out.signature.pattern.at("is_precision_orifice").get<bool>());
    // stem boss ≥ 1.4 × pocket entry (DFM stem-fit rule)
    const double bossDia   = out.signature.params.at("stem_boss_dia_mm").get<double>();
    const double pocketDia = out.signature.params.at("pocket_entry_dia_mm").get<double>();
    EXPECT_GE(bossDia, 1.4 * pocketDia);
}
