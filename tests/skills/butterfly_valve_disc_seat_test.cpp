// @lat: [[process/test-strategy#compound macro]]
//
// butterfly_valve_disc_seat — 5 sub-feature compound, API 609.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/butterfly_valve_disc_seat.hpp"

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
TEST(SkillButterflyValveDiscSeat, ApplyProducesValidVolumeDelta)
{
    auto stock = skill::createCuboidStock(80.0, 80.0, 80.0);
    skill::butterfly_valve_disc_seat::Input in;
    in.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1), 5.0, "largest" };
    in.center_x_mm  = 40.0;
    in.center_y_mm  = 40.0;
    in.center_z_mm  = 40.0;
    in.flow_bore_dia_mm      = 40.0;
    in.flow_bore_len_mm      = 50.0;
    in.seat_groove_id_mm     = 40.0;
    in.seat_groove_radial_depth_mm = 2.5;
    in.seat_groove_width_mm  = 4.0;
    in.stub_bearing_dia_mm   = 12.0;
    in.stub_bearing_depth_mm = 8.0;
    in.stem_port_dia_mm      = 10.0;
    in.body_y_span_mm        = 60.0;

    const double v0 = volumeOf(stock->shape());
    auto out = skill::butterfly_valve_disc_seat::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    const double v1 = volumeOf(out.workpiece->shape());

    // Expected:
    //   bore (through):  π × 20² × 80 = 100530
    //   seat (annular):  π × (22.5² − 20²) × 4 = 1335
    //   stub × 2:        2 × π × 6² × 8 = 1810
    //   port × 2 (through 80 mm): 2 × π × 5² × 80 = 12566
    // Overlaps: bore eats seat IDs; ports eat stub bearings (concentric).
    const double approx = 100530 + 1335 + 1810 + 12566 - 6000;
    const double diff = v0 - v1;
    EXPECT_GT(diff, 0.0);
    EXPECT_NEAR(diff, approx, approx * 0.25)
        << "removed=" << diff << " expected~" << approx;
}

// ─── T2: validate rejects bad inputs + apply throws ──────────────────────
TEST(SkillButterflyValveDiscSeat, ValidateRejectsBadDiscStemRatio)
{
    auto stock = skill::createCuboidStock(80.0, 80.0, 80.0);
    skill::butterfly_valve_disc_seat::Input in;
    in.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.center_x_mm  = 40.0;
    in.center_y_mm  = 40.0;
    in.center_z_mm  = 40.0;
    in.flow_bore_dia_mm      = 60.0;
    in.flow_bore_len_mm      = 50.0;
    in.seat_groove_id_mm     = 60.0;
    in.seat_groove_radial_depth_mm = 2.5;
    in.seat_groove_width_mm  = 4.0;
    in.stub_bearing_dia_mm   = 6.0;     // violates 0.15 × 60 = 9 mm rule
    in.stub_bearing_depth_mm = 8.0;
    in.stem_port_dia_mm      = 5.0;
    in.body_y_span_mm        = 60.0;

    auto r = skill::butterfly_valve_disc_seat::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-BFV-STEM-RATIO") { found = true; break; }
    EXPECT_TRUE(found);
    EXPECT_THROW(skill::butterfly_valve_disc_seat::apply(*stock, in),
                 skill::SkillError);
}

// ─── T3: signature carries compound + size info ──────────────────────────
TEST(SkillButterflyValveDiscSeat, SignatureRecordsCompound)
{
    auto stock = skill::createCuboidStock(80.0, 80.0, 80.0);
    skill::butterfly_valve_disc_seat::Input in;
    in.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.center_x_mm  = 40.0;
    in.center_y_mm  = 40.0;
    in.center_z_mm  = 40.0;
    in.flow_bore_dia_mm      = 40.0;
    in.flow_bore_len_mm      = 50.0;
    in.seat_groove_id_mm     = 40.0;
    in.seat_groove_radial_depth_mm = 2.5;
    in.seat_groove_width_mm  = 4.0;
    in.stub_bearing_dia_mm   = 12.0;
    in.stub_bearing_depth_mm = 8.0;
    in.stem_port_dia_mm      = 10.0;
    in.body_y_span_mm        = 60.0;

    auto out = skill::butterfly_valve_disc_seat::apply(*stock, in);
    EXPECT_EQ(out.signature.skill_id, std::string("butterfly_valve_disc_seat"));
    EXPECT_TRUE(out.signature.pattern.at("is_compound").get<bool>());
    EXPECT_EQ(out.signature.pattern.at("subfeature_count").get<int>(), 5);
    EXPECT_NEAR(out.signature.pattern.at("flow_bore_dia_mm").get<double>(),
                40.0, 1e-6);
    EXPECT_NEAR(out.signature.pattern.at("stub_bearing_dia_mm").get<double>(),
                12.0, 1e-6);
}

// ─── T4: recognize returns ≥1 candidate via metadata replay ──────────────
TEST(SkillButterflyValveDiscSeat, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(80.0, 80.0, 80.0);
    skill::butterfly_valve_disc_seat::Input in;
    in.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.center_x_mm  = 40.0;
    in.center_y_mm  = 40.0;
    in.center_z_mm  = 40.0;
    in.flow_bore_dia_mm      = 40.0;
    in.flow_bore_len_mm      = 50.0;
    in.seat_groove_id_mm     = 40.0;
    in.seat_groove_radial_depth_mm = 2.5;
    in.seat_groove_width_mm  = 4.0;
    in.stub_bearing_dia_mm   = 12.0;
    in.stub_bearing_depth_mm = 8.0;
    in.stem_port_dia_mm      = 10.0;
    in.body_y_span_mm        = 60.0;

    auto out = skill::butterfly_valve_disc_seat::apply(*stock, in);
    auto cands = skill::butterfly_valve_disc_seat::recognize(*out.workpiece);

    ASSERT_GE(cands.size(), 1u);
    EXPECT_EQ(cands.front().skill_id, std::string("butterfly_valve_disc_seat"));
    EXPECT_NEAR(cands.front().confidence, 1.0, 1e-9);
    EXPECT_NEAR(cands.front().recovered_params.at("flow_bore_dia_mm")
                .get<double>(), 40.0, 1e-6);
}

// ─── T5: stub bearing dia respects API 609 stem-ratio (≥ 15% of bore) ───
TEST(SkillButterflyValveDiscSeat, StubBearingMatchesApi609Ratio)
{
    auto stock = skill::createCuboidStock(80.0, 80.0, 80.0);
    skill::butterfly_valve_disc_seat::Input in;
    in.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.center_x_mm  = 40.0;
    in.center_y_mm  = 40.0;
    in.center_z_mm  = 40.0;
    in.flow_bore_dia_mm      = 40.0;
    in.flow_bore_len_mm      = 50.0;
    in.seat_groove_id_mm     = 40.0;
    in.seat_groove_radial_depth_mm = 2.5;
    in.seat_groove_width_mm  = 4.0;
    in.stub_bearing_dia_mm   = 12.0;       // 12 / 40 = 0.30 ≥ 0.15 ✓
    in.stub_bearing_depth_mm = 8.0;
    in.stem_port_dia_mm      = 10.0;
    in.body_y_span_mm        = 60.0;

    auto r = skill::butterfly_valve_disc_seat::validate(*stock, in);
    EXPECT_TRUE(r.passed);

    auto out = skill::butterfly_valve_disc_seat::apply(*stock, in);
    const double bore = out.signature.params.at("flow_bore_dia_mm").get<double>();
    const double stub = out.signature.params.at("stub_bearing_dia_mm").get<double>();
    EXPECT_GE(stub / bore, 0.15);
    const double port = out.signature.params.at("stem_port_dia_mm").get<double>();
    EXPECT_LE(port, stub);  // port fits inside bearing
}
