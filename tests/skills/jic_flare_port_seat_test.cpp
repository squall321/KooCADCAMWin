// @lat: [[process/test-strategy#compound macro]]
//
// jic_flare_port_seat — 37° flare seat + straight UN/UNF thread + chamfer.
//
// Test cases:
//   1. apply removes volume & changes face count.
//   2. validate rejects unknown jic_size.
//   3. signature carries is_compound + subfeature_count == 3.
//   4. recognize replays metadata at confidence 1.0.
//   5. seat_small_dia_mm < thread_major_dia_mm (37° taper geometry).

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/jic_flare_port_seat.hpp"

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

namespace {
skill::jic_flare_port_seat::Input goodInput()
{
    skill::jic_flare_port_seat::Input in;
    in.face_id              = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm        = 30.0;
    in.position_y_mm        = 30.0;
    in.axis_dir             = gp_Dir(0, 0, -1);
    in.jic_size             = "-06";          // 9/16-18 UNF
    in.thread_engagement_mm = 0.0;            // auto (= thread_dia)
    in.chamfer_leg_mm       = 0.5;
    in.part_thickness_mm    = 30.0;
    return in;
}
}  // namespace

// ─── 1. apply removes volume & changes face count ─────────────────────────
TEST(SkillJicFlarePortSeat, ApplyProducesValidShapeWithMultipleSubFeatures)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 30.0);
    const double v0 = volumeOf(stock->shape());
    const int    f0 = stock->faceCount();

    auto out = skill::jic_flare_port_seat::apply(*stock, goodInput());
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    const double v1 = volumeOf(out.workpiece->shape());
    EXPECT_GT(v0 - v1, 0.0);
    EXPECT_NE(out.workpiece->faceCount(), f0);
}

// ─── 2. validate rejects unknown jic_size ─────────────────────────────────
TEST(SkillJicFlarePortSeat, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 30.0);

    auto in = goodInput();
    in.jic_size = "-99";
    auto r = skill::jic_flare_port_seat::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-JIC-SIZE") { found = true; break; }
    EXPECT_TRUE(found);

    EXPECT_THROW(skill::jic_flare_port_seat::apply(*stock, in), skill::SkillError);
}

// ─── 3. Signature records compound kind ────────────────────────────────────
TEST(SkillJicFlarePortSeat, SignatureRecordsCompoundKind)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 30.0);
    auto out   = skill::jic_flare_port_seat::apply(*stock, goodInput());

    EXPECT_EQ(out.signature.skill_id, std::string("jic_flare_port_seat"));
    EXPECT_TRUE(out.signature.pattern.at("is_compound").get<bool>());
    EXPECT_GE(out.signature.pattern.at("subfeature_count").get<int>(), 2);
    EXPECT_EQ(out.signature.pattern.at("subfeature_count").get<int>(), 3);

    // Standard JIC -06 size lookup.
    double thread = 0.0, pitch = 0.0, seat = 0.0;
    EXPECT_TRUE(skill::jic_flare_port_seat::resolveJicSize(
        "-06", thread, pitch, seat));
    EXPECT_NEAR(thread, 14.29, 1e-6);
}

// ─── 4. Recognize replays metadata at confidence 1.0 ──────────────────────
TEST(SkillJicFlarePortSeat, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 30.0);
    auto out   = skill::jic_flare_port_seat::apply(*stock, goodInput());

    auto cands = skill::jic_flare_port_seat::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands.front().confidence, 1.0, 1e-9);
    EXPECT_EQ(cands.front().recovered_params["jic_size"].get<std::string>(),
              std::string("-06"));
}

// ─── 5. 37° seat geometry: seat_small_dia < thread_major_dia ──────────────
TEST(SkillJicFlarePortSeat, FlareGeometryRespectsStandard)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 30.0);
    auto out   = skill::jic_flare_port_seat::apply(*stock, goodInput());

    const double major   = out.signature.pattern.at("thread_major_dia_mm").get<double>();
    const double small   = out.signature.pattern.at("seat_small_dia_mm")  .get<double>();
    const double seatDep = out.signature.pattern.at("seat_depth_mm")      .get<double>();
    const double halfAng = out.signature.pattern.at("seat_half_angle_deg").get<double>();

    EXPECT_LT(small, major);            // taper goes inward
    EXPECT_GT(seatDep, 0.0);            // real seat depth
    EXPECT_NEAR(halfAng, 37.0, 1e-6);    // SAE J514 standard angle

    // Derived: seatDep = (major - small) / 2 / tan(37°).
    const double pred = (major - small) / 2.0 / std::tan(halfAng * M_PI / 180.0);
    EXPECT_NEAR(seatDep, pred, 1e-3);
}
