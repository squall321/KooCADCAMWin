// @lat: [[process/test-strategy#compound macro]]
//
// banjo_fitting_seat — through-bolt + 2 washer cbores + cross fluid passage.
//
// Test cases:
//   1. apply removes volume & changes face count.
//   2. validate rejects banjo_dia >= bolt_clearance_dia (DFM-BANJO-FLOW).
//   3. signature carries is_compound + subfeature_count == 4.
//   4. recognize replays metadata at confidence 1.0.
//   5. washer_seat_dia > bolt_clearance_dia (sealing-annulus invariant).

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/banjo_fitting_seat.hpp"

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
skill::banjo_fitting_seat::Input goodInput()
{
    skill::banjo_fitting_seat::Input in;
    in.face_id              = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm        = 25.0;
    in.position_y_mm        = 25.0;
    in.bolt_axis            = gp_Dir(0, 0, -1);
    in.bolt_M               = "M10";          // clearance 11.0 mm, washer 17.0 mm
    in.banjo_dia_mm         = 4.0;            // < 11.0 → valid
    in.banjo_axial_z_mm     = 6.0;            // mid-block cross-drill
    in.washer_seat_depth_mm = 1.5;
    in.part_thickness_mm    = 12.0;
    return in;
}
}  // namespace

// ─── 1. apply removes volume & changes face count ─────────────────────────
TEST(SkillBanjoFittingSeat, ApplyProducesValidShapeWithMultipleSubFeatures)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 12.0);
    const double v0 = volumeOf(stock->shape());
    const int    f0 = stock->faceCount();

    auto out = skill::banjo_fitting_seat::apply(*stock, goodInput());
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    const double v1 = volumeOf(out.workpiece->shape());
    EXPECT_GT(v0 - v1, 0.0);
    EXPECT_NE(out.workpiece->faceCount(), f0);
}

// ─── 2. validate rejects oversized banjo dia ──────────────────────────────
TEST(SkillBanjoFittingSeat, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 12.0);

    auto in = goodInput();
    in.banjo_dia_mm = 12.0;   // > bolt clearance (11.0) → sever bolt
    auto r = skill::banjo_fitting_seat::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-BANJO-FLOW") { found = true; break; }
    EXPECT_TRUE(found);

    EXPECT_THROW(skill::banjo_fitting_seat::apply(*stock, in), skill::SkillError);
}

// ─── 3. Signature records compound kind ────────────────────────────────────
TEST(SkillBanjoFittingSeat, SignatureRecordsCompoundKind)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 12.0);
    auto out   = skill::banjo_fitting_seat::apply(*stock, goodInput());

    EXPECT_EQ(out.signature.skill_id, std::string("banjo_fitting_seat"));
    EXPECT_TRUE(out.signature.pattern.at("is_compound").get<bool>());
    EXPECT_GE(out.signature.pattern.at("subfeature_count").get<int>(), 2);
    EXPECT_EQ(out.signature.pattern.at("subfeature_count").get<int>(), 4);

    // Standard bolt size lookup.
    double clr = 0.0, ws = 0.0;
    EXPECT_TRUE(skill::banjo_fitting_seat::resolveBoltM("M10", clr, ws));
    EXPECT_NEAR(clr, 11.0, 1e-6);
    EXPECT_NEAR(ws,  17.0, 1e-6);
}

// ─── 4. Recognize replays metadata at confidence 1.0 ──────────────────────
TEST(SkillBanjoFittingSeat, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 12.0);
    auto out   = skill::banjo_fitting_seat::apply(*stock, goodInput());

    auto cands = skill::banjo_fitting_seat::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands.front().confidence, 1.0, 1e-9);
    EXPECT_EQ(cands.front().recovered_params["bolt_M"].get<std::string>(),
              std::string("M10"));
}

// ─── 5. Sealing-annulus invariant: washer_seat_dia > bolt_clearance ────────
TEST(SkillBanjoFittingSeat, WasherSeatLargerThanBoltClearance)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 12.0);
    auto out   = skill::banjo_fitting_seat::apply(*stock, goodInput());

    const double clr = out.signature.pattern.at("bolt_clearance_dia_mm").get<double>();
    const double ws  = out.signature.pattern.at("washer_seat_dia_mm").get<double>();

    EXPECT_GT(ws, clr);                        // real seating annulus
    EXPECT_GT(ws - clr, 2.0);                  // > 1 mm radial band each side
    // Cross-drill must be < bolt clearance to leave bolt intact (DFM gate).
    const double bd = out.signature.pattern.at("banjo_dia_mm").get<double>();
    EXPECT_LT(bd, clr);
}
