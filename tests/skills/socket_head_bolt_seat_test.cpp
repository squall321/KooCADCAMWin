// @lat: [[process/test-strategy#compound fastener-seat]]
//
// socket_head_bolt_seat — COMPOUND macro: through drill + counterbore +
// chamfer at entry.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/socket_head_bolt_seat.hpp"

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

// ─── 1. apply produces valid shape with N sub-features ───────────────────
TEST(SkillSocketHeadBoltSeat, ApplyProducesValidShapeWithMultipleSubFeatures)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 20.0);
    const int stockFaceCount = stock->faceCount();
    const double stockVol = volumeOf(stock->shape());

    skill::socket_head_bolt_seat::Input in;
    in.entry_face    = skill::FaceByNormal{ gp_Dir(0, 0, 1), 5.0, "largest" };
    in.position_x_mm = 25.0;
    in.position_y_mm = 25.0;
    in.fastener_size = "M4";
    in.head_slip_mm  = 0.4;

    auto out = skill::socket_head_bolt_seat::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    // M4: clearance 4.5, head 7.0 + 0.4 = 7.4 dia, head_height 4.0, seat depth = 4.5.
    const double pilotDia = 4.5, seatDia = 7.4, seatDep = 4.5, stockThickness = 20.0;
    const double pilotR = pilotDia / 2.0, seatR = seatDia / 2.0;
    const double pilotVol = M_PI * pilotR * pilotR * stockThickness;
    const double seatAnnVol = M_PI * (seatR * seatR - pilotR * pilotR) * seatDep;
    // Volume removed ≈ pilot + seat annulus (chamfer is tiny — ignore).
    const double approx = pilotVol + seatAnnVol;
    const double removed = stockVol - volumeOf(out.workpiece->shape());
    EXPECT_NEAR(removed, approx, approx * 0.08);

    EXPECT_GT(out.workpiece->faceCount(), stockFaceCount);

    EXPECT_TRUE(out.signature.pattern.at("is_compound").get<bool>());
    EXPECT_EQ(out.signature.pattern.at("subfeature_count").get<int>(), 3);
}

// ─── 2. validate rejects bad input ───────────────────────────────────────
TEST(SkillSocketHeadBoltSeat, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 20.0);

    // (a) Empty fastener
    {
        skill::socket_head_bolt_seat::Input in;
        in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
        in.fastener_size = "";
        auto r = skill::socket_head_bolt_seat::validate(*stock, in);
        EXPECT_FALSE(r.passed);
        EXPECT_THROW(skill::socket_head_bolt_seat::apply(*stock, in), skill::SkillError);
    }
    // (b) Thin stock
    {
        auto thin = skill::createCuboidStock(50.0, 50.0, 3.0);
        skill::socket_head_bolt_seat::Input in;
        in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
        in.position_x_mm = 25.0; in.position_y_mm = 25.0;
        in.fastener_size = "M6";   // head height 6 → seat 6.5 > 3
        auto r = skill::socket_head_bolt_seat::validate(*thin, in);
        EXPECT_FALSE(r.passed);
        bool found = false;
        for (const auto& f : r.findings)
            if (f.code == "DFM-SHCS-THICKNESS") { found = true; break; }
        EXPECT_TRUE(found);
    }
}

// ─── 3. Signature records compound kind ──────────────────────────────────
TEST(SkillSocketHeadBoltSeat, SignatureRecordsCompoundKind)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 20.0);

    skill::socket_head_bolt_seat::Input in;
    in.entry_face    = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 25.0;
    in.position_y_mm = 25.0;
    in.fastener_size = "M3";

    auto out = skill::socket_head_bolt_seat::apply(*stock, in);
    EXPECT_EQ(out.signature.pattern.at("kind").get<std::string>(),
              std::string("socket_head_bolt_seat"));
    EXPECT_TRUE(out.signature.pattern.at("is_compound").get<bool>());
    EXPECT_GE(out.signature.pattern.at("subfeature_count").get<int>(), 2);
}

// ─── 4. recognize replays metadata at confidence 1.0 ─────────────────────
TEST(SkillSocketHeadBoltSeat, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 20.0);

    skill::socket_head_bolt_seat::Input in;
    in.entry_face    = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 25.0;
    in.position_y_mm = 25.0;
    in.fastener_size = "M5";

    auto out = skill::socket_head_bolt_seat::apply(*stock, in);
    auto cands = skill::socket_head_bolt_seat::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_DOUBLE_EQ(cands[0].confidence, 1.0);
    EXPECT_EQ(cands[0].recovered_params.at("fastener_size").get<std::string>(),
              std::string("M5"));
}

// ─── 5. Specific: seat dia is head dia + slip ────────────────────────────
TEST(SkillSocketHeadBoltSeat, SeatDiameterMatchesIsoTablePlusSlip)
{
    // ISO 4762 head OD lookups.
    EXPECT_NEAR(skill::socket_head_bolt_seat::headDiameterFor("M3"), 5.5, 1e-3);
    EXPECT_NEAR(skill::socket_head_bolt_seat::headDiameterFor("M5"), 8.5, 1e-3);
    EXPECT_NEAR(skill::socket_head_bolt_seat::headHeightFor   ("M4"), 4.0, 1e-3);

    auto stock = skill::createCuboidStock(50.0, 50.0, 20.0);
    skill::socket_head_bolt_seat::Input in;
    in.entry_face    = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 25.0; in.position_y_mm = 25.0;
    in.fastener_size = "M4";
    in.head_slip_mm  = 0.4;
    auto out = skill::socket_head_bolt_seat::apply(*stock, in);

    // seat dia = head dia (7.0) + slip (0.4) = 7.4
    EXPECT_NEAR(out.signature.pattern.at("seat_dia_mm").get<double>(), 7.4, 1e-6);
    // seat depth = head height (4.0) + 0.5 mm extra = 4.5
    EXPECT_NEAR(out.signature.pattern.at("seat_depth_mm").get<double>(), 4.5, 1e-6);
    EXPECT_NEAR(out.signature.pattern.at("pilot_dia_mm").get<double>(), 4.5, 1e-6);
}
