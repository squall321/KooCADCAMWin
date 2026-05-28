// @lat: [[process/test-strategy#skill round-trip]]
//
// mold_gate — small subtractive feature representing the injection point.
//
// Cases (7):
//   1. pin_gate cuts a straight cylinder, removing the right small volume.
//   2. submarine_gate cuts an angled cylinder (still removes volume).
//   3. fan_gate cuts a flat box.
//   4. DFM rejects size outside [0.3, 2.0] mm.
//   5. DFM rejects unknown gate_type.
//   6. DFM warns when length < 2 × dia (pin/submarine).
//   7. recognize replays history.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/mold_gate.hpp"

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

// ─── 1. pin_gate cuts cylindrical channel ─────────────────────────────────
TEST(SkillMoldGate, ApplyPinGate)
{
    auto stock = skill::createCuboidStock(20.0, 20.0, 5.0);
    const double stockVol = volumeOf(stock->shape());

    skill::mold_gate::Input in;
    in.entry_face      = skill::FaceByNormal{ gp_Dir(0, 0, 1), 5.0, "largest" };
    in.position_x_mm   = 10.0;
    in.position_y_mm   = 10.0;
    in.gate_type       = "pin_gate";
    in.dia_or_width_mm = 0.8;
    in.length_mm       = 2.0;

    auto out = skill::mold_gate::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    const double removed = stockVol - volumeOf(out.workpiece->shape());
    const double r = 0.4;
    const double approx = M_PI * r * r * 2.0;
    EXPECT_GT(removed, 0.0);
    EXPECT_NEAR(removed, approx, approx * 0.30);     // allow generous tol

    EXPECT_EQ(out.signature.pattern["gate_type"].get<std::string>(),
              std::string("pin_gate"));
}

// ─── 2. submarine_gate cuts angled cylinder ───────────────────────────────
TEST(SkillMoldGate, ApplySubmarineGate)
{
    auto stock = skill::createCuboidStock(20.0, 20.0, 5.0);
    const double stockVol = volumeOf(stock->shape());

    skill::mold_gate::Input in;
    in.entry_face      = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm   = 10.0;
    in.position_y_mm   = 10.0;
    in.gate_type       = "submarine_gate";
    in.dia_or_width_mm = 1.0;
    in.length_mm       = 3.0;

    auto out = skill::mold_gate::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    const double removed = stockVol - volumeOf(out.workpiece->shape());
    EXPECT_GT(removed, 0.0);
    const double r = 0.5;
    const double approx = M_PI * r * r * 3.0;
    EXPECT_LT(removed, approx * 1.5);
    EXPECT_GT(removed, approx * 0.3);
}

// ─── 3. fan_gate cuts a flat box ──────────────────────────────────────────
TEST(SkillMoldGate, ApplyFanGate)
{
    auto stock = skill::createCuboidStock(20.0, 20.0, 5.0);
    const double stockVol = volumeOf(stock->shape());

    skill::mold_gate::Input in;
    in.entry_face      = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm   = 10.0;
    in.position_y_mm   = 10.0;
    in.gate_type       = "fan_gate";
    in.dia_or_width_mm = 1.5;
    in.length_mm       = 3.0;

    auto out = skill::mold_gate::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    const double removed = stockVol - volumeOf(out.workpiece->shape());
    EXPECT_GT(removed, 0.0);
}

// ─── 4. DFM rejects size outside [0.3, 2.0] ───────────────────────────────
TEST(SkillMoldGate, ValidateRejectsOutOfRangeSize)
{
    auto stock = skill::createCuboidStock(20.0, 20.0, 5.0);

    skill::mold_gate::Input in;
    in.entry_face      = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm   = 10.0;
    in.position_y_mm   = 10.0;
    in.gate_type       = "pin_gate";
    in.dia_or_width_mm = 3.0;     // > 2.0
    in.length_mm       = 2.0;

    auto r = skill::mold_gate::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool sawCode = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-GATE-SIZE") { sawCode = true; break; }
    EXPECT_TRUE(sawCode);

    EXPECT_THROW(skill::mold_gate::apply(*stock, in), skill::SkillError);
}

// ─── 5. DFM rejects unknown gate_type ─────────────────────────────────────
TEST(SkillMoldGate, ValidateRejectsUnknownType)
{
    auto stock = skill::createCuboidStock(20.0, 20.0, 5.0);

    skill::mold_gate::Input in;
    in.entry_face      = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm   = 10.0;
    in.position_y_mm   = 10.0;
    in.gate_type       = "weird_gate";
    in.dia_or_width_mm = 1.0;
    in.length_mm       = 2.0;

    auto r = skill::mold_gate::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool sawCode = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-INPUT" && f.message.find("gate_type") != std::string::npos)
            sawCode = true;
    EXPECT_TRUE(sawCode);
}

// ─── 6. DFM warns on short pin gate ───────────────────────────────────────
TEST(SkillMoldGate, ValidateWarnsOnShortPinGate)
{
    auto stock = skill::createCuboidStock(20.0, 20.0, 5.0);

    skill::mold_gate::Input in;
    in.entry_face      = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm   = 10.0;
    in.position_y_mm   = 10.0;
    in.gate_type       = "pin_gate";
    in.dia_or_width_mm = 1.0;
    in.length_mm       = 1.0;    // 1.0 < 2 × 1.0

    auto r = skill::mold_gate::validate(*stock, in);
    EXPECT_TRUE(r.passed);
    bool sawWarn = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-GATE-LEN" && f.severity == "warning") { sawWarn = true; break; }
    EXPECT_TRUE(sawWarn);
}

// ─── 7. Recognize replays history ─────────────────────────────────────────
TEST(SkillMoldGate, RecognizeReplaysHistory)
{
    auto stock = skill::createCuboidStock(20.0, 20.0, 5.0);

    skill::mold_gate::Input in;
    in.entry_face      = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm   = 8.0;
    in.position_y_mm   = 12.0;
    in.gate_type       = "pin_gate";
    in.dia_or_width_mm = 0.8;
    in.length_mm       = 2.0;

    auto out = skill::mold_gate::apply(*stock, in);
    auto cands = skill::mold_gate::recognize(*out.workpiece);
    ASSERT_GE(cands.size(), 1u);

    const auto& c = cands.front();
    EXPECT_GT(c.confidence, 0.7);
    EXPECT_EQ(c.recovered_params["gate_type"].get<std::string>(),
              std::string("pin_gate"));
    EXPECT_NEAR(c.recovered_params["dia_or_width_mm"].get<double>(), 0.8, 0.05);
    EXPECT_NEAR(c.recovered_params["position_x_mm"].get<double>(), 8.0, 0.05);
}
