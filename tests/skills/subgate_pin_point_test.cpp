// @lat: [[process/test-strategy#skill round-trip]]
//
// subgate_pin_point — tapered conical gate + cylindrical land at parting line.
//
// Cases (5):
//   1. ApplyRemovesVolume.
//   2. ValidateRejectsBadTaper.
//   3. ValidateRejectsTinyLand.
//   4. SignatureRecordsCompoundKind.
//   5. RecognizeReplaysHistory.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/subgate_pin_point.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

using namespace koocadcam;

namespace {
double volumeOf(const TopoDS_Shape& s)
{
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}
}  // namespace

// ─── 1. Gate removes volume ──────────────────────────────────────────────
TEST(SkillSubgatePinPoint, ApplyRemovesVolume)
{
    auto stock = skill::createCuboidStock(40.0, 30.0, 20.0);
    const double volBefore = volumeOf(stock->shape());

    skill::subgate_pin_point::Input in;
    in.cavity_entry_x_mm = 20.0;
    in.cavity_entry_y_mm = 15.0;
    in.gate_dia_mm       = 1.0;
    in.land_length_mm    = 1.0;
    in.taper_angle_deg   = 8.0;

    auto out = skill::subgate_pin_point::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    const double volAfter = volumeOf(out.workpiece->shape());
    EXPECT_LT(volAfter, volBefore);
    EXPECT_GT(volBefore - volAfter, 0.05);
    EXPECT_EQ(out.signature.skill_id, std::string("subgate_pin_point"));
}

// ─── 2. DFM rejects bad taper ────────────────────────────────────────────
TEST(SkillSubgatePinPoint, ValidateRejectsBadTaper)
{
    auto stock = skill::createCuboidStock(40.0, 30.0, 20.0);

    skill::subgate_pin_point::Input in;
    in.cavity_entry_x_mm = 20.0;
    in.cavity_entry_y_mm = 15.0;
    in.gate_dia_mm       = 1.0;
    in.land_length_mm    = 1.0;
    in.taper_angle_deg   = 20.0;        // > 15

    auto r = skill::subgate_pin_point::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool sawCode = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-SUBGATE-TAPER") { sawCode = true; break; }
    EXPECT_TRUE(sawCode);
    EXPECT_THROW(skill::subgate_pin_point::apply(*stock, in),
                 skill::SkillError);
}

// ─── 3. DFM rejects tiny land ────────────────────────────────────────────
TEST(SkillSubgatePinPoint, ValidateRejectsTinyLand)
{
    auto stock = skill::createCuboidStock(40.0, 30.0, 20.0);

    skill::subgate_pin_point::Input in;
    in.cavity_entry_x_mm = 20.0;
    in.cavity_entry_y_mm = 15.0;
    in.gate_dia_mm       = 1.0;
    in.land_length_mm    = 0.2;         // < 0.5
    in.taper_angle_deg   = 8.0;

    auto r = skill::subgate_pin_point::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool sawCode = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-SUBGATE-LAND") { sawCode = true; break; }
    EXPECT_TRUE(sawCode);
}

// ─── 4. Signature records compound + kind ────────────────────────────────
TEST(SkillSubgatePinPoint, SignatureRecordsCompoundKind)
{
    auto stock = skill::createCuboidStock(40.0, 30.0, 20.0);

    skill::subgate_pin_point::Input in;
    in.cavity_entry_x_mm = 20.0;
    in.cavity_entry_y_mm = 15.0;
    in.gate_dia_mm       = 1.0;
    in.land_length_mm    = 1.0;
    in.taper_angle_deg   = 8.0;
    auto out = skill::subgate_pin_point::apply(*stock, in);
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("subgate_pin_point"));
    EXPECT_TRUE(out.signature.pattern["is_compound"].get<bool>());
    EXPECT_EQ(out.signature.pattern["mold_feature_type"].get<std::string>(),
              std::string("subgate_pin_point"));
    EXPECT_GT(out.signature.pattern["derived_volume_removed"].get<double>(),
              0.0);
    EXPECT_GT(out.signature.pattern["taper_height_mm"].get<double>(), 0.0);
}

// ─── 5. Recognize replays history ────────────────────────────────────────
TEST(SkillSubgatePinPoint, RecognizeReplaysHistory)
{
    auto stock = skill::createCuboidStock(40.0, 30.0, 20.0);

    skill::subgate_pin_point::Input in;
    in.cavity_entry_x_mm = 20.0;
    in.cavity_entry_y_mm = 15.0;
    in.gate_dia_mm       = 1.2;
    in.land_length_mm    = 1.0;
    in.taper_angle_deg   = 7.0;
    auto out = skill::subgate_pin_point::apply(*stock, in);

    auto cands = skill::subgate_pin_point::recognize(*out.workpiece);
    ASSERT_GE(cands.size(), 1u);
    EXPECT_GE(cands.front().confidence, 0.9);
    EXPECT_NEAR(cands.front().recovered_params["gate_dia_mm"].get<double>(),
                1.2, 0.01);
}
