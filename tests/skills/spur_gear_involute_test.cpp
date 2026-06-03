// @lat: [[process/test-strategy#spur_gear_involute]]
//
// spur_gear_involute — REAL multi-cut involute spur gear.
// Test verifies tooth_count match + volume change > 0 + signature replay.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/spur_gear_involute.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

#include <cmath>

using namespace koocadcam;

namespace {

double volumeOf(const TopoDS_Shape& s)
{
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}

skill::spur_gear_involute::Input goodInput()
{
    // m=2, z=20 → r_pitch=20, r_tip=22, r_root=17.5
    skill::spur_gear_involute::Input in;
    in.module_mm          = 2.0;
    in.teeth_count        = 20;
    in.face_width_mm      = 8.0;
    in.pressure_angle_deg = 20.0;
    in.blank_outer_dia_mm = 46.0;       // > 2·r_tip = 44
    return in;
}

}  // namespace

TEST(SkillSpurGearInvolute, ApplyRemovesRealGeometry)
{
    auto stock = skill::createCylindricalStock(46.0, 8.0);
    const double vStock = volumeOf(stock->shape());
    const int    fStock = stock->faceCount();

    auto in  = goodInput();
    auto out = skill::spur_gear_involute::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    const double vGear = volumeOf(out.workpiece->shape());
    EXPECT_LT(vGear, vStock) << "z teeth must remove real volume";
    EXPECT_GT(vStock - vGear, 1.0) << "non-trivial volume must be removed";

    // Face count grows — z gullets each leave ≥1 flank face.
    EXPECT_GT(out.workpiece->faceCount(), fStock + in.teeth_count / 2);
}

TEST(SkillSpurGearInvolute, ValidateRejectsBadInput)
{
    auto stock = skill::createCylindricalStock(46.0, 8.0);

    auto in = goodInput();
    in.teeth_count = 5;        // < 8
    auto rep = skill::spur_gear_involute::validate(*stock, in);
    EXPECT_FALSE(rep.passed);

    in = goodInput();
    in.pressure_angle_deg = 35.0;
    rep = skill::spur_gear_involute::validate(*stock, in);
    EXPECT_FALSE(rep.passed);

    in = goodInput();
    in.blank_outer_dia_mm = 20.0;   // < 2·r_tip = 44
    rep = skill::spur_gear_involute::validate(*stock, in);
    EXPECT_FALSE(rep.passed);
}

TEST(SkillSpurGearInvolute, SignatureCarriesGearMetadata)
{
    auto stock = skill::createCylindricalStock(46.0, 8.0);
    auto in    = goodInput();
    auto out   = skill::spur_gear_involute::apply(*stock, in);

    EXPECT_EQ(out.signature.skill_id, std::string("spur_gear_involute"));
    EXPECT_TRUE(out.signature.pattern.at("is_gear").get<bool>());
    EXPECT_EQ(out.signature.pattern.at("teeth_count").get<int>(), 20);
    EXPECT_NEAR(out.signature.pattern.at("module_mm").get<double>(), 2.0, 1e-9);
    EXPECT_NEAR(out.signature.pattern.at("pitch_dia_mm").get<double>(), 40.0, 1e-6);
    EXPECT_NEAR(out.signature.pattern.at("tip_dia_mm").get<double>(),   44.0, 1e-6);
    EXPECT_GT(out.signature.pattern.at("derived_volume_removed").get<double>(), 0.0);
}

TEST(SkillSpurGearInvolute, RecognizeReplaysSignature)
{
    auto stock = skill::createCylindricalStock(46.0, 8.0);
    auto in    = goodInput();
    auto out   = skill::spur_gear_involute::apply(*stock, in);

    auto cands = skill::spur_gear_involute::recognize(*out.workpiece);
    ASSERT_FALSE(cands.empty());

    bool found_replay = false;
    for (const auto& c : cands) {
        if (std::abs(c.confidence - 1.0) < 1e-9) { found_replay = true; break; }
    }
    EXPECT_TRUE(found_replay);
}
