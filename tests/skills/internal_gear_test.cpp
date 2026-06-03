// @lat: [[process/test-strategy#internal_gear]]
//
// internal_gear — REAL multi-cut internal (ring) gear with inward-pointing teeth.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/internal_gear.hpp"

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

skill::internal_gear::Input goodInput()
{
    // m=2, z=20 → r_pitch=20, r_root_outer = 22.5, need OD > 2·(22.5 + 4) = 53
    skill::internal_gear::Input in;
    in.module_mm          = 2.0;
    in.teeth_count        = 20;
    in.face_width_mm      = 10.0;
    in.pressure_angle_deg = 20.0;
    in.ring_outer_dia_mm  = 60.0;
    return in;
}

}  // namespace

TEST(SkillInternalGear, ApplyRemovesRealGeometry)
{
    auto stock = skill::createCylindricalStock(60.0, 10.0);
    const double vStock = volumeOf(stock->shape());
    const int    fStock = stock->faceCount();

    auto in  = goodInput();
    auto out = skill::internal_gear::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    const double vGear = volumeOf(out.workpiece->shape());
    EXPECT_LT(vGear, vStock);
    EXPECT_GT(vStock - vGear, 1.0);

    EXPECT_GT(out.workpiece->faceCount(), fStock + in.teeth_count / 2);
}

TEST(SkillInternalGear, ValidateRejectsThinWall)
{
    auto stock = skill::createCylindricalStock(60.0, 10.0);

    auto in = goodInput();
    in.ring_outer_dia_mm = 40.0;          // < required ~53
    auto rep = skill::internal_gear::validate(*stock, in);
    EXPECT_FALSE(rep.passed);
}

TEST(SkillInternalGear, SignatureMarksInternal)
{
    auto stock = skill::createCylindricalStock(60.0, 10.0);
    auto in    = goodInput();
    auto out   = skill::internal_gear::apply(*stock, in);

    EXPECT_EQ(out.signature.skill_id, std::string("internal_gear"));
    EXPECT_TRUE(out.signature.pattern.at("is_gear").get<bool>());
    EXPECT_TRUE(out.signature.pattern.at("is_internal").get<bool>());
    EXPECT_EQ(out.signature.pattern.at("teeth_count").get<int>(), 20);
    EXPECT_NEAR(out.signature.pattern.at("pitch_dia_mm").get<double>(), 40.0, 1e-6);
}

TEST(SkillInternalGear, RecognizeReplays)
{
    auto stock = skill::createCylindricalStock(60.0, 10.0);
    auto in    = goodInput();
    auto out   = skill::internal_gear::apply(*stock, in);

    auto cands = skill::internal_gear::recognize(*out.workpiece);
    ASSERT_FALSE(cands.empty());

    bool found = false;
    for (const auto& c : cands) {
        if (std::abs(c.confidence - 1.0) < 1e-9) { found = true; break; }
    }
    EXPECT_TRUE(found);
}
