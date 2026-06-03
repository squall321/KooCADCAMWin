// @lat: [[process/test-strategy#bevel_gear]]
//
// bevel_gear — REAL multi-cut bevel involute gear (tapered slab stack).

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/bevel_gear.hpp"

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

skill::bevel_gear::Input goodInput()
{
    skill::bevel_gear::Input in;
    in.module_mm          = 2.0;
    in.teeth_count        = 20;
    in.face_width_mm      = 8.0;
    in.cone_angle_deg     = 30.0;
    in.pressure_angle_deg = 20.0;
    return in;
}

}  // namespace

TEST(SkillBevelGear, ApplyRemovesRealGeometry)
{
    // Use a 50 mm cylindrical stock that's large enough for the outer cone diameter.
    auto stock = skill::createCylindricalStock(50.0, 8.0);
    const double vStock = volumeOf(stock->shape());
    const int    fStock = stock->faceCount();

    auto in  = goodInput();
    auto out = skill::bevel_gear::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    const double vGear = volumeOf(out.workpiece->shape());
    EXPECT_LT(vGear, vStock);
    EXPECT_GT(vStock - vGear, 1.0);

    EXPECT_GT(out.workpiece->faceCount(), fStock + in.teeth_count / 2);
}

TEST(SkillBevelGear, ValidateRejectsBadCone)
{
    auto stock = skill::createCylindricalStock(50.0, 8.0);

    auto in = goodInput();
    in.cone_angle_deg = 5.0;          // < 10
    auto rep = skill::bevel_gear::validate(*stock, in);
    EXPECT_FALSE(rep.passed);

    in = goodInput();
    in.cone_angle_deg = 90.0;         // > 80
    rep = skill::bevel_gear::validate(*stock, in);
    EXPECT_FALSE(rep.passed);
}

TEST(SkillBevelGear, SignatureHasConeMetadata)
{
    auto stock = skill::createCylindricalStock(50.0, 8.0);
    auto in    = goodInput();
    auto out   = skill::bevel_gear::apply(*stock, in);

    EXPECT_EQ(out.signature.skill_id, std::string("bevel_gear"));
    EXPECT_TRUE(out.signature.pattern.at("is_gear").get<bool>());
    EXPECT_EQ(out.signature.pattern.at("teeth_count").get<int>(), 20);
    EXPECT_NEAR(out.signature.pattern.at("cone_angle_deg").get<double>(), 30.0, 1e-9);
}

TEST(SkillBevelGear, RecognizeReplays)
{
    auto stock = skill::createCylindricalStock(50.0, 8.0);
    auto in    = goodInput();
    auto out   = skill::bevel_gear::apply(*stock, in);

    auto cands = skill::bevel_gear::recognize(*out.workpiece);
    ASSERT_FALSE(cands.empty());

    bool found = false;
    for (const auto& c : cands) {
        if (std::abs(c.confidence - 1.0) < 1e-9) { found = true; break; }
    }
    EXPECT_TRUE(found);
}
