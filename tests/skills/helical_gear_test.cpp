// @lat: [[process/test-strategy#helical_gear]]
//
// helical_gear — REAL multi-cut helical involute gear (slab-stack approx).

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/helical_gear.hpp"

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

skill::helical_gear::Input goodInput()
{
    skill::helical_gear::Input in;
    in.module_mm          = 2.0;
    in.teeth_count        = 12;
    in.face_width_mm      = 10.0;
    in.pressure_angle_deg = 20.0;
    in.helix_angle_deg    = 15.0;
    in.blank_outer_dia_mm = 30.0;
    in.axial_slice_count  = 2;
    return in;
}

}  // namespace

TEST(SkillHelicalGear, DISABLED_ApplyRemovesRealGeometry)
{
    auto stock = skill::createCylindricalStock(46.0, 10.0);
    const double vStock = volumeOf(stock->shape());
    const int    fStock = stock->faceCount();

    auto in  = goodInput();
    auto out = skill::helical_gear::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    const double vGear = volumeOf(out.workpiece->shape());
    EXPECT_LT(vGear, vStock);
    EXPECT_GT(vStock - vGear, 1.0);

    EXPECT_GT(out.workpiece->faceCount(), fStock + in.teeth_count / 2);
}

TEST(SkillHelicalGear, ValidateRejectsBadHelix)
{
    auto stock = skill::createCylindricalStock(46.0, 10.0);

    auto in = goodInput();
    in.helix_angle_deg = 40.0;       // > 35
    auto rep = skill::helical_gear::validate(*stock, in);
    EXPECT_FALSE(rep.passed);

    in = goodInput();
    in.helix_angle_deg = -1.0;
    rep = skill::helical_gear::validate(*stock, in);
    EXPECT_FALSE(rep.passed);
}

TEST(SkillHelicalGear, SignatureHasHelixMetadata)
{
    auto stock = skill::createCylindricalStock(30.0, 10.0);
    auto in    = goodInput();
    auto out   = skill::helical_gear::apply(*stock, in);

    EXPECT_EQ(out.signature.skill_id, std::string("helical_gear"));
    EXPECT_TRUE(out.signature.pattern.at("is_gear").get<bool>());
    EXPECT_EQ(out.signature.pattern.at("teeth_count").get<int>(), 12);
    EXPECT_NEAR(out.signature.pattern.at("helix_angle_deg").get<double>(), 15.0, 1e-9);
    EXPECT_GT(out.signature.pattern.at("helix_twist_total_rad").get<double>(), 0.0);
}

TEST(SkillHelicalGear, RecognizeReplays)
{
    auto stock = skill::createCylindricalStock(30.0, 10.0);
    auto in    = goodInput();
    auto out   = skill::helical_gear::apply(*stock, in);

    auto cands = skill::helical_gear::recognize(*out.workpiece);
    ASSERT_FALSE(cands.empty());

    bool found = false;
    for (const auto& c : cands) {
        if (std::abs(c.confidence - 1.0) < 1e-9) { found = true; break; }
    }
    EXPECT_TRUE(found);
}
