// @lat: [[process/test-strategy#skill round-trip]]

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/keyway_external.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

#include <cmath>

using namespace koocadcam;

namespace {
double volumeOf(const TopoDS_Shape& s) {
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}
}

TEST(SkillKeywayExternal, ApplyChangesGeometry)
{
    // Shaft: 20 mm dia × 40 mm tall along Z; center at (0,0).
    auto stock = skill::createCylindricalStock(20.0, 40.0);
    ASSERT_FALSE(stock->shape().IsNull());

    skill::keyway_external::Input in;
    in.entry_face        = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.shaft_axis        = gp_Dir(0, 0, 1);
    in.shaft_center_x_mm = 0.0;
    in.shaft_center_y_mm = 0.0;
    in.shaft_radius_mm   = 10.0;
    in.angle_rad         = 0.0;
    in.axial_center_mm   = 20.0;
    in.length_mm         = 20.0;
    in.width_mm          = 4.0;
    in.depth_mm          = 3.0;

    const double v0 = volumeOf(stock->shape());
    auto out = skill::keyway_external::apply(*stock, in);
    const double v1 = volumeOf(out.workpiece->shape());

    EXPECT_LT(v1, v0);
    // Material removed lies BETWEEN the slot's planar walls and the curved
    // shaft OD, so the box-cut overestimates.  Sanity check: removed
    // material < box volume (L × W × 2·D).
    EXPECT_LT(v0 - v1, in.length_mm * in.width_mm * 2.0 * in.depth_mm + 1e-3);
}

TEST(SkillKeywayExternal, ValidateRejectsBadInput)
{
    auto stock = skill::createCylindricalStock(20.0, 40.0);

    skill::keyway_external::Input in;
    in.entry_face        = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.shaft_radius_mm   = 10.0;
    in.length_mm         = 20.0;
    in.width_mm          = 0.5;     // < 0.8 mm → DFM-002
    in.depth_mm          = 3.0;

    auto r = skill::keyway_external::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-002") { found = true; break; }
    EXPECT_TRUE(found);
    EXPECT_THROW(skill::keyway_external::apply(*stock, in), skill::SkillError);
}

TEST(SkillKeywayExternal, SignatureRecordsKindAndParams)
{
    auto stock = skill::createCylindricalStock(20.0, 40.0);

    skill::keyway_external::Input in;
    in.entry_face        = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.shaft_radius_mm   = 10.0;
    in.angle_rad         = M_PI / 2.0;
    in.axial_center_mm   = 20.0;
    in.length_mm         = 15.0;
    in.width_mm          = 5.0;
    in.depth_mm          = 2.5;

    auto out = skill::keyway_external::apply(*stock, in);

    EXPECT_EQ(out.signature.skill_id, std::string("keyway_external"));
    EXPECT_EQ(out.signature.pattern["kind"], "keyway_external");
    EXPECT_NEAR(out.signature.params["width_mm"].get<double>(), 5.0, 1e-9);
    EXPECT_NEAR(out.signature.params["depth_mm"].get<double>(), 2.5, 1e-9);
    EXPECT_NEAR(out.signature.params["length_mm"].get<double>(), 15.0, 1e-9);
    EXPECT_NEAR(out.signature.params["shaft_radius_mm"].get<double>(), 10.0, 1e-9);
}

TEST(SkillKeywayExternal, RecognizeMetadataReplay)
{
    auto stock = skill::createCylindricalStock(20.0, 40.0);

    skill::keyway_external::Input in;
    in.entry_face        = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.shaft_radius_mm   = 10.0;
    in.axial_center_mm   = 20.0;
    in.length_mm         = 12.0;
    in.width_mm          = 4.0;
    in.depth_mm          = 2.0;

    auto out = skill::keyway_external::apply(*stock, in);
    auto cands = skill::keyway_external::recognize(*out.workpiece);

    ASSERT_EQ(cands.size(), 1u);
    EXPECT_EQ(cands[0].skill_id, std::string("keyway_external"));
    EXPECT_NEAR(cands[0].recovered_params["width_mm"].get<double>(),
                4.0, 1e-9);
    EXPECT_NEAR(cands[0].recovered_params["length_mm"].get<double>(),
                12.0, 1e-9);
}

// ─── 5. Specific: depth >= shaft_radius is rejected (would punch through) ──
TEST(SkillKeywayExternal, DepthBeyondRadiusIsRejected)
{
    auto stock = skill::createCylindricalStock(20.0, 40.0);

    skill::keyway_external::Input in;
    in.entry_face        = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.shaft_radius_mm   = 10.0;
    in.length_mm         = 12.0;
    in.width_mm          = 4.0;
    in.depth_mm          = 12.0;   // > shaft_radius

    auto r = skill::keyway_external::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-KEY-DEPTH") { found = true; break; }
    EXPECT_TRUE(found);
}
