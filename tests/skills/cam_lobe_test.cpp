// @lat: [[process/test-strategy#skill round-trip]]

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/cam_lobe.hpp"

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

TEST(SkillCamLobe, ApplyChangesGeometry)
{
    // Start with a 30 mm dia × 8 mm tall puck; reduce to baseR = 10 mm.
    auto stock = skill::createCylindricalStock(30.0, 8.0);
    ASSERT_FALSE(stock->shape().IsNull());

    skill::cam_lobe::Input in;
    in.entry_face    = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.center_x_mm   = 0.0;
    in.center_y_mm   = 0.0;
    in.angle_deg     = 0.0;
    in.lobe_lift_mm  = 2.0;
    in.lobe_angle_deg = 30.0;
    in.base_radius_mm = 10.0;
    in.thickness_mm  = 8.0;

    const double v0 = volumeOf(stock->shape());
    auto out = skill::cam_lobe::apply(*stock, in);
    const double v1 = volumeOf(out.workpiece->shape());

    EXPECT_LT(v1, v0) << "cam_lobe must reduce the stock to ~base_radius";
}

TEST(SkillCamLobe, ValidateRejectsBadInput)
{
    auto stock = skill::createCylindricalStock(30.0, 8.0);

    skill::cam_lobe::Input in;
    in.entry_face    = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.lobe_lift_mm  = 2.0;
    in.lobe_angle_deg = 400.0;     // > 359.9 — bad
    in.base_radius_mm = 10.0;
    in.thickness_mm  = 8.0;

    auto r = skill::cam_lobe::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-CAM-ANG") { found = true; break; }
    EXPECT_TRUE(found);
    EXPECT_THROW(skill::cam_lobe::apply(*stock, in), skill::SkillError);
}

TEST(SkillCamLobe, SignatureRecordsKindAndParams)
{
    auto stock = skill::createCylindricalStock(30.0, 8.0);

    skill::cam_lobe::Input in;
    in.entry_face    = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.angle_deg     = 90.0;
    in.lobe_lift_mm  = 3.0;
    in.lobe_angle_deg = 45.0;
    in.base_radius_mm = 10.0;
    in.thickness_mm  = 8.0;

    auto out = skill::cam_lobe::apply(*stock, in);

    EXPECT_EQ(out.signature.skill_id, std::string("cam_lobe"));
    EXPECT_EQ(out.signature.pattern["kind"], "cam_lobe");
    EXPECT_NEAR(out.signature.params["lobe_lift_mm"].get<double>(), 3.0, 1e-9);
    EXPECT_NEAR(out.signature.params["lobe_angle_deg"].get<double>(), 45.0, 1e-9);
    EXPECT_NEAR(out.signature.params["base_radius_mm"].get<double>(), 10.0, 1e-9);
    EXPECT_NEAR(out.signature.params["angle_deg"].get<double>(), 90.0, 1e-9);
}

TEST(SkillCamLobe, RecognizeMetadataReplay)
{
    auto stock = skill::createCylindricalStock(30.0, 8.0);

    skill::cam_lobe::Input in;
    in.entry_face    = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.lobe_lift_mm  = 2.5;
    in.lobe_angle_deg = 60.0;
    in.base_radius_mm = 11.0;
    in.thickness_mm  = 8.0;

    auto out = skill::cam_lobe::apply(*stock, in);
    auto cands = skill::cam_lobe::recognize(*out.workpiece);

    ASSERT_EQ(cands.size(), 1u);
    EXPECT_EQ(cands[0].skill_id, std::string("cam_lobe"));
    EXPECT_NEAR(cands[0].recovered_params["lobe_lift_mm"].get<double>(),
                2.5, 1e-9);
    EXPECT_NEAR(cands[0].recovered_params["lobe_angle_deg"].get<double>(),
                60.0, 1e-9);
}

// ─── 5. Specific: after apply, workpiece bbox radius ≤ base_radius (+slack) ─
TEST(SkillCamLobe, ReducesToBaseRadiusEnvelope)
{
    auto stock = skill::createCylindricalStock(30.0, 8.0);

    skill::cam_lobe::Input in;
    in.entry_face    = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.lobe_lift_mm  = 2.0;
    in.lobe_angle_deg = 30.0;
    in.base_radius_mm = 10.0;
    in.thickness_mm  = 8.0;

    auto out = skill::cam_lobe::apply(*stock, in);
    double xMin, yMin, zMin, xMax, yMax, zMax;
    out.workpiece->boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    // For cylindrical_reduction the result is a cylinder of radius=base_radius_mm.
    const double bboxR = std::max(xMax - xMin, yMax - yMin) / 2.0;
    EXPECT_LE(bboxR, in.base_radius_mm + 0.5)
        << "post-cam_lobe bbox radius should be ~ base_radius";
}
