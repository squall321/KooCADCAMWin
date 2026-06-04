// @lat: [[process/test-strategy#skill round-trip]]

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/keyway_din6886_taper_shaft.hpp"

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

TEST(SkillKeywayDin6886TaperShaft, ApplyRemovesMaterial)
{
    auto stock = skill::createCylindricalStock(20.0, 60.0);

    skill::keyway_din6886_taper_shaft::Input in;
    in.face_id           = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.shaft_axis        = gp_Dir(0, 0, 1);
    in.shaft_dia_mm      = 20.0;            // 17–22 band
    in.key_position_z_mm = 30.0;
    in.key_length_mm     = 24.0;            // > 1.5 * 6 = 9
    in.angle_rad         = 0.0;

    const double v0 = volumeOf(stock->shape());
    auto out = skill::keyway_din6886_taper_shaft::apply(*stock, in);
    const double v1 = volumeOf(out.workpiece->shape());

    EXPECT_LT(v1, v0);
    EXPECT_GT(v0 - v1, 0.0);
}

TEST(SkillKeywayDin6886TaperShaft, ValidateRejectsOutOfRangeDia)
{
    auto stock = skill::createCylindricalStock(20.0, 60.0);

    skill::keyway_din6886_taper_shaft::Input in;
    in.face_id        = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.shaft_dia_mm   = 300.0;
    in.key_length_mm  = 30.0;

    auto r = skill::keyway_din6886_taper_shaft::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-DIN6886-RANGE") { found = true; break; }
    EXPECT_TRUE(found);
}

TEST(SkillKeywayDin6886TaperShaft, ValidateRejectsTooShortLength)
{
    auto stock = skill::createCylindricalStock(20.0, 60.0);

    skill::keyway_din6886_taper_shaft::Input in;
    in.face_id        = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.shaft_dia_mm   = 20.0;
    in.key_length_mm  = 5.0;

    auto r = skill::keyway_din6886_taper_shaft::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-DIN6886-LEN") { found = true; break; }
    EXPECT_TRUE(found);
}

TEST(SkillKeywayDin6886TaperShaft, SignatureRecordsTaperSpec)
{
    auto stock = skill::createCylindricalStock(20.0, 60.0);

    skill::keyway_din6886_taper_shaft::Input in;
    in.face_id           = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.shaft_dia_mm      = 20.0;       // band: b=6, t1=3.5, taper=1.0
    in.key_position_z_mm = 30.0;
    in.key_length_mm     = 20.0;

    auto out = skill::keyway_din6886_taper_shaft::apply(*stock, in);

    EXPECT_EQ(out.signature.pattern["kind"], "keyway_din6886_taper_shaft");
    EXPECT_EQ(out.signature.pattern["is_compound"], true);
    EXPECT_EQ(out.signature.pattern["keyway_standard"],
              "DIN_6886_taper_shaft");
    EXPECT_NEAR(out.signature.pattern["key_width_mm"].get<double>(), 6.0, 1e-9);
    EXPECT_NEAR(out.signature.pattern["key_depth_mm"].get<double>(), 3.5, 1e-9);
    EXPECT_NEAR(out.signature.pattern["taper_per_100"].get<double>(), 1.0, 1e-9);
    // Thin-end depth = 3.5 - 20/100 = 3.3 mm.
    EXPECT_NEAR(out.signature.pattern["key_depth_thin_end_mm"].get<double>(),
                3.3, 1e-6);
}

TEST(SkillKeywayDin6886TaperShaft, RecognizeMetadataReplay)
{
    auto stock = skill::createCylindricalStock(20.0, 60.0);

    skill::keyway_din6886_taper_shaft::Input in;
    in.face_id           = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.shaft_dia_mm      = 20.0;
    in.key_position_z_mm = 30.0;
    in.key_length_mm     = 18.0;

    auto out = skill::keyway_din6886_taper_shaft::apply(*stock, in);
    auto cands = skill::keyway_din6886_taper_shaft::recognize(
        *out.workpiece);
    ASSERT_FALSE(cands.empty());
    EXPECT_NEAR(cands[0].recovered_params["shaft_dia_mm"].get<double>(),
                20.0, 1e-9);
}
