// @lat: [[process/test-strategy#skill round-trip]]

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/keyway_woodruff_arc.hpp"

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

TEST(SkillKeywayWoodruffArc, ApplyRemovesMaterial)
{
    // 40 mm dia × 60 mm tall shaft.  #808 Woodruff: b=6.35, d=25.4, dep=9.65.
    auto stock = skill::createCylindricalStock(40.0, 60.0);

    skill::keyway_woodruff_arc::Input in;
    in.face_id           = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.shaft_axis        = gp_Dir(0, 0, 1);
    in.shaft_dia_mm      = 40.0;
    in.key_position_z_mm = 30.0;
    in.angle_rad         = 0.0;
    in.woodruff_size_id  = "808";

    const double v0 = volumeOf(stock->shape());
    auto out = skill::keyway_woodruff_arc::apply(*stock, in);
    const double v1 = volumeOf(out.workpiece->shape());

    EXPECT_LT(v1, v0);
    EXPECT_GT(v0 - v1, 0.0);
}

TEST(SkillKeywayWoodruffArc, ValidateRejectsUnknownSize)
{
    auto stock = skill::createCylindricalStock(40.0, 60.0);

    skill::keyway_woodruff_arc::Input in;
    in.face_id          = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.shaft_dia_mm     = 40.0;
    in.woodruff_size_id = "9999";          // not in table

    auto r = skill::keyway_woodruff_arc::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-WOODRUFF-SIZE") { found = true; break; }
    EXPECT_TRUE(found);
    EXPECT_THROW(skill::keyway_woodruff_arc::apply(*stock, in),
                 skill::SkillError);
}

TEST(SkillKeywayWoodruffArc, SignatureRecordsSpec)
{
    auto stock = skill::createCylindricalStock(40.0, 60.0);

    skill::keyway_woodruff_arc::Input in;
    in.face_id          = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.shaft_dia_mm     = 40.0;
    in.key_position_z_mm = 30.0;
    in.woodruff_size_id = "808";

    auto out = skill::keyway_woodruff_arc::apply(*stock, in);

    EXPECT_EQ(out.signature.pattern["kind"], "keyway_woodruff_arc");
    EXPECT_EQ(out.signature.pattern["is_compound"], true);
    EXPECT_EQ(out.signature.pattern["keyway_standard"],
              "DIN_6888_woodruff_arc");
    EXPECT_NEAR(out.signature.pattern["key_width_mm"].get<double>(),
                6.35, 1e-9);
    EXPECT_NEAR(out.signature.pattern["key_diameter_mm"].get<double>(),
                25.4, 1e-9);
    EXPECT_EQ  (out.signature.pattern["end_geometry"], "arc");
    EXPECT_GT  (out.signature.pattern["derived_volume_removed"].get<double>(),
                0.0);
}

TEST(SkillKeywayWoodruffArc, RecognizeMetadataReplay)
{
    auto stock = skill::createCylindricalStock(40.0, 60.0);

    skill::keyway_woodruff_arc::Input in;
    in.face_id          = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.shaft_dia_mm     = 40.0;
    in.key_position_z_mm = 30.0;
    in.woodruff_size_id = "808";

    auto out = skill::keyway_woodruff_arc::apply(*stock, in);
    auto cands = skill::keyway_woodruff_arc::recognize(*out.workpiece);

    ASSERT_FALSE(cands.empty());
    EXPECT_EQ(cands[0].recovered_params["woodruff_size_id"].get<std::string>(),
              std::string("808"));
}
