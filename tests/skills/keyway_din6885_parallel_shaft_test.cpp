// @lat: [[process/test-strategy#skill round-trip]]

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/keyway_din6885_parallel_shaft.hpp"

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

TEST(SkillKeywayDin6885ParallelShaft, ApplyRemovesMaterial)
{
    // 20 mm dia × 40 mm tall shaft, axis Z.  20 mm falls in the 17–22 band.
    auto stock = skill::createCylindricalStock(20.0, 40.0);

    skill::keyway_din6885_parallel_shaft::Input in;
    in.face_id           = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.shaft_axis        = gp_Dir(0, 0, 1);
    in.shaft_center_x_mm = 0.0;
    in.shaft_center_y_mm = 0.0;
    in.shaft_dia_mm      = 20.0;
    in.key_position_z_mm = 20.0;
    in.key_length_mm     = 24.0;        // > 1.5 * 6 = 9.0
    in.angle_rad         = 0.0;

    const double v0 = volumeOf(stock->shape());
    auto out = skill::keyway_din6885_parallel_shaft::apply(*stock, in);
    const double v1 = volumeOf(out.workpiece->shape());

    EXPECT_LT(v1, v0);
    // Removed volume must be POSITIVE and bounded by the box envelope.
    const double removed = v0 - v1;
    EXPECT_GT(removed, 0.0);
    // Box envelope = length × width × 2·depth (cutter overshoots OD).
    const double envelope = 24.0 * 6.0 * 2.0 * 3.5;
    EXPECT_LT(removed, envelope + 1e-3);
}

TEST(SkillKeywayDin6885ParallelShaft, ValidateRejectsOutOfRangeDia)
{
    auto stock = skill::createCylindricalStock(20.0, 40.0);

    skill::keyway_din6885_parallel_shaft::Input in;
    in.face_id      = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.shaft_dia_mm = 200.0;           // out of 6..95 mm
    in.key_length_mm = 30.0;

    auto r = skill::keyway_din6885_parallel_shaft::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-DIN6885-RANGE") { found = true; break; }
    EXPECT_TRUE(found);
    EXPECT_THROW(skill::keyway_din6885_parallel_shaft::apply(*stock, in),
                 skill::SkillError);
}

TEST(SkillKeywayDin6885ParallelShaft, ValidateRejectsTooShortLength)
{
    auto stock = skill::createCylindricalStock(20.0, 40.0);

    skill::keyway_din6885_parallel_shaft::Input in;
    in.face_id      = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.shaft_dia_mm = 20.0;           // band b=6 mm
    in.key_length_mm = 5.0;           // < 1.5 * 6 = 9 mm

    auto r = skill::keyway_din6885_parallel_shaft::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-DIN6885-LEN") { found = true; break; }
    EXPECT_TRUE(found);
}

TEST(SkillKeywayDin6885ParallelShaft, SignatureRecordsDinSpec)
{
    auto stock = skill::createCylindricalStock(20.0, 40.0);

    skill::keyway_din6885_parallel_shaft::Input in;
    in.face_id           = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.shaft_dia_mm      = 20.0;     // 17–22 band: b=6, h=6, t1=3.5
    in.key_position_z_mm = 20.0;
    in.key_length_mm     = 20.0;

    auto out = skill::keyway_din6885_parallel_shaft::apply(*stock, in);

    EXPECT_EQ(out.signature.skill_id,
              std::string("keyway_din6885_parallel_shaft"));
    EXPECT_EQ(out.signature.pattern["kind"],
              "keyway_din6885_parallel_shaft");
    EXPECT_EQ(out.signature.pattern["is_compound"], true);
    EXPECT_EQ(out.signature.pattern["keyway_standard"],
              "DIN_6885_A_parallel_shaft");
    EXPECT_NEAR(out.signature.pattern["key_width_mm"].get<double>(),  6.0, 1e-9);
    EXPECT_NEAR(out.signature.pattern["key_depth_mm"].get<double>(),  3.5, 1e-9);
    EXPECT_GT  (out.signature.pattern["derived_volume_removed"].get<double>(), 0.0);
}

TEST(SkillKeywayDin6885ParallelShaft, RecognizeMetadataReplay)
{
    auto stock = skill::createCylindricalStock(20.0, 40.0);

    skill::keyway_din6885_parallel_shaft::Input in;
    in.face_id           = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.shaft_dia_mm      = 20.0;
    in.key_position_z_mm = 20.0;
    in.key_length_mm     = 18.0;

    auto out = skill::keyway_din6885_parallel_shaft::apply(*stock, in);
    auto cands = skill::keyway_din6885_parallel_shaft::recognize(
        *out.workpiece);

    ASSERT_FALSE(cands.empty());
    EXPECT_EQ(cands[0].skill_id,
              std::string("keyway_din6885_parallel_shaft"));
    EXPECT_NEAR(cands[0].recovered_params["shaft_dia_mm"].get<double>(),
                20.0, 1e-9);
}
