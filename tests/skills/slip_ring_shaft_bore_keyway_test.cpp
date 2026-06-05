// @lat: [[process/test-strategy#slip_ring_shaft_bore_keyway]]

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/slip_ring_shaft_bore_keyway.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

using namespace koocadcam;

namespace {
double volumeOf(const TopoDS_Shape& s) {
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}

skill::slip_ring_shaft_bore_keyway::Input goodInput()
{
    skill::slip_ring_shaft_bore_keyway::Input in;
    in.face_id                = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.axis_origin            = gp_Pnt(0.0, 0.0, 0.0);
    in.shaft_bore_dia_mm      = 60.0;   // → DIN 6885 band 58..65
    in.key_position_z_mm      = 5.0;
    in.key_length_mm          = 40.0;
    in.cable_channel_width_mm = 12.0;
    in.cable_channel_depth_mm = 10.0;
    return in;
}
}  // namespace

TEST(SkillSlipRingShaftBoreKeyway, ApplyRemovesMaterial)
{
    auto stock = skill::createCylindricalStock(120.0, 80.0);
    const double v0 = volumeOf(stock->shape());
    auto in  = goodInput();
    auto out = skill::slip_ring_shaft_bore_keyway::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    const double v1 = volumeOf(out.workpiece->shape());

    EXPECT_GT(v0 - v1, 0.0);   // bore + keyway + cable channel removed
    EXPECT_GT(out.workpiece->faceCount(), stock->faceCount());
}

TEST(SkillSlipRingShaftBoreKeyway, ValidateRejectsNoKeywayBand)
{
    auto stock = skill::createCylindricalStock(300.0, 80.0);
    auto in = goodInput();
    in.shaft_bore_dia_mm = 200.0;   // beyond DIN 6885 table max (95 mm)
    in.cable_channel_width_mm = 12.0;

    auto r = skill::slip_ring_shaft_bore_keyway::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-KEYWAY") { found = true; break; }
    EXPECT_TRUE(found);

    EXPECT_THROW(skill::slip_ring_shaft_bore_keyway::apply(*stock, in),
                 skill::SkillError);
}

TEST(SkillSlipRingShaftBoreKeyway, ValidateRejectsKeyTooLong)
{
    auto stock = skill::createCylindricalStock(120.0, 30.0);
    auto in = goodInput();
    in.key_position_z_mm = 5.0;
    in.key_length_mm     = 40.0;   // 45 > 30 bore depth

    auto r = skill::slip_ring_shaft_bore_keyway::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-KEY-POS") { found = true; break; }
    EXPECT_TRUE(found);
}

TEST(SkillSlipRingShaftBoreKeyway, SignatureCompound)
{
    auto stock = skill::createCylindricalStock(120.0, 80.0);
    auto in    = goodInput();
    auto out   = skill::slip_ring_shaft_bore_keyway::apply(*stock, in);

    EXPECT_EQ(out.signature.skill_id,
              std::string("slip_ring_shaft_bore_keyway"));
    EXPECT_TRUE(out.signature.pattern.value("is_compound", false));
    EXPECT_EQ(out.signature.pattern.value("wind_feature_type", std::string()),
              std::string("slip_ring_shaft_bore_keyway"));
    EXPECT_EQ(out.signature.pattern.value("subfeature_count", 0), 3);
}

TEST(SkillSlipRingShaftBoreKeyway, RecognizeMetadataReplay)
{
    auto stock = skill::createCylindricalStock(120.0, 80.0);
    auto in    = goodInput();
    auto out   = skill::slip_ring_shaft_bore_keyway::apply(*stock, in);
    auto cands = skill::slip_ring_shaft_bore_keyway::recognize(*out.workpiece);
    ASSERT_FALSE(cands.empty());
    EXPECT_DOUBLE_EQ(cands[0].confidence, 1.0);
}
