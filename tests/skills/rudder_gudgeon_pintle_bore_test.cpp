// @lat: [[process/test-strategy#rudder_gudgeon_pintle_bore]]

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/rudder_gudgeon_pintle_bore.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

using namespace koocadcam;

namespace {
double volumeOf(const TopoDS_Shape& s) {
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}

skill::rudder_gudgeon_pintle_bore::Input goodInput()
{
    skill::rudder_gudgeon_pintle_bore::Input in;
    in.face_id         = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.center_xy       = gp_Pnt(20.0, 20.0, 0.0);
    in.pintle_dia_mm   = 12.0;
    in.bushing_od_mm   = 20.0;
    in.groove_width_mm = 3.0;
    in.bore_depth_mm   = 30.0;
    return in;
}
}  // namespace

TEST(SkillRudderGudgeonPintleBore, ApplyRemovesMaterial)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 40.0);
    const double v0 = volumeOf(stock->shape());
    auto in  = goodInput();
    auto out = skill::rudder_gudgeon_pintle_bore::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    const double v1 = volumeOf(out.workpiece->shape());

    EXPECT_GT(v0 - v1, 0.0);   // bore + grease groove
    EXPECT_GT(out.workpiece->faceCount(), stock->faceCount());
}

TEST(SkillRudderGudgeonPintleBore, ValidateRejectsBushingTooSmall)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 40.0);
    auto in = goodInput();
    in.bushing_od_mm = 10.0;   // <= pintle_dia_mm (12)

    auto r = skill::rudder_gudgeon_pintle_bore::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-MARINE-FIT") { found = true; break; }
    EXPECT_TRUE(found);

    EXPECT_THROW(skill::rudder_gudgeon_pintle_bore::apply(*stock, in),
                 skill::SkillError);
}

TEST(SkillRudderGudgeonPintleBore, ValidateRejectsGrooveDeeperThanBore)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 40.0);
    auto in = goodInput();
    in.groove_width_mm = 35.0;   // >= bore_depth_mm (30)

    auto r = skill::rudder_gudgeon_pintle_bore::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-MARINE-GROOVE") { found = true; break; }
    EXPECT_TRUE(found);
}

TEST(SkillRudderGudgeonPintleBore, SignatureCompoundBore)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 40.0);
    auto in    = goodInput();
    auto out   = skill::rudder_gudgeon_pintle_bore::apply(*stock, in);

    EXPECT_EQ(out.signature.skill_id,
              std::string("rudder_gudgeon_pintle_bore"));
    EXPECT_TRUE(out.signature.pattern.value("is_compound", false));
    EXPECT_EQ(out.signature.pattern.value("marine_feature_type", std::string()),
              std::string("rudder_gudgeon_pintle_bore"));
    EXPECT_EQ(out.signature.pattern.value("subfeature_count", 0), 2);
}

TEST(SkillRudderGudgeonPintleBore, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 40.0);
    auto in    = goodInput();
    auto out   = skill::rudder_gudgeon_pintle_bore::apply(*stock, in);
    auto cands = skill::rudder_gudgeon_pintle_bore::recognize(*out.workpiece);
    ASSERT_FALSE(cands.empty());
    EXPECT_DOUBLE_EQ(cands[0].confidence, 1.0);
}
