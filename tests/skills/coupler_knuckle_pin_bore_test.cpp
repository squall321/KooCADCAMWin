// @lat: [[process/test-strategy#coupler_knuckle_pin_bore]]

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/coupler_knuckle_pin_bore.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

using namespace koocadcam;

namespace {
double volumeOf(const TopoDS_Shape& s) {
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}

skill::coupler_knuckle_pin_bore::Input goodInput()
{
    skill::coupler_knuckle_pin_bore::Input in;
    in.face_id                = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.center_xy              = gp_Pnt(60.0, 60.0, 0.0);
    in.pin_dia_mm             = 50.0;
    in.washer_recess_dia_mm   = 70.0;
    in.washer_recess_depth_mm = 6.0;
    in.lock_pin_dia_mm        = 12.0;
    return in;
}
}  // namespace

TEST(SkillCouplerKnucklePinBore, ApplyRemovesMaterial)
{
    auto stock = skill::createCuboidStock(120.0, 120.0, 60.0);
    const double v0 = volumeOf(stock->shape());
    auto in  = goodInput();
    auto out = skill::coupler_knuckle_pin_bore::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    const double v1 = volumeOf(out.workpiece->shape());

    EXPECT_GT(v0 - v1, 0.0);   // material removed (bore + recess + cross hole)
    EXPECT_GT(out.workpiece->faceCount(), stock->faceCount());
}

TEST(SkillCouplerKnucklePinBore, ValidateRejectsSmallRecess)
{
    auto stock = skill::createCuboidStock(120.0, 120.0, 60.0);
    auto in = goodInput();
    in.washer_recess_dia_mm = 40.0;   // <= pin_dia_mm

    auto r = skill::coupler_knuckle_pin_bore::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-RECESS") { found = true; break; }
    EXPECT_TRUE(found);

    EXPECT_THROW(skill::coupler_knuckle_pin_bore::apply(*stock, in),
                 skill::SkillError);
}

TEST(SkillCouplerKnucklePinBore, ValidateRejectsLargeLockPin)
{
    auto stock = skill::createCuboidStock(120.0, 120.0, 60.0);
    auto in = goodInput();
    in.lock_pin_dia_mm = 80.0;   // >= pin_dia_mm

    auto r = skill::coupler_knuckle_pin_bore::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-LOCKPIN") { found = true; break; }
    EXPECT_TRUE(found);
}

TEST(SkillCouplerKnucklePinBore, SignatureCompoundKnuckle)
{
    auto stock = skill::createCuboidStock(120.0, 120.0, 60.0);
    auto in    = goodInput();
    auto out   = skill::coupler_knuckle_pin_bore::apply(*stock, in);

    EXPECT_EQ(out.signature.skill_id, std::string("coupler_knuckle_pin_bore"));
    EXPECT_TRUE(out.signature.pattern.value("is_compound", false));
    EXPECT_EQ(out.signature.pattern.value("railway_feature_type", std::string()),
              std::string("coupler_knuckle_pin_bore"));
    EXPECT_EQ(out.signature.pattern.value("subfeature_count", 0), 3);
}

TEST(SkillCouplerKnucklePinBore, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(120.0, 120.0, 60.0);
    auto in    = goodInput();
    auto out   = skill::coupler_knuckle_pin_bore::apply(*stock, in);
    auto cands = skill::coupler_knuckle_pin_bore::recognize(*out.workpiece);
    ASSERT_FALSE(cands.empty());
    EXPECT_DOUBLE_EQ(cands[0].confidence, 1.0);
}
