// @lat: [[process/test-strategy#cutless_bearing_housing_seat]]

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/cutless_bearing_housing_seat.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

using namespace koocadcam;

namespace {
double volumeOf(const TopoDS_Shape& s) {
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}

skill::cutless_bearing_housing_seat::Input goodInput()
{
    skill::cutless_bearing_housing_seat::Input in;
    in.face_id              = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.axis_origin          = gp_Pnt(0.0, 0.0, 0.0);
    in.bearing_od_mm        = 30.0;
    in.housing_length_mm    = 50.0;
    in.set_screw_thread_key = "M6";
    in.groove_count         = 3;
    return in;
}
}  // namespace

TEST(SkillCutlessBearingHousingSeat, ApplyRemovesMaterial)
{
    auto stock = skill::createCylindricalStock(60.0, 60.0);
    const double v0 = volumeOf(stock->shape());
    auto in  = goodInput();
    auto out = skill::cutless_bearing_housing_seat::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    const double v1 = volumeOf(out.workpiece->shape());

    EXPECT_GT(v0 - v1, 0.0);   // bore + 2 set screws + grooves
    EXPECT_GT(out.workpiece->faceCount(), stock->faceCount());
}

TEST(SkillCutlessBearingHousingSeat, ValidateRejectsUnknownThread)
{
    auto stock = skill::createCylindricalStock(60.0, 60.0);
    auto in = goodInput();
    in.set_screw_thread_key = "M99";

    auto r = skill::cutless_bearing_housing_seat::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-M-THREAD") { found = true; break; }
    EXPECT_TRUE(found);

    EXPECT_THROW(skill::cutless_bearing_housing_seat::apply(*stock, in),
                 skill::SkillError);
}

TEST(SkillCutlessBearingHousingSeat, ValidateRejectsGrooveCountOutOfRange)
{
    auto stock = skill::createCylindricalStock(60.0, 60.0);
    auto in = goodInput();
    in.groove_count = 0;   // < 1

    auto r = skill::cutless_bearing_housing_seat::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-MARINE-GROOVE") { found = true; break; }
    EXPECT_TRUE(found);
}

TEST(SkillCutlessBearingHousingSeat, SignatureCompoundHousing)
{
    auto stock = skill::createCylindricalStock(60.0, 60.0);
    auto in    = goodInput();
    auto out   = skill::cutless_bearing_housing_seat::apply(*stock, in);

    EXPECT_EQ(out.signature.skill_id,
              std::string("cutless_bearing_housing_seat"));
    EXPECT_TRUE(out.signature.pattern.value("is_compound", false));
    EXPECT_EQ(out.signature.pattern.value("marine_feature_type", std::string()),
              std::string("cutless_bearing_housing_seat"));
    EXPECT_EQ(out.signature.pattern.value("subfeature_count", 0),
              1 + 2 + in.groove_count);
}

TEST(SkillCutlessBearingHousingSeat, RecognizeMetadataReplay)
{
    auto stock = skill::createCylindricalStock(60.0, 60.0);
    auto in    = goodInput();
    auto out   = skill::cutless_bearing_housing_seat::apply(*stock, in);
    auto cands = skill::cutless_bearing_housing_seat::recognize(*out.workpiece);
    ASSERT_FALSE(cands.empty());
    EXPECT_DOUBLE_EQ(cands[0].confidence, 1.0);
}
