// @lat: [[process/test-strategy#axle_box_bearing_housing]]

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/axle_box_bearing_housing.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

using namespace koocadcam;

namespace {
double volumeOf(const TopoDS_Shape& s) {
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}

skill::axle_box_bearing_housing::Input goodInput()
{
    skill::axle_box_bearing_housing::Input in;
    in.face_id           = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.axis_origin       = gp_Pnt(0.0, 0.0, 0.0);
    in.bearing_od_mm     = 130.0;
    in.housing_depth_mm  = 40.0;
    in.ring_size_key     = "100mm";
    in.grease_thread_key = "M10";
    return in;
}
}  // namespace

TEST(SkillAxleBoxBearingHousing, ApplyRemovesMaterial)
{
    auto stock = skill::createCylindricalStock(200.0, 55.0);
    const double v0 = volumeOf(stock->shape());
    auto in  = goodInput();
    auto out = skill::axle_box_bearing_housing::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    const double v1 = volumeOf(out.workpiece->shape());

    EXPECT_GT(v0 - v1, 0.0);   // material removed (bore + groove + grease port)
    EXPECT_GT(out.workpiece->faceCount(), stock->faceCount());
}

TEST(SkillAxleBoxBearingHousing, ValidateRejectsUnknownRing)
{
    auto stock = skill::createCylindricalStock(200.0, 55.0);
    auto in = goodInput();
    in.ring_size_key = "999mm";

    auto r = skill::axle_box_bearing_housing::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-RING") { found = true; break; }
    EXPECT_TRUE(found);

    EXPECT_THROW(skill::axle_box_bearing_housing::apply(*stock, in),
                 skill::SkillError);
}

TEST(SkillAxleBoxBearingHousing, ValidateRejectsUnknownThread)
{
    auto stock = skill::createCylindricalStock(200.0, 55.0);
    auto in = goodInput();
    in.grease_thread_key = "M99";

    auto r = skill::axle_box_bearing_housing::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-THREAD") { found = true; break; }
    EXPECT_TRUE(found);
}

TEST(SkillAxleBoxBearingHousing, SignatureCompoundAxleBox)
{
    auto stock = skill::createCylindricalStock(200.0, 55.0);
    auto in    = goodInput();
    auto out   = skill::axle_box_bearing_housing::apply(*stock, in);

    EXPECT_EQ(out.signature.skill_id, std::string("axle_box_bearing_housing"));
    EXPECT_TRUE(out.signature.pattern.value("is_compound", false));
    EXPECT_EQ(out.signature.pattern.value("railway_feature_type", std::string()),
              std::string("axle_box_bearing_housing"));
    EXPECT_EQ(out.signature.pattern.value("subfeature_count", 0), 3);
}

TEST(SkillAxleBoxBearingHousing, RecognizeMetadataReplay)
{
    auto stock = skill::createCylindricalStock(200.0, 55.0);
    auto in    = goodInput();
    auto out   = skill::axle_box_bearing_housing::apply(*stock, in);
    auto cands = skill::axle_box_bearing_housing::recognize(*out.workpiece);
    ASSERT_FALSE(cands.empty());
    EXPECT_DOUBLE_EQ(cands[0].confidence, 1.0);
}
