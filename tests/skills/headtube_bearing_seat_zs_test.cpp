// @lat: [[process/test-strategy#headtube_bearing_seat_zs]]

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/headtube_bearing_seat_zs.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

using namespace koocadcam;

namespace {
double volumeOf(const TopoDS_Shape& s) {
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}

skill::headtube_bearing_seat_zs::Input goodInput()
{
    skill::headtube_bearing_seat_zs::Input in;
    in.face_id        = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.axis_origin    = gp_Pnt(0.0, 0.0, 0.0);
    in.headtube_id_mm = 30.5;
    in.bearing_od_mm  = 41.0;
    in.seat_angle_deg = 45.0;
    in.seat_depth_mm  = 8.0;
    return in;
}
}  // namespace

TEST(SkillHeadtubeBearingSeatZs, ApplyRemovesMaterial)
{
    // Head-tube stock: Ø50 x 110 mm tall (axis along Z).
    auto stock = skill::createCylindricalStock(50.0, 110.0);
    const double v0 = volumeOf(stock->shape());
    auto in  = goodInput();
    auto out = skill::headtube_bearing_seat_zs::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    const double v1 = volumeOf(out.workpiece->shape());

    EXPECT_GT(v0 - v1, 0.0);   // central bore + 2 conical seats removed
    EXPECT_GT(out.workpiece->faceCount(), stock->faceCount());
}

TEST(SkillHeadtubeBearingSeatZs, ValidateRejectsSeatNotOpening)
{
    auto stock = skill::createCylindricalStock(50.0, 110.0);
    auto in = goodInput();
    in.bearing_od_mm = 28.0;   // smaller than headtube_id -> seat doesn't open out

    auto r = skill::headtube_bearing_seat_zs::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-SEAT-OD") { found = true; break; }
    EXPECT_TRUE(found);

    EXPECT_THROW(skill::headtube_bearing_seat_zs::apply(*stock, in),
                 skill::SkillError);
}

TEST(SkillHeadtubeBearingSeatZs, ValidateRejectsBadAngle)
{
    auto stock = skill::createCylindricalStock(50.0, 110.0);
    auto in = goodInput();
    in.seat_angle_deg = 30.0;   // not the 45-degree ZS standard

    auto r = skill::headtube_bearing_seat_zs::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-ANGLE") { found = true; break; }
    EXPECT_TRUE(found);
}

TEST(SkillHeadtubeBearingSeatZs, SignatureCompoundZsSeat)
{
    auto stock = skill::createCylindricalStock(50.0, 110.0);
    auto in    = goodInput();
    auto out   = skill::headtube_bearing_seat_zs::apply(*stock, in);

    EXPECT_EQ(out.signature.skill_id, std::string("headtube_bearing_seat_zs"));
    EXPECT_TRUE(out.signature.pattern.value("is_compound", false));
    EXPECT_EQ(out.signature.pattern.value("bicycle_feature_type", std::string()),
              std::string("zero_stack_headset_bearing_seat"));
    EXPECT_EQ(out.signature.pattern.value("subfeature_count", 0), 3);
}

TEST(SkillHeadtubeBearingSeatZs, RecognizeMetadataReplay)
{
    auto stock = skill::createCylindricalStock(50.0, 110.0);
    auto in    = goodInput();
    auto out   = skill::headtube_bearing_seat_zs::apply(*stock, in);
    auto cands = skill::headtube_bearing_seat_zs::recognize(*out.workpiece);
    ASSERT_FALSE(cands.empty());
    EXPECT_DOUBLE_EQ(cands[0].confidence, 1.0);
}
