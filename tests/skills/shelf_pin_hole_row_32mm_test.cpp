// @lat: [[process/test-strategy#shelf_pin_hole_row_32mm]]

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/shelf_pin_hole_row_32mm.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

using namespace koocadcam;

namespace {
double volumeOf(const TopoDS_Shape& s) {
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}

skill::shelf_pin_hole_row_32mm::Input goodInput()
{
    skill::shelf_pin_hole_row_32mm::Input in;
    in.face_id       = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.row_origin    = gp_Pnt(20.0, 40.0, 0.0);
    in.pin_dia_mm    = 5.0;
    in.hole_count    = 6;
    in.pitch_mm      = 32.0;
    in.hole_depth_mm = 12.0;
    return in;
}
}  // namespace

TEST(SkillShelfPinHoleRow32mm, ApplyRemovesMaterial)
{
    auto stock = skill::createCuboidStock(220.0, 80.0, 18.0);
    const double v0 = volumeOf(stock->shape());
    auto in  = goodInput();
    auto out = skill::shelf_pin_hole_row_32mm::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    const double v1 = volumeOf(out.workpiece->shape());

    EXPECT_GT(v0 - v1, 0.0);   // N shelf-pin holes
    EXPECT_GT(out.workpiece->faceCount(), stock->faceCount());
}

TEST(SkillShelfPinHoleRow32mm, ValidateRejectsOverlapPitch)
{
    auto stock = skill::createCuboidStock(220.0, 80.0, 18.0);
    auto in = goodInput();
    in.pitch_mm = 4.0;   // <= pin_dia_mm

    auto r = skill::shelf_pin_hole_row_32mm::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-PITCH") { found = true; break; }
    EXPECT_TRUE(found);

    EXPECT_THROW(skill::shelf_pin_hole_row_32mm::apply(*stock, in),
                 skill::SkillError);
}

TEST(SkillShelfPinHoleRow32mm, ValidateRejectsThroughDepth)
{
    auto stock = skill::createCuboidStock(220.0, 80.0, 18.0);
    auto in = goodInput();
    in.hole_depth_mm = 25.0;   // >= panel thickness (18)

    auto r = skill::shelf_pin_hole_row_32mm::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-DEPTH") { found = true; break; }
    EXPECT_TRUE(found);
}

TEST(SkillShelfPinHoleRow32mm, SignatureCompound)
{
    auto stock = skill::createCuboidStock(220.0, 80.0, 18.0);
    auto in    = goodInput();
    auto out   = skill::shelf_pin_hole_row_32mm::apply(*stock, in);

    EXPECT_TRUE(out.signature.pattern.value("is_compound", false));
    EXPECT_EQ(out.signature.pattern.value("furniture_feature_type", std::string()),
              std::string("system32_shelf_pin_row"));
    EXPECT_EQ(out.signature.pattern.value("subfeature_count", 0), 6);
}

TEST(SkillShelfPinHoleRow32mm, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(220.0, 80.0, 18.0);
    auto in    = goodInput();
    auto out   = skill::shelf_pin_hole_row_32mm::apply(*stock, in);
    auto cands = skill::shelf_pin_hole_row_32mm::recognize(*out.workpiece);
    ASSERT_FALSE(cands.empty());
    EXPECT_DOUBLE_EQ(cands[0].confidence, 1.0);
}
