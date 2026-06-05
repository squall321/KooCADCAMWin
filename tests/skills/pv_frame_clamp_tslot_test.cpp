// @lat: [[process/test-strategy#pv_frame_clamp_tslot]]

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/pv_frame_clamp_tslot.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

using namespace koocadcam;

namespace {
double volumeOf(const TopoDS_Shape& s) {
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}

skill::pv_frame_clamp_tslot::Input goodInput()
{
    skill::pv_frame_clamp_tslot::Input in;
    in.face_id        = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.center_xy      = gp_Pnt(40.0, 40.0, 0.0);
    in.slot_width_mm  = 10.0;
    in.slot_depth_mm  = 6.0;
    in.slot_length_mm = 40.0;
    in.tooth_count    = 4;
    in.tooth_depth_mm = 0.8;
    return in;
}
}  // namespace

TEST(SkillPvFrameClampTslot, ApplyRemovesMaterial)
{
    auto stock = skill::createCuboidStock(80.0, 80.0, 30.0);
    const double v0 = volumeOf(stock->shape());
    auto in  = goodInput();
    auto out = skill::pv_frame_clamp_tslot::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    const double v1 = volumeOf(out.workpiece->shape());

    EXPECT_GT(v0 - v1, 0.0);
    EXPECT_GT(out.workpiece->faceCount(), stock->faceCount());
}

TEST(SkillPvFrameClampTslot, ValidateRejectsBadTeeth)
{
    auto stock = skill::createCuboidStock(80.0, 80.0, 30.0);
    auto in = goodInput();
    in.tooth_count = 12;

    auto r = skill::pv_frame_clamp_tslot::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-TEETH") { found = true; break; }
    EXPECT_TRUE(found);

    EXPECT_THROW(skill::pv_frame_clamp_tslot::apply(*stock, in),
                 skill::SkillError);
}

TEST(SkillPvFrameClampTslot, ValidateRejectsZeroDim)
{
    auto stock = skill::createCuboidStock(80.0, 80.0, 30.0);
    auto in = goodInput();
    in.slot_depth_mm = 0.0;

    auto r = skill::pv_frame_clamp_tslot::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-INPUT") { found = true; break; }
    EXPECT_TRUE(found);
}

TEST(SkillPvFrameClampTslot, SignatureCompound)
{
    auto stock = skill::createCuboidStock(80.0, 80.0, 30.0);
    auto in    = goodInput();
    auto out   = skill::pv_frame_clamp_tslot::apply(*stock, in);

    EXPECT_TRUE(out.signature.pattern.value("is_compound", false));
    EXPECT_EQ(out.signature.pattern.value("solar_feature_type", std::string()),
              std::string("frame_clamp_tslot_grounding"));
    EXPECT_EQ(out.signature.pattern.value("subfeature_count", 0),
              1 + in.tooth_count);
}

TEST(SkillPvFrameClampTslot, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(80.0, 80.0, 30.0);
    auto in    = goodInput();
    auto out   = skill::pv_frame_clamp_tslot::apply(*stock, in);
    auto cands = skill::pv_frame_clamp_tslot::recognize(*out.workpiece);
    ASSERT_FALSE(cands.empty());
    EXPECT_DOUBLE_EQ(cands[0].confidence, 1.0);
}
