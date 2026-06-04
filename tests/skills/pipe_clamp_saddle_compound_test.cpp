// @lat: [[process/test-strategy#pipe_clamp_saddle_compound]]

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/pipe_clamp_saddle_compound.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

using namespace koocadcam;

namespace {
double volumeOf(const TopoDS_Shape& s) {
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}

skill::pipe_clamp_saddle_compound::Input goodInput()
{
    skill::pipe_clamp_saddle_compound::Input in;
    in.face_id              = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.pipe_axis_origin     = gp_Pnt(0.0, 0.0, 0.0);
    in.pipe_axis_dir        = gp_Dir(1, 0, 0);
    in.pipe_dia_mm          = 20.0;
    in.saddle_thickness_mm  = 12.0;
    in.bolt_a_xy[0] = 18.0;  in.bolt_a_xy[1] =  18.0;
    in.bolt_b_xy[0] = 18.0;  in.bolt_b_xy[1] = -18.0;
    in.bolt_thread_size_key = "M8";
    return in;
}
}  // namespace

TEST(SkillPipeClampSaddleCompound, ApplyRemovesMaterial)
{
    auto stock = skill::createCuboidStock(80.0, 50.0, 30.0);
    const double v0 = volumeOf(stock->shape());
    auto in  = goodInput();
    auto out = skill::pipe_clamp_saddle_compound::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    const double v1 = volumeOf(out.workpiece->shape());

    EXPECT_GT(v0 - v1, 0.0);   // material removed (saddle pocket + 2 bolt holes)
    EXPECT_GT(out.workpiece->faceCount(), stock->faceCount());
}

TEST(SkillPipeClampSaddleCompound, ValidateRejectsUnknownBoltThread)
{
    auto stock = skill::createCuboidStock(80.0, 50.0, 30.0);
    auto in = goodInput();
    in.bolt_thread_size_key = "M99";

    auto r = skill::pipe_clamp_saddle_compound::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-M-THREAD") { found = true; break; }
    EXPECT_TRUE(found);

    EXPECT_THROW(skill::pipe_clamp_saddle_compound::apply(*stock, in),
                 skill::SkillError);
}

TEST(SkillPipeClampSaddleCompound, SignatureCompoundThreeSubfeatures)
{
    auto stock = skill::createCuboidStock(80.0, 50.0, 30.0);
    auto in    = goodInput();
    auto out   = skill::pipe_clamp_saddle_compound::apply(*stock, in);

    EXPECT_EQ(out.signature.skill_id, std::string("pipe_clamp_saddle_compound"));
    EXPECT_TRUE(out.signature.pattern.value("is_compound", false));
    EXPECT_EQ(out.signature.pattern.value("hvac_feature_type", std::string()),
              std::string("pipe_clamp_saddle"));
    EXPECT_EQ(out.signature.pattern.value("subfeature_count", 0), 3);
    EXPECT_EQ(out.signature.pattern.value("bolt_count", 0), 2);
}

TEST(SkillPipeClampSaddleCompound, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(80.0, 50.0, 30.0);
    auto in    = goodInput();
    auto out   = skill::pipe_clamp_saddle_compound::apply(*stock, in);
    auto cands = skill::pipe_clamp_saddle_compound::recognize(*out.workpiece);
    ASSERT_FALSE(cands.empty());
    EXPECT_DOUBLE_EQ(cands[0].confidence, 1.0);
}
