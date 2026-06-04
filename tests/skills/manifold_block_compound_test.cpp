// @lat: [[process/test-strategy#manifold_block_compound]]

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/manifold_block_compound.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

using namespace koocadcam;

namespace {
double volumeOf(const TopoDS_Shape& s) {
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}

skill::manifold_block_compound::Input goodInput()
{
    skill::manifold_block_compound::Input in;
    in.face_id                = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.main_bore_origin       = gp_Pnt(20.0, 40.0, 20.0);
    in.main_axis_dir          = gp_Dir(1, 0, 0);
    in.main_dia_mm            = 16.0;
    in.main_length_mm         = 80.0;
    in.branch_count           = 4;
    in.branch_dia_mm          = 8.0;
    in.branch_pitch_mm        = 18.0;
    in.branch_thread_size_key = "G1/4";
    return in;
}
}  // namespace

TEST(SkillManifoldBlockCompound, ApplyRemovesMaterial)
{
    auto stock = skill::createCuboidStock(120.0, 80.0, 40.0);
    const double v0 = volumeOf(stock->shape());
    auto in  = goodInput();
    auto out = skill::manifold_block_compound::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    const double v1 = volumeOf(out.workpiece->shape());

    EXPECT_GT(v0 - v1, 0.0);   // material removed (main + 4 branches)
    EXPECT_GT(out.workpiece->faceCount(), stock->faceCount());
}

TEST(SkillManifoldBlockCompound, ValidateRejectsUnknownBspThread)
{
    auto stock = skill::createCuboidStock(120.0, 80.0, 40.0);
    auto in = goodInput();
    in.branch_thread_size_key = "G99";

    auto r = skill::manifold_block_compound::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-BSP-CODE") { found = true; break; }
    EXPECT_TRUE(found);

    EXPECT_THROW(skill::manifold_block_compound::apply(*stock, in),
                 skill::SkillError);
}

TEST(SkillManifoldBlockCompound, SignatureCompoundBranchCount)
{
    auto stock = skill::createCuboidStock(120.0, 80.0, 40.0);
    auto in    = goodInput();
    auto out   = skill::manifold_block_compound::apply(*stock, in);

    EXPECT_EQ(out.signature.skill_id, std::string("manifold_block_compound"));
    EXPECT_TRUE(out.signature.pattern.value("is_compound", false));
    EXPECT_EQ(out.signature.pattern.value("hvac_feature_type", std::string()),
              std::string("hydraulic_manifold_block"));
    EXPECT_EQ(out.signature.pattern.value("subfeature_count", 0),
              1 + in.branch_count);
    EXPECT_EQ(out.signature.pattern.value("branch_count", 0), in.branch_count);
}

TEST(SkillManifoldBlockCompound, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(120.0, 80.0, 40.0);
    auto in    = goodInput();
    auto out   = skill::manifold_block_compound::apply(*stock, in);
    auto cands = skill::manifold_block_compound::recognize(*out.workpiece);
    ASSERT_FALSE(cands.empty());
    EXPECT_DOUBLE_EQ(cands[0].confidence, 1.0);
}
