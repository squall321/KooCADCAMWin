// @lat: [[process/test-strategy#dowel_joint_boring_pair]]

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/dowel_joint_boring_pair.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

using namespace koocadcam;

namespace {
double volumeOf(const TopoDS_Shape& s) {
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}

skill::dowel_joint_boring_pair::Input goodInput()
{
    skill::dowel_joint_boring_pair::Input in;
    in.face_id              = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.row_origin           = gp_Pnt(20.0, 40.0, 0.0);
    in.dowel_dia_mm         = 8.0;
    in.dowel_count          = 3;
    in.pitch_mm             = 64.0;
    in.dowel_depth_mm       = 15.0;
    in.glue_groove_width_mm = 3.0;
    in.glue_groove_depth_mm = 1.0;
    return in;
}
}  // namespace

TEST(SkillDowelJointBoringPair, ApplyRemovesMaterial)
{
    auto stock = skill::createCuboidStock(180.0, 80.0, 25.0);
    const double v0 = volumeOf(stock->shape());
    auto in  = goodInput();
    auto out = skill::dowel_joint_boring_pair::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    const double v1 = volumeOf(out.workpiece->shape());

    EXPECT_GT(v0 - v1, 0.0);   // glue groove + N dowel holes
    EXPECT_GT(out.workpiece->faceCount(), stock->faceCount());
}

TEST(SkillDowelJointBoringPair, ValidateRejectsOverlapPitch)
{
    auto stock = skill::createCuboidStock(180.0, 80.0, 25.0);
    auto in = goodInput();
    in.pitch_mm = 6.0;   // <= dowel_dia_mm

    auto r = skill::dowel_joint_boring_pair::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-PITCH") { found = true; break; }
    EXPECT_TRUE(found);

    EXPECT_THROW(skill::dowel_joint_boring_pair::apply(*stock, in),
                 skill::SkillError);
}

TEST(SkillDowelJointBoringPair, ValidateRejectsWideGroove)
{
    auto stock = skill::createCuboidStock(180.0, 80.0, 25.0);
    auto in = goodInput();
    in.glue_groove_width_mm = 10.0;   // >= dowel_dia_mm (8)

    auto r = skill::dowel_joint_boring_pair::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-GROOVE") { found = true; break; }
    EXPECT_TRUE(found);
}

TEST(SkillDowelJointBoringPair, SignatureCompound)
{
    auto stock = skill::createCuboidStock(180.0, 80.0, 25.0);
    auto in    = goodInput();
    auto out   = skill::dowel_joint_boring_pair::apply(*stock, in);

    EXPECT_TRUE(out.signature.pattern.value("is_compound", false));
    EXPECT_EQ(out.signature.pattern.value("furniture_feature_type", std::string()),
              std::string("dowel_joint_with_glue_groove"));
    EXPECT_EQ(out.signature.pattern.value("subfeature_count", 0), 4);
}

TEST(SkillDowelJointBoringPair, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(180.0, 80.0, 25.0);
    auto in    = goodInput();
    auto out   = skill::dowel_joint_boring_pair::apply(*stock, in);
    auto cands = skill::dowel_joint_boring_pair::recognize(*out.workpiece);
    ASSERT_FALSE(cands.empty());
    EXPECT_DOUBLE_EQ(cands[0].confidence, 1.0);
}
