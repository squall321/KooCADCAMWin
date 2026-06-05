// @lat: [[process/test-strategy#blade_root_tbolt_compound]]

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/blade_root_tbolt_compound.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

using namespace koocadcam;

namespace {
double volumeOf(const TopoDS_Shape& s) {
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}

skill::blade_root_tbolt_compound::Input goodInput()
{
    skill::blade_root_tbolt_compound::Input in;
    in.face_id          = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.axis_origin      = gp_Pnt(40.0, 40.0, 0.0);
    in.stud_bore_dia_mm = 24.0;
    in.stud_depth_mm    = 120.0;
    in.barrel_nut_dia_mm= 30.0;
    in.barrel_depth_mm  = 50.0;
    in.stud_thread_key  = "M24";
    return in;
}
}  // namespace

TEST(SkillBladeRootTBoltCompound, ApplyRemovesMaterial)
{
    auto stock = skill::createCuboidStock(80.0, 80.0, 160.0);
    const double v0 = volumeOf(stock->shape());
    auto in  = goodInput();
    auto out = skill::blade_root_tbolt_compound::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    const double v1 = volumeOf(out.workpiece->shape());

    EXPECT_GT(v0 - v1, 0.0);   // stud bore + barrel cross bore removed
    EXPECT_GT(out.workpiece->faceCount(), stock->faceCount());
}

TEST(SkillBladeRootTBoltCompound, ValidateRejectsUnknownThread)
{
    auto stock = skill::createCuboidStock(80.0, 80.0, 160.0);
    auto in = goodInput();
    in.stud_thread_key = "M99";

    auto r = skill::blade_root_tbolt_compound::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-THREAD") { found = true; break; }
    EXPECT_TRUE(found);

    EXPECT_THROW(skill::blade_root_tbolt_compound::apply(*stock, in),
                 skill::SkillError);
}

TEST(SkillBladeRootTBoltCompound, ValidateRejectsShallowBarrel)
{
    auto stock = skill::createCuboidStock(80.0, 80.0, 160.0);
    auto in = goodInput();
    in.barrel_depth_mm = 5.0;   // < stud radius (12 mm) → no intersection

    auto r = skill::blade_root_tbolt_compound::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-INTERSECT") { found = true; break; }
    EXPECT_TRUE(found);
}

TEST(SkillBladeRootTBoltCompound, SignatureCompound)
{
    auto stock = skill::createCuboidStock(80.0, 80.0, 160.0);
    auto in    = goodInput();
    auto out   = skill::blade_root_tbolt_compound::apply(*stock, in);

    EXPECT_EQ(out.signature.skill_id,
              std::string("blade_root_tbolt_compound"));
    EXPECT_TRUE(out.signature.pattern.value("is_compound", false));
    EXPECT_EQ(out.signature.pattern.value("wind_feature_type", std::string()),
              std::string("blade_root_tbolt_joint"));
    EXPECT_EQ(out.signature.pattern.value("subfeature_count", 0), 2);
}

TEST(SkillBladeRootTBoltCompound, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(80.0, 80.0, 160.0);
    auto in    = goodInput();
    auto out   = skill::blade_root_tbolt_compound::apply(*stock, in);
    auto cands = skill::blade_root_tbolt_compound::recognize(*out.workpiece);
    ASSERT_FALSE(cands.empty());
    EXPECT_DOUBLE_EQ(cands[0].confidence, 1.0);
}
