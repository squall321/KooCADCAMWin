// @lat: [[process/test-strategy#shaft_collar_clamp_split]]

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/shaft_collar_clamp_split.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

using namespace koocadcam;

namespace {
double volumeOf(const TopoDS_Shape& s) {
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}

skill::shaft_collar_clamp_split::Input goodInput()
{
    skill::shaft_collar_clamp_split::Input in;
    in.face_id          = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.axis_origin      = gp_Pnt(0.0, 0.0, 0.0);
    in.bore_dia_mm      = 20.0;
    in.collar_od_mm     = 40.0;
    in.split_width_mm   = 3.0;
    in.clamp_thread_key = "M6";
    return in;
}
}  // namespace

TEST(SkillShaftCollarClampSplit, ApplyRemovesMaterial)
{
    auto stock = skill::createCylindricalStock(40.0, 20.0);
    const double v0 = volumeOf(stock->shape());
    auto in  = goodInput();
    auto out = skill::shaft_collar_clamp_split::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    const double v1 = volumeOf(out.workpiece->shape());

    EXPECT_GT(v0 - v1, 0.0);   // bore + split + clamp bore removed
    EXPECT_GT(out.workpiece->faceCount(), stock->faceCount());
}

TEST(SkillShaftCollarClampSplit, ValidateRejectsUnknownThread)
{
    auto stock = skill::createCylindricalStock(40.0, 20.0);
    auto in = goodInput();
    in.clamp_thread_key = "M99";

    auto r = skill::shaft_collar_clamp_split::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-PT-THREAD") { found = true; break; }
    EXPECT_TRUE(found);

    EXPECT_THROW(skill::shaft_collar_clamp_split::apply(*stock, in),
                 skill::SkillError);
}

TEST(SkillShaftCollarClampSplit, ValidateRejectsSplitTooWide)
{
    auto stock = skill::createCylindricalStock(40.0, 20.0);
    auto in = goodInput();
    in.split_width_mm = 12.0;   // >= wall thickness (10 mm)

    auto r = skill::shaft_collar_clamp_split::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-PT-SPLIT") { found = true; break; }
    EXPECT_TRUE(found);
}

TEST(SkillShaftCollarClampSplit, SignatureCompoundClampCollar)
{
    auto stock = skill::createCylindricalStock(40.0, 20.0);
    auto in    = goodInput();
    auto out   = skill::shaft_collar_clamp_split::apply(*stock, in);

    EXPECT_EQ(out.signature.skill_id,
              std::string("shaft_collar_clamp_split"));
    EXPECT_TRUE(out.signature.pattern.value("is_compound", false));
    EXPECT_EQ(out.signature.pattern.value("powertrans_feature_type", std::string()),
              std::string("clamp_split_shaft_collar"));
    EXPECT_EQ(out.signature.pattern.value("subfeature_count", 0), 3);
}

TEST(SkillShaftCollarClampSplit, RecognizeMetadataReplay)
{
    auto stock = skill::createCylindricalStock(40.0, 20.0);
    auto in    = goodInput();
    auto out   = skill::shaft_collar_clamp_split::apply(*stock, in);
    auto cands = skill::shaft_collar_clamp_split::recognize(*out.workpiece);
    ASSERT_FALSE(cands.empty());
    EXPECT_DOUBLE_EQ(cands[0].confidence, 1.0);
}
