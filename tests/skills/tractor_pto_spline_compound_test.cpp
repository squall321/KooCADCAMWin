// @lat: [[process/test-strategy#tractor_pto_spline_compound]]

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/tractor_pto_spline_compound.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

using namespace koocadcam;

namespace {
double volumeOf(const TopoDS_Shape& s) {
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}

skill::tractor_pto_spline_compound::Input goodInput()
{
    skill::tractor_pto_spline_compound::Input in;
    in.face_id          = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.pto_type         = "type1_6spline";
    in.shaft_dia_mm     = 34.92;
    in.spline_length_mm = 50.0;
    in.shaft_origin     = gp_Pnt(0.0, 0.0, 0.0);
    return in;
}
}  // namespace

TEST(SkillTractorPtoSplineCompound, ApplyRemovesMaterial)
{
    auto stock = skill::createCylindricalStock(35.0, 80.0);
    const double v0 = volumeOf(stock->shape());
    auto in  = goodInput();
    auto out = skill::tractor_pto_spline_compound::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    const double v1 = volumeOf(out.workpiece->shape());

    EXPECT_GT(v0 - v1, 0.0);   // material removed (6 spline slots)
    EXPECT_GT(out.workpiece->faceCount(), stock->faceCount());
}

TEST(SkillTractorPtoSplineCompound, ValidateRejectsUnknownPtoType)
{
    auto stock = skill::createCylindricalStock(35.0, 80.0);
    auto in = goodInput();
    in.pto_type = "bogus_pto_type";

    auto r = skill::tractor_pto_spline_compound::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-PTO-TYPE") { found = true; break; }
    EXPECT_TRUE(found);

    EXPECT_THROW(skill::tractor_pto_spline_compound::apply(*stock, in),
                 skill::SkillError);
}

TEST(SkillTractorPtoSplineCompound, SignatureCompoundSplineCount)
{
    auto stock = skill::createCylindricalStock(35.0, 80.0);
    auto in    = goodInput();
    auto out   = skill::tractor_pto_spline_compound::apply(*stock, in);

    EXPECT_EQ(out.signature.skill_id, std::string("tractor_pto_spline_compound"));
    EXPECT_TRUE(out.signature.pattern.value("is_compound", false));
    EXPECT_EQ(out.signature.pattern.value("ag_feature_type", std::string()),
              std::string("tractor_pto_spline"));
    // Type 1 = 6 spline → subfeature_count = 6
    EXPECT_EQ(out.signature.pattern.value("subfeature_count", 0), 6);
    EXPECT_EQ(out.signature.pattern.value("spline_count", 0), 6);
}

TEST(SkillTractorPtoSplineCompound, RecognizeMetadataReplay)
{
    auto stock = skill::createCylindricalStock(35.0, 80.0);
    auto in    = goodInput();
    auto out   = skill::tractor_pto_spline_compound::apply(*stock, in);
    auto cands = skill::tractor_pto_spline_compound::recognize(*out.workpiece);
    ASSERT_FALSE(cands.empty());
    EXPECT_DOUBLE_EQ(cands[0].confidence, 1.0);
}
