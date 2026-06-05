// @lat: [[process/test-strategy#chainsaw_bar_chain_groove]]

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/chainsaw_bar_chain_groove.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

using namespace koocadcam;

namespace {
double volumeOf(const TopoDS_Shape& s) {
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}

skill::chainsaw_bar_chain_groove::Input goodInput()
{
    skill::chainsaw_bar_chain_groove::Input in;
    in.face_id            = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.bar_axis_origin    = gp_Pnt(20.0, 25.0, 0.0);
    in.bar_length_mm      = 400.0;
    in.groove_width_mm    = 1.5;
    in.groove_depth_mm    = 7.5;
    in.sprocket_radius_mm = 18.0;
    return in;
}
}  // namespace

TEST(SkillChainsawBarChainGroove, ApplyRemovesMaterial)
{
    auto stock = skill::createCuboidStock(450.0, 50.0, 8.0);
    const double v0 = volumeOf(stock->shape());
    auto in  = goodInput();
    in.bar_axis_origin = gp_Pnt(20.0, 25.0, 0.0);
    auto out = skill::chainsaw_bar_chain_groove::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    const double v1 = volumeOf(out.workpiece->shape());

    EXPECT_GT(v0 - v1, 0.0);   // material removed (groove + sprocket relief)
}

TEST(SkillChainsawBarChainGroove, ValidateRejectsBadGrooveWidth)
{
    auto stock = skill::createCuboidStock(450.0, 50.0, 8.0);
    auto in = goodInput();
    in.groove_width_mm = 5.0;  // way out of spec

    auto r = skill::chainsaw_bar_chain_groove::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-GROOVE") { found = true; break; }
    EXPECT_TRUE(found);

    EXPECT_THROW(skill::chainsaw_bar_chain_groove::apply(*stock, in),
                 skill::SkillError);
}

TEST(SkillChainsawBarChainGroove, SignatureCompoundAgFeatureType)
{
    auto stock = skill::createCuboidStock(450.0, 50.0, 8.0);
    auto in    = goodInput();
    auto out   = skill::chainsaw_bar_chain_groove::apply(*stock, in);

    EXPECT_EQ(out.signature.skill_id, std::string("chainsaw_bar_chain_groove"));
    EXPECT_TRUE(out.signature.pattern.value("is_compound", false));
    EXPECT_EQ(out.signature.pattern.value("ag_feature_type", std::string()),
              std::string("chainsaw_guide_bar_groove"));
    EXPECT_EQ(out.signature.pattern.value("subfeature_count", 0), 2);
}

TEST(SkillChainsawBarChainGroove, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(450.0, 50.0, 8.0);
    auto in    = goodInput();
    auto out   = skill::chainsaw_bar_chain_groove::apply(*stock, in);
    auto cands = skill::chainsaw_bar_chain_groove::recognize(*out.workpiece);
    ASSERT_FALSE(cands.empty());
    EXPECT_DOUBLE_EQ(cands[0].confidence, 1.0);
}
