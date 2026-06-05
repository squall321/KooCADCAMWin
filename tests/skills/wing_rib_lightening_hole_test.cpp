// @lat: [[process/test-strategy#wing_rib_lightening_hole]]

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/wing_rib_lightening_hole.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

using namespace koocadcam;

namespace {
double volumeOf(const TopoDS_Shape& s) {
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}

skill::wing_rib_lightening_hole::Input goodInput()
{
    skill::wing_rib_lightening_hole::Input in;
    in.face_id         = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.row_origin      = gp_Pnt(25.0, 40.0, 0.0);
    in.hole_dia_mm     = 20.0;
    in.flange_width_mm = 3.0;
    in.hole_count      = 3;
    in.pitch_mm        = 35.0;
    return in;
}
}  // namespace

TEST(SkillWingRibLighteningHole, ApplyRemovesMaterial)
{
    auto stock = skill::createCuboidStock(140.0, 80.0, 8.0);
    const double v0 = volumeOf(stock->shape());
    auto in  = goodInput();
    auto out = skill::wing_rib_lightening_hole::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    const double v1 = volumeOf(out.workpiece->shape());

    EXPECT_GT(v0 - v1, 0.0);
    EXPECT_GT(out.workpiece->faceCount(), stock->faceCount());
}

TEST(SkillWingRibLighteningHole, ValidateRejectsBadPitch)
{
    auto stock = skill::createCuboidStock(140.0, 80.0, 8.0);
    auto in = goodInput();
    in.pitch_mm = 22.0;   // < dia(20) + 2*flange(3) = 26

    auto r = skill::wing_rib_lightening_hole::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-PITCH") { found = true; break; }
    EXPECT_TRUE(found);

    EXPECT_THROW(skill::wing_rib_lightening_hole::apply(*stock, in),
                 skill::SkillError);
}

TEST(SkillWingRibLighteningHole, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(140.0, 80.0, 8.0);
    auto in = goodInput();
    in.hole_count = 0;

    auto r = skill::wing_rib_lightening_hole::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-INPUT") { found = true; break; }
    EXPECT_TRUE(found);
}

TEST(SkillWingRibLighteningHole, SignatureCompound)
{
    auto stock = skill::createCuboidStock(140.0, 80.0, 8.0);
    auto in    = goodInput();
    auto out   = skill::wing_rib_lightening_hole::apply(*stock, in);

    EXPECT_TRUE(out.signature.pattern.value("is_compound", false));
    EXPECT_EQ(out.signature.pattern.value("aerostruct_feature_type", std::string()),
              std::string("wing_rib_lightening_hole"));
    EXPECT_EQ(out.signature.pattern.value("subfeature_count", 0), 3 * 2);
}

TEST(SkillWingRibLighteningHole, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(140.0, 80.0, 8.0);
    auto in    = goodInput();
    auto out   = skill::wing_rib_lightening_hole::apply(*stock, in);
    auto cands = skill::wing_rib_lightening_hole::recognize(*out.workpiece);
    ASSERT_FALSE(cands.empty());
    EXPECT_DOUBLE_EQ(cands[0].confidence, 1.0);
}
