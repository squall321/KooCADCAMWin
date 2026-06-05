// @lat: [[process/test-strategy#harvester_blade_arc_compound]]

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/harvester_blade_arc_compound.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

using namespace koocadcam;

namespace {
double volumeOf(const TopoDS_Shape& s) {
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}

skill::harvester_blade_arc_compound::Input goodInput()
{
    skill::harvester_blade_arc_compound::Input in;
    in.face_id             = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.blade_origin        = gp_Pnt(20.0, 0.0, 0.0);
    in.blade_section_count = 8;
    in.section_pitch_mm    = 76.2;
    in.blade_height_mm     = 6.0;
    in.edge_angle_deg      = 20.0;
    return in;
}
}  // namespace

TEST(SkillHarvesterBladeArcCompound, ApplyRemovesMaterial)
{
    // Stock long enough for 8 sections * 76.2 + start offset
    auto stock = skill::createCuboidStock(700.0, 60.0, 8.0);
    const double v0 = volumeOf(stock->shape());
    auto in  = goodInput();
    auto out = skill::harvester_blade_arc_compound::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    const double v1 = volumeOf(out.workpiece->shape());

    EXPECT_GT(v0 - v1, 0.0);   // material removed (sickle tooth gaps)
    EXPECT_GT(out.workpiece->faceCount(), stock->faceCount());
}

TEST(SkillHarvesterBladeArcCompound, ValidateRejectsBadEdgeAngle)
{
    auto stock = skill::createCuboidStock(700.0, 60.0, 8.0);
    auto in = goodInput();
    in.edge_angle_deg = 60.0;  // > 35°

    auto r = skill::harvester_blade_arc_compound::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-EDGE-ANGLE") { found = true; break; }
    EXPECT_TRUE(found);

    EXPECT_THROW(skill::harvester_blade_arc_compound::apply(*stock, in),
                 skill::SkillError);
}

TEST(SkillHarvesterBladeArcCompound, SignatureCompoundSectionCount)
{
    auto stock = skill::createCuboidStock(700.0, 60.0, 8.0);
    auto in    = goodInput();
    auto out   = skill::harvester_blade_arc_compound::apply(*stock, in);

    EXPECT_EQ(out.signature.skill_id, std::string("harvester_blade_arc_compound"));
    EXPECT_TRUE(out.signature.pattern.value("is_compound", false));
    EXPECT_EQ(out.signature.pattern.value("ag_feature_type", std::string()),
              std::string("harvester_sickle_blade"));
    EXPECT_EQ(out.signature.pattern.value("subfeature_count", 0),
              in.blade_section_count);
}

TEST(SkillHarvesterBladeArcCompound, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(700.0, 60.0, 8.0);
    auto in    = goodInput();
    auto out   = skill::harvester_blade_arc_compound::apply(*stock, in);
    auto cands = skill::harvester_blade_arc_compound::recognize(*out.workpiece);
    ASSERT_FALSE(cands.empty());
    EXPECT_DOUBLE_EQ(cands[0].confidence, 1.0);
}
