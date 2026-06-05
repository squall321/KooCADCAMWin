// @lat: [[process/test-strategy#boat_cleat_mounting_compound]]

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/boat_cleat_mounting_compound.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

using namespace koocadcam;

namespace {
double volumeOf(const TopoDS_Shape& s) {
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}

skill::boat_cleat_mounting_compound::Input goodInput()
{
    skill::boat_cleat_mounting_compound::Input in;
    in.face_id              = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.center_xy            = gp_Pnt(60.0, 40.0, 0.0);
    in.cleat_length_mm      = 120.0;
    in.bolt_spacing_mm      = 80.0;
    in.bolt_thread_size_key = "M8";
    in.pad_depth_mm         = 4.0;
    return in;
}
}  // namespace

TEST(SkillBoatCleatMountingCompound, ApplyRemovesMaterial)
{
    auto stock = skill::createCuboidStock(160.0, 100.0, 20.0);
    const double v0 = volumeOf(stock->shape());
    auto in  = goodInput();
    auto out = skill::boat_cleat_mounting_compound::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    const double v1 = volumeOf(out.workpiece->shape());

    EXPECT_GT(v0 - v1, 0.0);   // material removed (pad + 2 holes)
    EXPECT_GT(out.workpiece->faceCount(), stock->faceCount());
}

TEST(SkillBoatCleatMountingCompound, ValidateRejectsUnknownThread)
{
    auto stock = skill::createCuboidStock(160.0, 100.0, 20.0);
    auto in = goodInput();
    in.bolt_thread_size_key = "M99";

    auto r = skill::boat_cleat_mounting_compound::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-M-THREAD") { found = true; break; }
    EXPECT_TRUE(found);

    EXPECT_THROW(skill::boat_cleat_mounting_compound::apply(*stock, in),
                 skill::SkillError);
}

TEST(SkillBoatCleatMountingCompound, ValidateRejectsSpacingTooWide)
{
    auto stock = skill::createCuboidStock(160.0, 100.0, 20.0);
    auto in = goodInput();
    in.bolt_spacing_mm = 130.0;   // >= cleat_length_mm (120)

    auto r = skill::boat_cleat_mounting_compound::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-MARINE-SPAN") { found = true; break; }
    EXPECT_TRUE(found);
}

TEST(SkillBoatCleatMountingCompound, SignatureCompoundCleat)
{
    auto stock = skill::createCuboidStock(160.0, 100.0, 20.0);
    auto in    = goodInput();
    auto out   = skill::boat_cleat_mounting_compound::apply(*stock, in);

    EXPECT_EQ(out.signature.skill_id,
              std::string("boat_cleat_mounting_compound"));
    EXPECT_TRUE(out.signature.pattern.value("is_compound", false));
    EXPECT_EQ(out.signature.pattern.value("marine_feature_type", std::string()),
              std::string("horn_cleat_mounting_base"));
    EXPECT_EQ(out.signature.pattern.value("subfeature_count", 0), 3);
}

TEST(SkillBoatCleatMountingCompound, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(160.0, 100.0, 20.0);
    auto in    = goodInput();
    auto out   = skill::boat_cleat_mounting_compound::apply(*stock, in);
    auto cands = skill::boat_cleat_mounting_compound::recognize(*out.workpiece);
    ASSERT_FALSE(cands.empty());
    EXPECT_DOUBLE_EQ(cands[0].confidence, 1.0);
}
