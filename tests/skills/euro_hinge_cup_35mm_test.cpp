// @lat: [[process/test-strategy#euro_hinge_cup_35mm]]

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/euro_hinge_cup_35mm.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

using namespace koocadcam;

namespace {
double volumeOf(const TopoDS_Shape& s) {
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}

skill::euro_hinge_cup_35mm::Input goodInput()
{
    skill::euro_hinge_cup_35mm::Input in;
    in.face_id            = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.center_xy          = gp_Pnt(50.0, 50.0, 0.0);
    in.cup_dia_mm         = 35.0;
    in.cup_depth_mm       = 11.5;
    in.screw_spacing_mm   = 45.0;
    in.screw_pilot_dia_mm = 2.5;
    return in;
}
}  // namespace

TEST(SkillEuroHingeCup35mm, ApplyRemovesMaterial)
{
    auto stock = skill::createCuboidStock(100.0, 100.0, 18.0);
    const double v0 = volumeOf(stock->shape());
    auto in  = goodInput();
    auto out = skill::euro_hinge_cup_35mm::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    const double v1 = volumeOf(out.workpiece->shape());

    EXPECT_GT(v0 - v1, 0.0);   // material removed (cup + 2 pilots)
    EXPECT_GT(out.workpiece->faceCount(), stock->faceCount());
}

TEST(SkillEuroHingeCup35mm, ValidateRejectsCupDiaOutOfRange)
{
    auto stock = skill::createCuboidStock(100.0, 100.0, 18.0);
    auto in = goodInput();
    in.cup_dia_mm = 50.0;   // > 40

    auto r = skill::euro_hinge_cup_35mm::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-CUPDIA") { found = true; break; }
    EXPECT_TRUE(found);

    EXPECT_THROW(skill::euro_hinge_cup_35mm::apply(*stock, in),
                 skill::SkillError);
}

TEST(SkillEuroHingeCup35mm, ValidateRejectsPilotIntoCup)
{
    auto stock = skill::createCuboidStock(100.0, 100.0, 18.0);
    auto in = goodInput();
    in.screw_spacing_mm = 20.0;   // pilots break into the 35 mm cup

    auto r = skill::euro_hinge_cup_35mm::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-PILOT") { found = true; break; }
    EXPECT_TRUE(found);
}

TEST(SkillEuroHingeCup35mm, SignatureCompound)
{
    auto stock = skill::createCuboidStock(100.0, 100.0, 18.0);
    auto in    = goodInput();
    auto out   = skill::euro_hinge_cup_35mm::apply(*stock, in);

    EXPECT_TRUE(out.signature.pattern.value("is_compound", false));
    EXPECT_EQ(out.signature.pattern.value("furniture_feature_type", std::string()),
              std::string("euro_concealed_hinge_cup"));
    EXPECT_EQ(out.signature.pattern.value("subfeature_count", 0), 3);
}

TEST(SkillEuroHingeCup35mm, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(100.0, 100.0, 18.0);
    auto in    = goodInput();
    auto out   = skill::euro_hinge_cup_35mm::apply(*stock, in);
    auto cands = skill::euro_hinge_cup_35mm::recognize(*out.workpiece);
    ASSERT_FALSE(cands.empty());
    EXPECT_DOUBLE_EQ(cands[0].confidence, 1.0);
}
