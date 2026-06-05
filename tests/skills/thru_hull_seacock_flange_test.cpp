// @lat: [[process/test-strategy#thru_hull_seacock_flange]]

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/thru_hull_seacock_flange.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

using namespace koocadcam;

namespace {
double volumeOf(const TopoDS_Shape& s) {
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}

skill::thru_hull_seacock_flange::Input goodInput()
{
    skill::thru_hull_seacock_flange::Input in;
    in.face_id            = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.center_xy          = gp_Pnt(50.0, 50.0, 0.0);
    in.hull_bore_dia_mm   = 38.0;
    in.flange_dia_mm      = 80.0;
    in.bolt_circle_dia_mm = 62.0;
    in.o_ring_size_key    = "-116";
    return in;
}
}  // namespace

TEST(SkillThruHullSeacockFlange, ApplyRemovesMaterial)
{
    auto stock = skill::createCuboidStock(100.0, 100.0, 25.0);
    const double v0 = volumeOf(stock->shape());
    auto in  = goodInput();
    auto out = skill::thru_hull_seacock_flange::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    const double v1 = volumeOf(out.workpiece->shape());

    EXPECT_GT(v0 - v1, 0.0);   // bore + seat + groove + 3 bolts
    EXPECT_GT(out.workpiece->faceCount(), stock->faceCount());
}

TEST(SkillThruHullSeacockFlange, ValidateRejectsUnknownORing)
{
    auto stock = skill::createCuboidStock(100.0, 100.0, 25.0);
    auto in = goodInput();
    in.o_ring_size_key = "-999";

    auto r = skill::thru_hull_seacock_flange::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-AS568") { found = true; break; }
    EXPECT_TRUE(found);

    EXPECT_THROW(skill::thru_hull_seacock_flange::apply(*stock, in),
                 skill::SkillError);
}

TEST(SkillThruHullSeacockFlange, ValidateRejectsBoltCircleTooLarge)
{
    auto stock = skill::createCuboidStock(100.0, 100.0, 25.0);
    auto in = goodInput();
    in.bolt_circle_dia_mm = 90.0;   // >= flange_dia_mm (80)

    auto r = skill::thru_hull_seacock_flange::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-MARINE-PCD") { found = true; break; }
    EXPECT_TRUE(found);
}

TEST(SkillThruHullSeacockFlange, SignatureCompoundFlange)
{
    auto stock = skill::createCuboidStock(100.0, 100.0, 25.0);
    auto in    = goodInput();
    auto out   = skill::thru_hull_seacock_flange::apply(*stock, in);

    EXPECT_EQ(out.signature.skill_id,
              std::string("thru_hull_seacock_flange"));
    EXPECT_TRUE(out.signature.pattern.value("is_compound", false));
    EXPECT_EQ(out.signature.pattern.value("marine_feature_type", std::string()),
              std::string("thru_hull_seacock_flange"));
    EXPECT_EQ(out.signature.pattern.value("subfeature_count", 0), 6);
}

TEST(SkillThruHullSeacockFlange, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(100.0, 100.0, 25.0);
    auto in    = goodInput();
    auto out   = skill::thru_hull_seacock_flange::apply(*stock, in);
    auto cands = skill::thru_hull_seacock_flange::recognize(*out.workpiece);
    ASSERT_FALSE(cands.empty());
    EXPECT_DOUBLE_EQ(cands[0].confidence, 1.0);
}
