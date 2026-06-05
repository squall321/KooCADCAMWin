// @lat: [[process/test-strategy#winch_drum_base_bolt_circle]]

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/winch_drum_base_bolt_circle.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

using namespace koocadcam;

namespace {
double volumeOf(const TopoDS_Shape& s) {
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}

skill::winch_drum_base_bolt_circle::Input goodInput()
{
    skill::winch_drum_base_bolt_circle::Input in;
    in.face_id                 = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.center_xy               = gp_Pnt(60.0, 60.0, 0.0);
    in.base_bolt_circle_dia_mm = 90.0;
    in.bolt_count              = 4;
    in.bolt_dia_mm             = 8.0;
    in.shaft_bore_dia_mm       = 25.0;
    in.pawl_pocket_len_mm      = 18.0;
    in.pawl_pocket_wid_mm      = 10.0;
    in.pawl_pocket_depth_mm    = 6.0;
    return in;
}
}  // namespace

TEST(SkillWinchDrumBaseBoltCircle, ApplyRemovesMaterial)
{
    auto stock = skill::createCuboidStock(120.0, 120.0, 20.0);
    const double v0 = volumeOf(stock->shape());
    auto in  = goodInput();
    auto out = skill::winch_drum_base_bolt_circle::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    const double v1 = volumeOf(out.workpiece->shape());

    EXPECT_GT(v0 - v1, 0.0);   // bolts + shaft bore + pawl pocket
    EXPECT_GT(out.workpiece->faceCount(), stock->faceCount());
}

TEST(SkillWinchDrumBaseBoltCircle, ValidateRejectsBoltCountOutOfRange)
{
    auto stock = skill::createCuboidStock(120.0, 120.0, 20.0);
    auto in = goodInput();
    in.bolt_count = 2;   // < 3

    auto r = skill::winch_drum_base_bolt_circle::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-MARINE-COUNT") { found = true; break; }
    EXPECT_TRUE(found);

    EXPECT_THROW(skill::winch_drum_base_bolt_circle::apply(*stock, in),
                 skill::SkillError);
}

TEST(SkillWinchDrumBaseBoltCircle, ValidateRejectsShaftLargerThanBoltCircle)
{
    auto stock = skill::createCuboidStock(120.0, 120.0, 20.0);
    auto in = goodInput();
    in.shaft_bore_dia_mm = 100.0;   // >= base_bolt_circle_dia_mm (90)

    auto r = skill::winch_drum_base_bolt_circle::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-MARINE-SHAFT") { found = true; break; }
    EXPECT_TRUE(found);
}

TEST(SkillWinchDrumBaseBoltCircle, SignatureCompoundWinchBase)
{
    auto stock = skill::createCuboidStock(120.0, 120.0, 20.0);
    auto in    = goodInput();
    auto out   = skill::winch_drum_base_bolt_circle::apply(*stock, in);

    EXPECT_EQ(out.signature.skill_id,
              std::string("winch_drum_base_bolt_circle"));
    EXPECT_TRUE(out.signature.pattern.value("is_compound", false));
    EXPECT_EQ(out.signature.pattern.value("marine_feature_type", std::string()),
              std::string("winch_drum_base"));
    EXPECT_EQ(out.signature.pattern.value("subfeature_count", 0),
              in.bolt_count + 2);
}

TEST(SkillWinchDrumBaseBoltCircle, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(120.0, 120.0, 20.0);
    auto in    = goodInput();
    auto out   = skill::winch_drum_base_bolt_circle::apply(*stock, in);
    auto cands = skill::winch_drum_base_bolt_circle::recognize(*out.workpiece);
    ASSERT_FALSE(cands.empty());
    EXPECT_DOUBLE_EQ(cands[0].confidence, 1.0);
}
