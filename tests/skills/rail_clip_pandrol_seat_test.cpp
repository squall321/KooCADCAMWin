// @lat: [[process/test-strategy#rail_clip_pandrol_seat]]

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/rail_clip_pandrol_seat.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

using namespace koocadcam;

namespace {
double volumeOf(const TopoDS_Shape& s) {
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}

skill::rail_clip_pandrol_seat::Input goodInput()
{
    skill::rail_clip_pandrol_seat::Input in;
    in.face_id            = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.center_xy          = gp_Pnt(80.0, 60.0, 0.0);
    in.rail_foot_width_mm = 76.0;
    in.clip_seat_depth_mm = 10.0;
    in.bolt_thread_key    = "M20";
    return in;
}
}  // namespace

TEST(SkillRailClipPandrolSeat, ApplyRemovesMaterial)
{
    auto stock = skill::createCuboidStock(160.0, 120.0, 30.0);
    const double v0 = volumeOf(stock->shape());
    auto in  = goodInput();
    auto out = skill::rail_clip_pandrol_seat::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    const double v1 = volumeOf(out.workpiece->shape());

    EXPECT_GT(v0 - v1, 0.0);   // material removed (recess + pocket + bolt hole)
    EXPECT_GT(out.workpiece->faceCount(), stock->faceCount());
}

TEST(SkillRailClipPandrolSeat, ValidateRejectsUnknownThread)
{
    auto stock = skill::createCuboidStock(160.0, 120.0, 30.0);
    auto in = goodInput();
    in.bolt_thread_key = "M99";

    auto r = skill::rail_clip_pandrol_seat::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-THREAD") { found = true; break; }
    EXPECT_TRUE(found);

    EXPECT_THROW(skill::rail_clip_pandrol_seat::apply(*stock, in),
                 skill::SkillError);
}

TEST(SkillRailClipPandrolSeat, ValidateRejectsDepthExceedsFoot)
{
    auto stock = skill::createCuboidStock(160.0, 120.0, 30.0);
    auto in = goodInput();
    in.clip_seat_depth_mm = 120.0;   // >= rail_foot_width_mm

    auto r = skill::rail_clip_pandrol_seat::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-FOOT") { found = true; break; }
    EXPECT_TRUE(found);
}

TEST(SkillRailClipPandrolSeat, SignatureCompoundPandrol)
{
    auto stock = skill::createCuboidStock(160.0, 120.0, 30.0);
    auto in    = goodInput();
    auto out   = skill::rail_clip_pandrol_seat::apply(*stock, in);

    EXPECT_EQ(out.signature.skill_id, std::string("rail_clip_pandrol_seat"));
    EXPECT_TRUE(out.signature.pattern.value("is_compound", false));
    EXPECT_EQ(out.signature.pattern.value("railway_feature_type", std::string()),
              std::string("pandrol_eclip_rail_seat"));
    EXPECT_EQ(out.signature.pattern.value("subfeature_count", 0), 3);
}

TEST(SkillRailClipPandrolSeat, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(160.0, 120.0, 30.0);
    auto in    = goodInput();
    auto out   = skill::rail_clip_pandrol_seat::apply(*stock, in);
    auto cands = skill::rail_clip_pandrol_seat::recognize(*out.workpiece);
    ASSERT_FALSE(cands.empty());
    EXPECT_DOUBLE_EQ(cands[0].confidence, 1.0);
}
