// @lat: [[process/test-strategy#grounding_lug_layin_seat]]

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/grounding_lug_layin_seat.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

using namespace koocadcam;

namespace {
double volumeOf(const TopoDS_Shape& s) {
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}

skill::grounding_lug_layin_seat::Input goodInput()
{
    skill::grounding_lug_layin_seat::Input in;
    in.face_id                 = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.center_xy               = gp_Pnt(40.0, 40.0, 0.0);
    in.seat_len_mm             = 24.0;
    in.seat_wid_mm             = 12.0;
    in.seat_depth_mm           = 2.0;
    in.conductor_groove_dia_mm = 6.0;
    in.set_screw_thread_key    = "M6";
    return in;
}
}  // namespace

TEST(SkillGroundingLugLayinSeat, ApplyRemovesMaterial)
{
    auto stock = skill::createCuboidStock(80.0, 80.0, 20.0);
    const double v0 = volumeOf(stock->shape());
    auto in  = goodInput();
    auto out = skill::grounding_lug_layin_seat::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    const double v1 = volumeOf(out.workpiece->shape());

    EXPECT_GT(v0 - v1, 0.0);
    EXPECT_GT(out.workpiece->faceCount(), stock->faceCount());
}

TEST(SkillGroundingLugLayinSeat, ValidateRejectsUnknownThread)
{
    auto stock = skill::createCuboidStock(80.0, 80.0, 20.0);
    auto in = goodInput();
    in.set_screw_thread_key = "M99";

    auto r = skill::grounding_lug_layin_seat::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-THREAD") { found = true; break; }
    EXPECT_TRUE(found);

    EXPECT_THROW(skill::grounding_lug_layin_seat::apply(*stock, in),
                 skill::SkillError);
}

TEST(SkillGroundingLugLayinSeat, ValidateRejectsOversizeGroove)
{
    auto stock = skill::createCuboidStock(80.0, 80.0, 20.0);
    auto in = goodInput();
    in.conductor_groove_dia_mm = 20.0;   // >= seat_wid_mm

    auto r = skill::grounding_lug_layin_seat::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-GROOVE") { found = true; break; }
    EXPECT_TRUE(found);
}

TEST(SkillGroundingLugLayinSeat, SignatureCompound)
{
    auto stock = skill::createCuboidStock(80.0, 80.0, 20.0);
    auto in    = goodInput();
    auto out   = skill::grounding_lug_layin_seat::apply(*stock, in);

    EXPECT_TRUE(out.signature.pattern.value("is_compound", false));
    EXPECT_EQ(out.signature.pattern.value("solar_feature_type", std::string()),
              std::string("grounding_lug_layin_seat"));
    EXPECT_EQ(out.signature.pattern.value("subfeature_count", 0), 3);
}

TEST(SkillGroundingLugLayinSeat, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(80.0, 80.0, 20.0);
    auto in    = goodInput();
    auto out   = skill::grounding_lug_layin_seat::apply(*stock, in);
    auto cands = skill::grounding_lug_layin_seat::recognize(*out.workpiece);
    ASSERT_FALSE(cands.empty());
    EXPECT_DOUBLE_EQ(cands[0].confidence, 1.0);
}
