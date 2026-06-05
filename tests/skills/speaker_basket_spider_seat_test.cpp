// @lat: [[process/test-strategy#speaker_basket_spider_seat]]

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/speaker_basket_spider_seat.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

using namespace koocadcam;

namespace {
double volumeOf(const TopoDS_Shape& s) {
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}

skill::speaker_basket_spider_seat::Input goodInput()
{
    skill::speaker_basket_spider_seat::Input in;
    in.face_id            = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.center_xy          = gp_Pnt(40.0, 40.0, 0.0);
    in.spider_ledge_dia_mm = 44.0;
    in.ledge_depth_mm      = 3.0;
    in.vc_clearance_dia_mm = 26.0;
    in.vent_count          = 6;
    in.vent_dia_mm         = 3.0;
    in.vent_circle_dia_mm  = 36.0;
    return in;
}
}  // namespace

TEST(SkillSpeakerBasketSpiderSeat, ApplyRemovesMaterial)
{
    auto stock = skill::createCuboidStock(80.0, 80.0, 20.0);
    const double v0 = volumeOf(stock->shape());
    auto in  = goodInput();
    auto out = skill::speaker_basket_spider_seat::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    const double v1 = volumeOf(out.workpiece->shape());

    EXPECT_GT(v0 - v1, 0.0);   // ledge + VC bore + vents removed
    EXPECT_GT(out.workpiece->faceCount(), stock->faceCount());
}

TEST(SkillSpeakerBasketSpiderSeat, ValidateRejectsVcLargerThanLedge)
{
    auto stock = skill::createCuboidStock(80.0, 80.0, 20.0);
    auto in = goodInput();
    in.vc_clearance_dia_mm = 50.0;   // larger than 44 ledge

    auto r = skill::speaker_basket_spider_seat::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-VC-BORE") { found = true; break; }
    EXPECT_TRUE(found);

    EXPECT_THROW(skill::speaker_basket_spider_seat::apply(*stock, in),
                 skill::SkillError);
}

TEST(SkillSpeakerBasketSpiderSeat, ValidateRejectsBadVentFit)
{
    auto stock = skill::createCuboidStock(80.0, 80.0, 20.0);
    auto in = goodInput();
    in.vent_circle_dia_mm = 28.0;   // vents collide with the VC bore wall

    auto r = skill::speaker_basket_spider_seat::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-VENT-FIT") { found = true; break; }
    EXPECT_TRUE(found);
}

TEST(SkillSpeakerBasketSpiderSeat, SignatureCompoundSpiderSeat)
{
    auto stock = skill::createCuboidStock(80.0, 80.0, 20.0);
    auto in    = goodInput();
    auto out   = skill::speaker_basket_spider_seat::apply(*stock, in);

    EXPECT_TRUE(out.signature.pattern.value("is_compound", false));
    EXPECT_EQ(out.signature.pattern.value("audio_feature_type", std::string()),
              std::string("spider_seat_vented"));
    EXPECT_EQ(out.signature.pattern.value("subfeature_count", 0),
              2 + in.vent_count);
}

TEST(SkillSpeakerBasketSpiderSeat, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(80.0, 80.0, 20.0);
    auto in    = goodInput();
    auto out   = skill::speaker_basket_spider_seat::apply(*stock, in);
    auto cands = skill::speaker_basket_spider_seat::recognize(*out.workpiece);
    ASSERT_FALSE(cands.empty());
    EXPECT_DOUBLE_EQ(cands[0].confidence, 1.0);
}
