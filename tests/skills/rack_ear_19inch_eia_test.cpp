// @lat: [[process/test-strategy#rack_ear_19inch_eia]]

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/rack_ear_19inch_eia.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

using namespace koocadcam;

namespace {
double volumeOf(const TopoDS_Shape& s) {
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}

skill::rack_ear_19inch_eia::Input goodInput()
{
    skill::rack_ear_19inch_eia::Input in;
    in.face_id            = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.center_xy          = gp_Pnt(20.0, 50.0, 0.0);
    in.hole_dia_mm        = 6.6;
    in.hole_thread_key    = "M6";
    in.handle_slot_len_mm = 40.0;
    in.handle_slot_wid_mm = 10.0;
    return in;
}
}  // namespace

TEST(SkillRackEar19inchEia, ApplyRemovesMaterial)
{
    auto stock = skill::createCuboidStock(44.45, 110.0, 16.0);
    const double v0 = volumeOf(stock->shape());
    auto in  = goodInput();
    auto out = skill::rack_ear_19inch_eia::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    const double v1 = volumeOf(out.workpiece->shape());

    EXPECT_GT(v0 - v1, 0.0);   // 3 EIA holes + handle slot removed
    EXPECT_GT(out.workpiece->faceCount(), stock->faceCount());
}

TEST(SkillRackEar19inchEia, ValidateRejectsUnknownThread)
{
    auto stock = skill::createCuboidStock(44.45, 110.0, 16.0);
    auto in = goodInput();
    in.hole_thread_key = "M999";

    auto r = skill::rack_ear_19inch_eia::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-THREAD") { found = true; break; }
    EXPECT_TRUE(found);

    EXPECT_THROW(skill::rack_ear_19inch_eia::apply(*stock, in),
                 skill::SkillError);
}

TEST(SkillRackEar19inchEia, ValidateRejectsDegenerateSlot)
{
    auto stock = skill::createCuboidStock(44.45, 110.0, 16.0);
    auto in = goodInput();
    in.handle_slot_len_mm = 8.0;
    in.handle_slot_wid_mm = 10.0;   // wider than long

    auto r = skill::rack_ear_19inch_eia::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-SLOT") { found = true; break; }
    EXPECT_TRUE(found);
}

TEST(SkillRackEar19inchEia, SignatureCompoundRackEar)
{
    auto stock = skill::createCuboidStock(44.45, 110.0, 16.0);
    auto in    = goodInput();
    auto out   = skill::rack_ear_19inch_eia::apply(*stock, in);

    EXPECT_TRUE(out.signature.pattern.value("is_compound", false));
    EXPECT_EQ(out.signature.pattern.value("audio_feature_type", std::string()),
              std::string("rack_ear_eia310"));
    EXPECT_EQ(out.signature.pattern.value("subfeature_count", 0), 4);
}

TEST(SkillRackEar19inchEia, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(44.45, 110.0, 16.0);
    auto in    = goodInput();
    auto out   = skill::rack_ear_19inch_eia::apply(*stock, in);
    auto cands = skill::rack_ear_19inch_eia::recognize(*out.workpiece);
    ASSERT_FALSE(cands.empty());
    EXPECT_DOUBLE_EQ(cands[0].confidence, 1.0);
}
