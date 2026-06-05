// @lat: [[process/test-strategy#skill round-trip]]
//
// bilge_drain_threaded_compound (slice 16) — REAL geometric verification.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/bilge_drain_threaded_compound.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using namespace koocadcam;

namespace {

double volumeOf(const TopoDS_Shape& s)
{
    GProp_GProps p;
    BRepGProp::VolumeProperties(s, p);
    return p.Mass();
}

bool hasFinding(const skill::DFMReport& r, const std::string& code)
{
    for (const auto& f : r.findings) if (f.code == code) return true;
    return false;
}

skill::bilge_drain_threaded_compound::Input goodInput()
{
    skill::bilge_drain_threaded_compound::Input in;
    in.entry_face         = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.center_x_mm        = 60.0;
    in.center_y_mm        = 60.0;
    in.axis_dir           = gp_Dir(0, 0, 1);
    in.thread_size_key    = "G1/2";
    in.spotface_dia_mm    = 30.0;
    in.spotface_depth_mm  = 2.0;
    in.drain_dia_mm       = 14.0;
    in.plate_thickness_mm = 12.0;
    return in;
}

}  // namespace

TEST(SkillBilgeDrainThreaded, ApplyRemovesExpectedVolume)
{
    auto stock = skill::createCuboidStock(120.0, 120.0, 12.0);
    ASSERT_FALSE(stock->shape().IsNull());

    const double volBefore = volumeOf(stock->shape());
    const int facesBefore  = stock->faceCount();

    auto in  = goodInput();
    auto out = skill::bilge_drain_threaded_compound::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    const double volAfter = volumeOf(out.workpiece->shape());
    const double removed  = volBefore - volAfter;

    const double rDrain = in.drain_dia_mm / 2.0;
    const double vBoreMin = M_PI * rDrain * rDrain * in.plate_thickness_mm * 0.9;
    EXPECT_GT(removed, vBoreMin);
    EXPECT_GT(out.workpiece->faceCount(), facesBefore);
}

TEST(SkillBilgeDrainThreaded, ValidateRejectsUnknownBspKey)
{
    auto stock = skill::createCuboidStock(120.0, 120.0, 12.0);

    auto in = goodInput();
    in.thread_size_key = "G42";  // not in central BspG table

    auto r = skill::bilge_drain_threaded_compound::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-BSP"));
    EXPECT_THROW(skill::bilge_drain_threaded_compound::apply(*stock, in),
                 skill::SkillError);
}

TEST(SkillBilgeDrainThreaded, SignatureIsCompound)
{
    auto stock = skill::createCuboidStock(120.0, 120.0, 12.0);

    auto in  = goodInput();
    auto out = skill::bilge_drain_threaded_compound::apply(*stock, in);

    EXPECT_EQ(out.signature.skill_id,
              std::string("bilge_drain_threaded_compound"));
    EXPECT_TRUE(out.signature.pattern.value("is_compound", false));
    EXPECT_EQ(out.signature.pattern.value("marine_feature_type", std::string{}),
              std::string("bilge_drain_threaded"));
    EXPECT_EQ(out.signature.pattern.value("subfeature_count", 0), 3);
    EXPECT_EQ(out.signature.pattern.value("thread_size", std::string{}),
              std::string("G1/2"));
}

TEST(SkillBilgeDrainThreaded, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(120.0, 120.0, 12.0);

    auto in  = goodInput();
    auto out = skill::bilge_drain_threaded_compound::apply(*stock, in);
    auto cands = skill::bilge_drain_threaded_compound::recognize(*out.workpiece);

    ASSERT_FALSE(cands.empty());
    bool ok = false;
    for (const auto& c : cands) {
        if (c.confidence >= 0.8) { ok = true; break; }
    }
    EXPECT_TRUE(ok);
}

TEST(SkillBilgeDrainThreaded, ValidateRejectsDrainTooLargeForThread)
{
    auto stock = skill::createCuboidStock(120.0, 120.0, 12.0);

    auto in = goodInput();
    in.drain_dia_mm = 25.0;  // > BSPP G1/2 minor (18.6 mm)

    auto r = skill::bilge_drain_threaded_compound::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-BSP"));
}
