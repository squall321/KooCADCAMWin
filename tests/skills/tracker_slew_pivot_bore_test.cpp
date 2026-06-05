// @lat: [[process/test-strategy#tracker_slew_pivot_bore]]

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/tracker_slew_pivot_bore.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

using namespace koocadcam;

namespace {
double volumeOf(const TopoDS_Shape& s) {
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}

skill::tracker_slew_pivot_bore::Input goodInput()
{
    skill::tracker_slew_pivot_bore::Input in;
    in.face_id           = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.axis_origin       = gp_Pnt(50.0, 50.0, 0.0);
    in.pivot_bore_dia_mm = 24.0;
    in.bushing_od_mm     = 30.0;
    in.bore_depth_mm     = 18.0;
    in.grease_thread_key = "M8";
    in.stop_pin_dia_mm   = 8.0;
    return in;
}
}  // namespace

TEST(SkillTrackerSlewPivotBore, ApplyRemovesMaterial)
{
    auto stock = skill::createCuboidStock(100.0, 100.0, 30.0);
    const double v0 = volumeOf(stock->shape());
    auto in  = goodInput();
    auto out = skill::tracker_slew_pivot_bore::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    const double v1 = volumeOf(out.workpiece->shape());

    EXPECT_GT(v0 - v1, 0.0);
    EXPECT_GT(out.workpiece->faceCount(), stock->faceCount());
}

TEST(SkillTrackerSlewPivotBore, ValidateRejectsUnknownThread)
{
    auto stock = skill::createCuboidStock(100.0, 100.0, 30.0);
    auto in = goodInput();
    in.grease_thread_key = "M99";

    auto r = skill::tracker_slew_pivot_bore::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-THREAD") { found = true; break; }
    EXPECT_TRUE(found);

    EXPECT_THROW(skill::tracker_slew_pivot_bore::apply(*stock, in),
                 skill::SkillError);
}

TEST(SkillTrackerSlewPivotBore, ValidateRejectsBadSeat)
{
    auto stock = skill::createCuboidStock(100.0, 100.0, 30.0);
    auto in = goodInput();
    in.bushing_od_mm = 20.0;   // <= pivot_bore_dia_mm

    auto r = skill::tracker_slew_pivot_bore::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-SEAT") { found = true; break; }
    EXPECT_TRUE(found);
}

TEST(SkillTrackerSlewPivotBore, SignatureCompound)
{
    auto stock = skill::createCuboidStock(100.0, 100.0, 30.0);
    auto in    = goodInput();
    auto out   = skill::tracker_slew_pivot_bore::apply(*stock, in);

    EXPECT_TRUE(out.signature.pattern.value("is_compound", false));
    EXPECT_EQ(out.signature.pattern.value("solar_feature_type", std::string()),
              std::string("tracker_slew_pivot_bore"));
    EXPECT_EQ(out.signature.pattern.value("subfeature_count", 0), 3);
}

TEST(SkillTrackerSlewPivotBore, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(100.0, 100.0, 30.0);
    auto in    = goodInput();
    auto out   = skill::tracker_slew_pivot_bore::apply(*stock, in);
    auto cands = skill::tracker_slew_pivot_bore::recognize(*out.workpiece);
    ASSERT_FALSE(cands.empty());
    EXPECT_DOUBLE_EQ(cands[0].confidence, 1.0);
}
