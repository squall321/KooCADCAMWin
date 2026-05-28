// @lat: [[process/test-strategy#skill round-trip]]

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/bearing_seat.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

#include <cmath>

using namespace koocadcam;

namespace {
double volumeOf(const TopoDS_Shape& s) {
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}
}

TEST(SkillBearingSeat, ApplyChangesGeometry)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 20.0);

    skill::bearing_seat::Input in;
    in.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 25.0;
    in.position_y_mm = 25.0;
    in.outer_dia_mm  = 12.0;
    in.bore_depth_mm = 8.0;
    in.bearing_id    = "6201";
    in.fit_class     = "k6";

    const double v0 = volumeOf(stock->shape());
    auto out = skill::bearing_seat::apply(*stock, in);
    const double v1 = volumeOf(out.workpiece->shape());

    const double approx = M_PI * 6.0 * 6.0 * 8.0;
    EXPECT_NEAR(v0 - v1, approx, approx * 0.05);
}

TEST(SkillBearingSeat, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 20.0);

    skill::bearing_seat::Input in;
    in.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 25.0;
    in.position_y_mm = 25.0;
    in.outer_dia_mm  = 2.0;        // < 4 mm
    in.bore_depth_mm = 5.0;

    auto r = skill::bearing_seat::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-BS-DIA") { found = true; break; }
    EXPECT_TRUE(found);
    EXPECT_THROW(skill::bearing_seat::apply(*stock, in), skill::SkillError);
}

TEST(SkillBearingSeat, SignatureRecordsKindAndParams)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 20.0);

    skill::bearing_seat::Input in;
    in.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 25.0;
    in.position_y_mm = 25.0;
    in.outer_dia_mm  = 14.0;
    in.bore_depth_mm = 9.0;
    in.bearing_id    = "6204";
    in.fit_class     = "m6";
    in.preload_N     = 50.0;

    auto out = skill::bearing_seat::apply(*stock, in);

    EXPECT_EQ(out.signature.skill_id, std::string("bearing_seat"));
    EXPECT_EQ(out.signature.pattern["kind"], "bearing_seat");
    EXPECT_NEAR(out.signature.params["outer_dia_mm"].get<double>(), 14.0, 1e-9);
    EXPECT_NEAR(out.signature.params["bore_depth_mm"].get<double>(), 9.0, 1e-9);
    EXPECT_EQ(out.signature.params["bearing_id"], "6204");
    EXPECT_EQ(out.signature.params["fit_class"],  "m6");
    EXPECT_NEAR(out.signature.params["preload_N"].get<double>(), 50.0, 1e-9);
}

TEST(SkillBearingSeat, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 20.0);

    skill::bearing_seat::Input in;
    in.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 25.0;
    in.position_y_mm = 25.0;
    in.outer_dia_mm  = 12.0;
    in.bore_depth_mm = 7.0;
    in.bearing_id    = "6202";
    in.fit_class     = "k6";

    auto out = skill::bearing_seat::apply(*stock, in);
    auto cands = skill::bearing_seat::recognize(*out.workpiece);

    ASSERT_EQ(cands.size(), 1u);
    EXPECT_EQ(cands[0].skill_id, std::string("bearing_seat"));
    EXPECT_EQ(cands[0].recovered_params["bearing_id"], "6202");
    EXPECT_EQ(cands[0].recovered_params["fit_class"],  "k6");
    EXPECT_NEAR(cands[0].recovered_params["outer_dia_mm"].get<double>(),
                12.0, 1e-9);
}

// ─── 5. Specific: tooling tool_type is boring_bar, extra carries fit_class ─
TEST(SkillBearingSeat, ToolingIsBoringBarAndExtraCarriesFit)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 20.0);

    skill::bearing_seat::Input in;
    in.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 25.0;
    in.position_y_mm = 25.0;
    in.outer_dia_mm  = 16.0;
    in.bore_depth_mm = 6.0;
    in.bearing_id    = "6203";
    in.fit_class     = "p6";
    in.preload_N     = 100.0;

    auto out = skill::bearing_seat::apply(*stock, in);
    EXPECT_EQ(out.signature.tooling.tool_type, std::string("boring_bar"));
    EXPECT_EQ(out.signature.tooling.extra["fit_class"], "p6");
    EXPECT_NEAR(out.signature.tooling.extra["preload_N"].get<double>(),
                100.0, 1e-9);
}
