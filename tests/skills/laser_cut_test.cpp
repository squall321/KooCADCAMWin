// @lat: [[process/test-strategy#skill round-trip]]

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/laser_cut.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

#include <cmath>

using namespace koocadcam;

namespace {
double volumeOf(const TopoDS_Shape& s) {
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}
}  // namespace

// ── 1. Linear cut: very narrow kerf ──────────────────────────────────────
TEST(SkillLaserCut, ApplyLinearNarrowKerf)
{
    auto stock = skill::createCuboidStock(50.0, 30.0, 5.0);

    skill::laser_cut::Input in;
    in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.kind = skill::laser_cut::Input::Kind::Linear;
    in.start_x_mm = 5.0;  in.start_y_mm = 15.0;
    in.end_x_mm   = 45.0; in.end_y_mm   = 15.0;

    auto out = skill::laser_cut::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    const double kerf = 0.2;
    const double L    = 40.0;
    // kerf is very small, end-cap circles negligible → use L × kerf × t
    const double expected = L * kerf * 5.0;
    const double removed  = volumeOf(stock->shape()) - volumeOf(out.workpiece->shape());
    // OCCT booleans on very thin shells can have larger relative error.
    EXPECT_NEAR(removed, expected, expected * 0.30);

    EXPECT_EQ(out.signature.pattern["tool_type"].get<std::string>(), "laser");
    EXPECT_NEAR(out.signature.pattern["kerf_mm"].get<double>(), 0.2, 1e-6);
    EXPECT_EQ(out.signature.pattern["surface_finish"].get<std::string>(), "fine");
}

// ── 2. Circular cut: arc segment ─────────────────────────────────────────
TEST(SkillLaserCut, ApplyCircularArc)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 4.0);

    skill::laser_cut::Input in;
    in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.kind = skill::laser_cut::Input::Kind::Circular;
    in.cx_mm = 30.0; in.cy_mm = 30.0;
    in.radius_mm = 15.0;
    in.start_angle_deg = 45.0;
    in.end_angle_deg   = 135.0;

    auto out = skill::laser_cut::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    EXPECT_GT(volumeOf(stock->shape()), volumeOf(out.workpiece->shape()));
    EXPECT_EQ(out.signature.pattern["cut_kind"].get<std::string>(), "Circular");
    EXPECT_NEAR(out.signature.pattern["path_length_mm"].get<double>(),
                15.0 * M_PI / 2.0, 1e-3);
}

// ── 3. DFM-THICKNESS error above 25 mm (laser's hard limit) ──────────────
TEST(SkillLaserCut, DfmRejectsThickStock)
{
    auto stock = skill::createCuboidStock(30.0, 30.0, 30.0);

    skill::laser_cut::Input in;
    in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.kind = skill::laser_cut::Input::Kind::Linear;
    in.start_x_mm = 5.0;  in.start_y_mm = 15.0;
    in.end_x_mm   = 25.0; in.end_y_mm   = 15.0;

    auto r = skill::laser_cut::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-THICKNESS" && f.severity == "error") { found = true; break; }
    EXPECT_TRUE(found);
    EXPECT_THROW(skill::laser_cut::apply(*stock, in), skill::SkillError);
}

// ── 4. Polyline cut on a tight zig-zag ──────────────────────────────────
TEST(SkillLaserCut, ApplyTightPolyline)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 3.0);

    skill::laser_cut::Input in;
    in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.kind = skill::laser_cut::Input::Kind::Polyline;
    in.waypoints = {
        { 10.0, 10.0 },
        { 20.0, 30.0 },
        { 30.0, 10.0 },
        { 40.0, 30.0 },
    };

    auto out = skill::laser_cut::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    EXPECT_GT(out.signature.pattern["path_length_mm"].get<double>(), 60.0);
}

// ── 5. DFM-SURFACE info note: "fine" ─────────────────────────────────────
TEST(SkillLaserCut, DfmReportsFineFinish)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 5.0);
    skill::laser_cut::Input in;
    in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.kind = skill::laser_cut::Input::Kind::Linear;
    in.start_x_mm = 5.0; in.start_y_mm = 20.0;
    in.end_x_mm   = 35.0; in.end_y_mm  = 20.0;

    auto r = skill::laser_cut::validate(*stock, in);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-SURFACE" && f.severity == "info"
            && f.message.find("fine") != std::string::npos) { found = true; break; }
    EXPECT_TRUE(found);
}

// ── 6. Recognize at kerf ≈ 0.2 → laser conf 0.7 (highest) ────────────────
TEST(SkillLaserCut, RecognizeHighConfAtSmallKerf)
{
    auto stock = skill::createCuboidStock(40.0, 30.0, 4.0);

    skill::laser_cut::Input in;
    in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.kind = skill::laser_cut::Input::Kind::Linear;
    in.start_x_mm = 5.0;  in.start_y_mm = 15.0;
    in.end_x_mm   = 35.0; in.end_y_mm   = 15.0;

    auto out   = skill::laser_cut::apply(*stock, in);
    auto cands = skill::laser_cut::recognize(*out.workpiece);
    ASSERT_FALSE(cands.empty());
    EXPECT_NEAR(cands[0].confidence, 0.70, 1e-9);
    EXPECT_LT(cands[0].recovered_params["kerf_mm"].get<double>(), 0.5);
}
