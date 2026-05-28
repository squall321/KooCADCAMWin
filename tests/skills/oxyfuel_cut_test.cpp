// @lat: [[process/test-strategy#skill round-trip]]

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/oxyfuel_cut.hpp"

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

// ── 1. Linear cut on thick steel plate: very wide kerf ───────────────────
TEST(SkillOxyfuelCut, ApplyLinearThickPlate)
{
    auto stock = skill::createCuboidStock(120.0, 60.0, 50.0);

    skill::oxyfuel_cut::Input in;
    in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.kind = skill::oxyfuel_cut::Input::Kind::Linear;
    in.start_x_mm = 20.0; in.start_y_mm = 30.0;
    in.end_x_mm   = 100.0; in.end_y_mm  = 30.0;

    auto out = skill::oxyfuel_cut::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    const double kerf = 6.5;
    const double r    = kerf / 2.0;
    const double L    = 80.0;
    const double expected = (M_PI * r * r + L * kerf) * 50.0;
    const double removed  = volumeOf(stock->shape()) - volumeOf(out.workpiece->shape());
    EXPECT_NEAR(removed, expected, expected * 0.15);

    EXPECT_EQ(out.signature.pattern["tool_type"].get<std::string>(), "oxyfuel");
    EXPECT_NEAR(out.signature.pattern["kerf_mm"].get<double>(), 6.5, 1e-6);
}

// ── 2. Circular sector on thick plate ────────────────────────────────────
TEST(SkillOxyfuelCut, ApplyCircularSectorThick)
{
    auto stock = skill::createCuboidStock(150.0, 150.0, 40.0);

    skill::oxyfuel_cut::Input in;
    in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.kind = skill::oxyfuel_cut::Input::Kind::Circular;
    in.cx_mm = 75.0; in.cy_mm = 75.0;
    in.radius_mm = 40.0;
    in.start_angle_deg = 0.0;
    in.end_angle_deg   = 270.0;

    auto out = skill::oxyfuel_cut::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    EXPECT_GT(volumeOf(stock->shape()), volumeOf(out.workpiece->shape()));
    EXPECT_EQ(out.signature.pattern["cut_kind"].get<std::string>(), "Circular");
}

// ── 3. DFM-THICKNESS error above 300 mm ─────────────────────────────────
TEST(SkillOxyfuelCut, DfmRejectsExtremeThickStock)
{
    auto stock = skill::createCuboidStock(80.0, 80.0, 400.0);

    skill::oxyfuel_cut::Input in;
    in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.kind = skill::oxyfuel_cut::Input::Kind::Linear;
    in.start_x_mm = 10.0; in.start_y_mm = 40.0;
    in.end_x_mm   = 70.0; in.end_y_mm   = 40.0;

    auto r = skill::oxyfuel_cut::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_THROW(skill::oxyfuel_cut::apply(*stock, in), skill::SkillError);
}

// ── 4. DFM-SURFACE rough_oxidized + dross info ──────────────────────────
TEST(SkillOxyfuelCut, DfmReportsOxidizedFinish)
{
    auto stock = skill::createCuboidStock(60.0, 40.0, 30.0);

    skill::oxyfuel_cut::Input in;
    in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.kind = skill::oxyfuel_cut::Input::Kind::Linear;
    in.start_x_mm = 5.0;  in.start_y_mm = 20.0;
    in.end_x_mm   = 55.0; in.end_y_mm   = 20.0;

    auto r = skill::oxyfuel_cut::validate(*stock, in);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-SURFACE" && f.severity == "info"
            && f.message.find("rough_oxidized") != std::string::npos) {
            found = true; break;
        }
    EXPECT_TRUE(found);
}

// ── 5. Polyline cut over a structural shape ─────────────────────────────
TEST(SkillOxyfuelCut, ApplyPolylineStructural)
{
    auto stock = skill::createCuboidStock(200.0, 80.0, 25.0);

    skill::oxyfuel_cut::Input in;
    in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.kind = skill::oxyfuel_cut::Input::Kind::Polyline;
    in.waypoints = {
        { 20.0, 20.0 },
        { 100.0, 20.0 },
        { 100.0, 60.0 },
        { 180.0, 60.0 },
    };

    auto out = skill::oxyfuel_cut::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    EXPECT_GT(out.signature.pattern["path_length_mm"].get<double>(), 160.0);
}

// ── 6. Recognize at kerf 6.5 → oxyfuel conf 0.7 ─────────────────────────
TEST(SkillOxyfuelCut, RecognizeHighConfAtLargeKerf)
{
    auto stock = skill::createCuboidStock(80.0, 60.0, 30.0);

    skill::oxyfuel_cut::Input in;
    in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.kind = skill::oxyfuel_cut::Input::Kind::Linear;
    in.start_x_mm = 10.0; in.start_y_mm = 30.0;
    in.end_x_mm   = 70.0; in.end_y_mm   = 30.0;

    auto out   = skill::oxyfuel_cut::apply(*stock, in);
    auto cands = skill::oxyfuel_cut::recognize(*out.workpiece);
    ASSERT_FALSE(cands.empty());
    EXPECT_NEAR(cands[0].confidence, 0.70, 1e-9);
    EXPECT_GT(cands[0].recovered_params["kerf_mm"].get<double>(), 6.0);
}
