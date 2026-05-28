// @lat: [[process/test-strategy#skill round-trip]]

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/plasma_cut.hpp"

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

// ── 1. Linear cut: wide oxidised kerf ────────────────────────────────────
TEST(SkillPlasmaCut, ApplyLinearWideKerf)
{
    auto stock = skill::createCuboidStock(80.0, 40.0, 15.0);

    skill::plasma_cut::Input in;
    in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.kind = skill::plasma_cut::Input::Kind::Linear;
    in.start_x_mm = 10.0; in.start_y_mm = 20.0;
    in.end_x_mm   = 70.0; in.end_y_mm   = 20.0;

    auto out = skill::plasma_cut::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    const double kerf = 3.5;
    const double r    = kerf / 2.0;
    const double L    = 60.0;
    const double expected = (M_PI * r * r + L * kerf) * 15.0;
    const double removed  = volumeOf(stock->shape()) - volumeOf(out.workpiece->shape());
    EXPECT_NEAR(removed, expected, expected * 0.15);

    EXPECT_EQ(out.signature.pattern["tool_type"].get<std::string>(), "plasma");
    EXPECT_NEAR(out.signature.pattern["kerf_mm"].get<double>(), 3.5, 1e-6);
    EXPECT_EQ(out.signature.pattern["surface_finish"].get<std::string>(),
              "rough_oxidized");
}

// ── 2. Circular sector cut ──────────────────────────────────────────────
TEST(SkillPlasmaCut, ApplyCircularSector)
{
    auto stock = skill::createCuboidStock(80.0, 80.0, 10.0);

    skill::plasma_cut::Input in;
    in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.kind = skill::plasma_cut::Input::Kind::Circular;
    in.cx_mm = 40.0; in.cy_mm = 40.0;
    in.radius_mm = 25.0;
    in.start_angle_deg = 0.0;
    in.end_angle_deg   = 180.0;

    auto out = skill::plasma_cut::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    EXPECT_GT(volumeOf(stock->shape()), volumeOf(out.workpiece->shape()));
    EXPECT_EQ(out.signature.pattern["cut_kind"].get<std::string>(), "Circular");
}

// ── 3. DFM-THICKNESS error above 50 mm ──────────────────────────────────
TEST(SkillPlasmaCut, DfmRejectsThickStock)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 80.0);

    skill::plasma_cut::Input in;
    in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.kind = skill::plasma_cut::Input::Kind::Linear;
    in.start_x_mm = 5.0;  in.start_y_mm = 20.0;
    in.end_x_mm   = 35.0; in.end_y_mm   = 20.0;

    auto r = skill::plasma_cut::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_THROW(skill::plasma_cut::apply(*stock, in), skill::SkillError);
}

// ── 4. Polyline cut with sharp turns ────────────────────────────────────
TEST(SkillPlasmaCut, ApplyPolylineSharpCorners)
{
    auto stock = skill::createCuboidStock(100.0, 100.0, 10.0);

    skill::plasma_cut::Input in;
    in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.kind = skill::plasma_cut::Input::Kind::Polyline;
    in.waypoints = {
        { 20.0, 50.0 },
        { 40.0, 80.0 },
        { 60.0, 50.0 },
        { 80.0, 80.0 },
    };

    auto out = skill::plasma_cut::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    EXPECT_EQ(out.signature.pattern["cut_kind"].get<std::string>(), "Polyline");
}

// ── 5. DFM-SURFACE rough_oxidized info ──────────────────────────────────
TEST(SkillPlasmaCut, DfmReportsRoughOxidizedFinish)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::plasma_cut::Input in;
    in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.kind = skill::plasma_cut::Input::Kind::Linear;
    in.start_x_mm = 5.0; in.start_y_mm = 20.0;
    in.end_x_mm   = 35.0; in.end_y_mm  = 20.0;

    auto r = skill::plasma_cut::validate(*stock, in);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-SURFACE" && f.severity == "info"
            && f.message.find("rough_oxidized") != std::string::npos) {
            found = true; break;
        }
    EXPECT_TRUE(found);
}

// ── 6. Recognize at kerf ≈ 3.5 → plasma conf 0.6 ────────────────────────
TEST(SkillPlasmaCut, RecognizeAtMidKerf)
{
    auto stock = skill::createCuboidStock(80.0, 40.0, 10.0);

    skill::plasma_cut::Input in;
    in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.kind = skill::plasma_cut::Input::Kind::Linear;
    in.start_x_mm = 10.0; in.start_y_mm = 20.0;
    in.end_x_mm   = 70.0; in.end_y_mm   = 20.0;

    auto out   = skill::plasma_cut::apply(*stock, in);
    auto cands = skill::plasma_cut::recognize(*out.workpiece);
    ASSERT_FALSE(cands.empty());
    EXPECT_NEAR(cands[0].confidence, 0.60, 1e-9);
}
