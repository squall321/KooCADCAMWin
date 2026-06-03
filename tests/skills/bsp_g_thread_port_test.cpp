// @lat: [[process/test-strategy#hydraulic ports — bsp_g_thread_port]]
//
// Tests:
//   1. apply removes real volume; port_dia (drill) matches central table.
//   2. validate rejects unknown thread_size.
//   3. validate rejects insufficient material.
//   4. signature compound + standard.
//   5. recognize metadata replay at confidence 1.0.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/_hydraulic_ports.hpp"
#include "skills/bsp_g_thread_port.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

using namespace koocadcam;

namespace {

double volumeOf(const TopoDS_Shape& s) {
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}

skill::bsp_g_thread_port::Input goodInput()
{
    skill::bsp_g_thread_port::Input in;
    in.face_id     = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.center_x_mm = 50.0;
    in.center_y_mm = 50.0;
    in.axis_dir    = gp_Dir(0, 0, -1);
    in.thread_size = "G1/4";   // drill_dia = 11.80 mm
    return in;
}

}  // namespace

// ─── 1. apply + drill_dia matches table within 0.05 mm ────────────────────
TEST(SkillBspGThreadPort, ApplyAndDrillDiaMatchesTable)
{
    auto stock = skill::createCuboidStock(100.0, 100.0, 30.0);
    const double v0 = volumeOf(stock->shape());
    auto out = skill::bsp_g_thread_port::apply(*stock, goodInput());
    EXPECT_GT(v0 - volumeOf(out.workpiece->shape()), 0.0);

    const auto* spec = skill::hyd_ports::findBspG("G1/4");
    ASSERT_NE(spec, nullptr);
    EXPECT_NEAR(out.signature.pattern.at("drill_dia_mm").get<double>(),
                spec->drill_dia_mm, 0.05);
    EXPECT_NEAR(out.signature.pattern.at("port_dia_mm").get<double>(),
                spec->drill_dia_mm, 0.05);
    EXPECT_NEAR(out.signature.pattern.at("thread_major_mm").get<double>(),
                spec->thread_major_mm, 0.05);
}

// ─── 2. validate rejects unknown thread_size ──────────────────────────────
TEST(SkillBspGThreadPort, ValidateRejectsUnknownThreadSize)
{
    auto stock = skill::createCuboidStock(100.0, 100.0, 30.0);
    auto in = goodInput();
    in.thread_size = "G2";   // not in table
    auto r = skill::bsp_g_thread_port::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-BSPG-CODE") { found = true; break; }
    EXPECT_TRUE(found);
    EXPECT_THROW(skill::bsp_g_thread_port::apply(*stock, in), skill::SkillError);
}

// ─── 3. validate rejects too-small face ───────────────────────────────────
TEST(SkillBspGThreadPort, ValidateRejectsInsufficientMaterial)
{
    // G1/4 spotface = 20 mm → need ≥ 30 mm face span; 25 mm fails.
    auto tiny = skill::createCuboidStock(25.0, 25.0, 30.0);
    auto in = goodInput();
    in.center_x_mm = 12.5;
    in.center_y_mm = 12.5;
    auto r = skill::bsp_g_thread_port::validate(*tiny, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-BSPG-MAT") { found = true; break; }
    EXPECT_TRUE(found);
}

// ─── 4. signature compound + standard ─────────────────────────────────────
TEST(SkillBspGThreadPort, SignatureCompoundAndStandard)
{
    auto stock = skill::createCuboidStock(100.0, 100.0, 30.0);
    auto out   = skill::bsp_g_thread_port::apply(*stock, goodInput());

    EXPECT_EQ(out.signature.skill_id, std::string("bsp_g_thread_port"));
    EXPECT_TRUE(out.signature.pattern.at("is_compound").get<bool>());
    EXPECT_EQ(out.signature.pattern.at("port_standard").get<std::string>(),
              std::string("BSPP_ISO_228"));
    EXPECT_EQ(out.signature.pattern.at("thread_size").get<std::string>(),
              std::string("G1/4"));
    EXPECT_GT(out.signature.pattern.at("derived_volume_removed").get<double>(), 0.0);
}

// ─── 5. recognize metadata replay ─────────────────────────────────────────
TEST(SkillBspGThreadPort, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(100.0, 100.0, 30.0);
    auto out   = skill::bsp_g_thread_port::apply(*stock, goodInput());

    auto cands = skill::bsp_g_thread_port::recognize(*out.workpiece);
    ASSERT_GE(cands.size(), 1u);
    EXPECT_NEAR(cands.front().confidence, 1.0, 1e-9);
    EXPECT_EQ(cands.front().recovered_params["thread_size"].get<std::string>(),
              std::string("G1/4"));
}
