// @lat: [[process/test-strategy#hydraulic ports — npt_taper_port_b120]]
//
// Tests:
//   1. apply removes real volume; drill_dia matches central table.
//   2. validate rejects unknown thread_size.
//   3. validate rejects insufficient material.
//   4. signature carries port_standard + taper_per_side_deg.
//   5. recognize metadata replay at confidence 1.0.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/_hydraulic_ports.hpp"
#include "skills/npt_taper_port_b120.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

using namespace koocadcam;

namespace {

double volumeOf(const TopoDS_Shape& s) {
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}

skill::npt_taper_port_b120::Input goodInput()
{
    skill::npt_taper_port_b120::Input in;
    in.face_id     = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.center_x_mm = 50.0;
    in.center_y_mm = 50.0;
    in.axis_dir    = gp_Dir(0, 0, -1);
    in.thread_size = "1/4";   // drill = 11.10 mm, depth = 9.6 mm
    return in;
}

}  // namespace

// ─── 1. apply + drill_dia table match (≤ 0.05 mm) ─────────────────────────
TEST(SkillNptTaperPortB120, ApplyAndDrillDiaMatchesTable)
{
    auto stock = skill::createCuboidStock(80.0, 80.0, 25.0);
    const double v0 = volumeOf(stock->shape());
    auto out = skill::npt_taper_port_b120::apply(*stock, goodInput());
    EXPECT_GT(v0 - volumeOf(out.workpiece->shape()), 0.0);

    const auto* spec = skill::hyd_ports::findNpt("1/4");
    ASSERT_NE(spec, nullptr);
    EXPECT_NEAR(out.signature.pattern.at("drill_dia_mm").get<double>(),
                spec->drill_dia_mm, 0.05);
    EXPECT_NEAR(out.signature.pattern.at("port_dia_mm").get<double>(),
                spec->drill_dia_mm, 0.05);
    EXPECT_NEAR(out.signature.pattern.at("taper_per_side_deg").get<double>(),
                1.7899, 1e-3);
}

// ─── 2. validate rejects unknown thread_size ──────────────────────────────
TEST(SkillNptTaperPortB120, ValidateRejectsUnknownThreadSize)
{
    auto stock = skill::createCuboidStock(80.0, 80.0, 25.0);
    auto in = goodInput();
    in.thread_size = "2";
    auto r = skill::npt_taper_port_b120::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-NPT-CODE") { found = true; break; }
    EXPECT_TRUE(found);
    EXPECT_THROW(skill::npt_taper_port_b120::apply(*stock, in), skill::SkillError);
}

// ─── 3. validate rejects too-small face ───────────────────────────────────
TEST(SkillNptTaperPortB120, ValidateRejectsInsufficientMaterial)
{
    // 1/4 pitch_dia E1 ≈ 13.23 mm → need ≥ 19.84 mm; 15 mm fails.
    auto tiny = skill::createCuboidStock(15.0, 15.0, 25.0);
    auto in = goodInput();
    in.center_x_mm = 7.5;
    in.center_y_mm = 7.5;
    auto r = skill::npt_taper_port_b120::validate(*tiny, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-NPT-MAT") { found = true; break; }
    EXPECT_TRUE(found);
}

// ─── 4. signature compound + standard ─────────────────────────────────────
TEST(SkillNptTaperPortB120, SignatureCompoundAndStandard)
{
    auto stock = skill::createCuboidStock(80.0, 80.0, 25.0);
    auto out   = skill::npt_taper_port_b120::apply(*stock, goodInput());

    EXPECT_EQ(out.signature.skill_id, std::string("npt_taper_port_b120"));
    EXPECT_TRUE(out.signature.pattern.at("is_compound").get<bool>());
    EXPECT_EQ(out.signature.pattern.at("port_standard").get<std::string>(),
              std::string("NPT_ASME_B1_20_1"));
    EXPECT_EQ(out.signature.pattern.at("thread_size").get<std::string>(),
              std::string("1/4"));
}

// ─── 5. recognize metadata replay ─────────────────────────────────────────
TEST(SkillNptTaperPortB120, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(80.0, 80.0, 25.0);
    auto out   = skill::npt_taper_port_b120::apply(*stock, goodInput());

    auto cands = skill::npt_taper_port_b120::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands.front().confidence, 1.0, 1e-9);
    EXPECT_EQ(cands.front().recovered_params["thread_size"].get<std::string>(),
              std::string("1/4"));
}
