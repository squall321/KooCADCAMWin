// @lat: [[process/test-strategy#skill round-trip]]
//
// snap_assembly — rectangular cavity for a mating tab.
//
// Cases (6):
//   1. Apply cuts a rectangular cavity (subtractive volume).
//   2. Signature carries cavity dimensions + tab_engage + partner_part.
//   3. DFM rejects sub-1 mm cavity dimensions.
//   4. DFM rejects tab_engage >= cavity_depth.
//   5. DFM rejects depth outside [0.5, 10.0].
//   6. Recognize replays history.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/snap_assembly.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

#include <cmath>

using namespace koocadcam;

namespace {

double volumeOf(const TopoDS_Shape& s)
{
    GProp_GProps p;
    BRepGProp::VolumeProperties(s, p);
    return p.Mass();
}

}  // namespace

// ─── 1. Apply cuts the cavity ────────────────────────────────────────────
TEST(SkillSnapAssembly, ApplyRemovesCavityVolume)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);
    const double volBefore = volumeOf(stock->shape());

    skill::snap_assembly::Input in;
    in.entry_face        = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm     = 25.0;
    in.position_y_mm     = 25.0;
    in.insertion_axis_xy = gp_Dir(1.0, 0.0, 0.0);
    in.cavity_length_mm  = 8.0;
    in.cavity_width_mm   = 3.0;
    in.cavity_depth_mm   = 2.0;
    in.tab_engage_mm     = 1.5;
    in.partner_part      = "lid";

    auto out = skill::snap_assembly::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    const double expected = 8.0 * 3.0 * 2.0;        // = 48 mm³
    const double removed  = volBefore - volumeOf(out.workpiece->shape());
    EXPECT_NEAR(removed, expected, expected * 0.05);

    EXPECT_EQ(out.signature.skill_id, std::string("snap_assembly"));
}

// ─── 2. Signature carries cavity metadata ────────────────────────────────
TEST(SkillSnapAssembly, SignatureCarriesMetadata)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::snap_assembly::Input in;
    in.entry_face        = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm     = 25.0;
    in.position_y_mm     = 25.0;
    in.insertion_axis_xy = gp_Dir(1.0, 0.0, 0.0);
    in.cavity_length_mm  = 6.0;
    in.cavity_width_mm   = 4.0;
    in.cavity_depth_mm   = 1.8;
    in.tab_engage_mm     = 1.2;
    in.partner_part      = "battery_door";

    auto out = skill::snap_assembly::apply(*stock, in);
    EXPECT_NEAR(out.signature.params["cavity_length_mm"].get<double>(), 6.0, 1e-9);
    EXPECT_NEAR(out.signature.params["cavity_width_mm"].get<double>(),  4.0, 1e-9);
    EXPECT_NEAR(out.signature.params["cavity_depth_mm"].get<double>(),  1.8, 1e-9);
    EXPECT_NEAR(out.signature.params["tab_engage_mm"].get<double>(),    1.2, 1e-9);
    EXPECT_EQ(out.signature.params["partner_part"].get<std::string>(), std::string("battery_door"));
    EXPECT_EQ(out.signature.pattern["planar_wall_count"].get<int>(), 4);
}

// ─── 3. DFM rejects too-small cavity ─────────────────────────────────────
TEST(SkillSnapAssembly, ValidateRejectsTinyCavity)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::snap_assembly::Input in;
    in.entry_face        = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm     = 25.0;
    in.position_y_mm     = 25.0;
    in.insertion_axis_xy = gp_Dir(1.0, 0.0, 0.0);
    in.cavity_length_mm  = 0.5;       // < 1.0
    in.cavity_width_mm   = 3.0;
    in.cavity_depth_mm   = 2.0;
    in.tab_engage_mm     = 1.0;

    auto r = skill::snap_assembly::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool sawCode = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-SA-LENGTH") { sawCode = true; break; }
    EXPECT_TRUE(sawCode);
    EXPECT_THROW(skill::snap_assembly::apply(*stock, in), skill::SkillError);
}

// ─── 4. DFM rejects tab_engage >= depth ──────────────────────────────────
TEST(SkillSnapAssembly, ValidateRejectsTabTooDeep)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::snap_assembly::Input in;
    in.entry_face        = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm     = 25.0;
    in.position_y_mm     = 25.0;
    in.insertion_axis_xy = gp_Dir(1.0, 0.0, 0.0);
    in.cavity_length_mm  = 8.0;
    in.cavity_width_mm   = 3.0;
    in.cavity_depth_mm   = 2.0;
    in.tab_engage_mm     = 2.5;       // > depth

    auto r = skill::snap_assembly::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool sawCode = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-SA-TAB") { sawCode = true; break; }
    EXPECT_TRUE(sawCode);
}

// ─── 5. DFM rejects bad cavity_depth ─────────────────────────────────────
TEST(SkillSnapAssembly, ValidateRejectsDepth)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 20.0);

    skill::snap_assembly::Input in;
    in.entry_face        = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm     = 25.0;
    in.position_y_mm     = 25.0;
    in.insertion_axis_xy = gp_Dir(1.0, 0.0, 0.0);
    in.cavity_length_mm  = 8.0;
    in.cavity_width_mm   = 3.0;
    in.cavity_depth_mm   = 12.0;      // > 10.0
    in.tab_engage_mm     = 6.0;

    auto r = skill::snap_assembly::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool sawCode = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-SA-DEPTH") { sawCode = true; break; }
    EXPECT_TRUE(sawCode);
}

// ─── 6. Recognize replays history ────────────────────────────────────────
TEST(SkillSnapAssembly, RecognizeReplaysFromHistory)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::snap_assembly::Input in;
    in.entry_face        = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm     = 20.0;
    in.position_y_mm     = 30.0;
    in.insertion_axis_xy = gp_Dir(0.0, 1.0, 0.0);
    in.cavity_length_mm  = 5.0;
    in.cavity_width_mm   = 2.5;
    in.cavity_depth_mm   = 1.5;
    in.tab_engage_mm     = 1.0;
    in.partner_part      = "cap";

    auto out = skill::snap_assembly::apply(*stock, in);
    auto cands = skill::snap_assembly::recognize(*out.workpiece);

    ASSERT_FALSE(cands.empty());
    const auto& c = cands.front();
    EXPECT_NEAR(c.confidence, 1.0, 1e-9);
    EXPECT_NEAR(c.recovered_params["cavity_length_mm"].get<double>(), 5.0, 1e-6);
    EXPECT_NEAR(c.recovered_params["cavity_width_mm"].get<double>(),  2.5, 1e-6);
    EXPECT_NEAR(c.recovered_params["cavity_depth_mm"].get<double>(),  1.5, 1e-6);
    EXPECT_NEAR(c.recovered_params["tab_engage_mm"].get<double>(),    1.0, 1e-6);
    EXPECT_EQ(c.recovered_params["partner_part"].get<std::string>(), std::string("cap"));
}
