// @lat: [[process/test-strategy#hydraulic ports — sae_j518_port_code61]]
//
// Tests:
//   1. apply produces real volume change & port_dia matches table within 0.05 mm.
//   2. validate rejects unknown dash_size.
//   3. validate rejects insufficient material (face too small).
//   4. signature carries is_compound + port_standard + derived params.
//   5. recognize replays metadata at confidence 1.0.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/_hydraulic_ports.hpp"
#include "skills/sae_j518_port_code61.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

#include <cmath>

using namespace koocadcam;

namespace {

double volumeOf(const TopoDS_Shape& s) {
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}

skill::sae_j518_port_code61::Input goodInput()
{
    skill::sae_j518_port_code61::Input in;
    in.face_id     = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.center_x_mm = 60.0;
    in.center_y_mm = 60.0;
    in.axis_dir    = gp_Dir(0, 0, -1);
    in.dash_size   = "-12";   // port_dia = 19.05 mm, PCD = 38.10 mm
    in.depth_mm    = 16.0;
    return in;
}

}  // namespace

// ─── 1. apply + port_dia table match (≤ 0.05 mm) ──────────────────────────
TEST(SkillSaeJ518Code61, ApplyAndPortDiaMatchesTableWithinTolerance)
{
    auto stock = skill::createCuboidStock(120.0, 120.0, 25.0);
    const double v0 = volumeOf(stock->shape());

    auto out = skill::sae_j518_port_code61::apply(*stock, goodInput());
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    const double v1 = volumeOf(out.workpiece->shape());
    EXPECT_GT(v0 - v1, 0.0);
    EXPECT_NE(out.workpiece->faceCount(), stock->faceCount());

    // KEY ASSERTION: port_dia in signature pattern == table value within 0.05 mm.
    const auto* spec =
        skill::hyd_ports::findSaeJ518Code61("-12");
    ASSERT_NE(spec, nullptr);
    EXPECT_NEAR(out.signature.pattern.at("port_dia_mm").get<double>(),
                spec->port_dia_mm, 0.05);

    // Bolt circle and bolt dia also match table.
    EXPECT_NEAR(out.signature.pattern.at("bolt_circle_dia_mm").get<double>(),
                spec->bolt_circle_dia_mm, 0.05);
    EXPECT_NEAR(out.signature.pattern.at("bolt_dia_mm").get<double>(),
                spec->bolt_dia_mm, 0.05);
    EXPECT_EQ(out.signature.pattern.at("bolt_count").get<int>(), 4);
}

// ─── 2. validate rejects unknown dash_size ────────────────────────────────
TEST(SkillSaeJ518Code61, ValidateRejectsUnknownDashSize)
{
    auto stock = skill::createCuboidStock(120.0, 120.0, 25.0);
    auto in    = goodInput();
    in.dash_size = "-99";
    auto r = skill::sae_j518_port_code61::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-J518C61-CODE") { found = true; break; }
    EXPECT_TRUE(found);
    EXPECT_THROW(skill::sae_j518_port_code61::apply(*stock, in), skill::SkillError);
}

// ─── 3. validate rejects too-small face (insufficient material) ───────────
TEST(SkillSaeJ518Code61, ValidateRejectsInsufficientMaterial)
{
    // -12 PCD = 38.1 mm → need ≥ 57.15 mm face span; 40 mm fails.
    auto tiny = skill::createCuboidStock(40.0, 40.0, 25.0);
    auto in   = goodInput();
    in.center_x_mm = 20.0;
    in.center_y_mm = 20.0;
    auto r = skill::sae_j518_port_code61::validate(*tiny, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-J518C61-MAT") { found = true; break; }
    EXPECT_TRUE(found);
}

// ─── 4. signature compound + standard + derived params ────────────────────
TEST(SkillSaeJ518Code61, SignatureCompoundAndStandard)
{
    auto stock = skill::createCuboidStock(120.0, 120.0, 25.0);
    auto out   = skill::sae_j518_port_code61::apply(*stock, goodInput());

    EXPECT_EQ(out.signature.skill_id, std::string("sae_j518_port_code61"));
    EXPECT_TRUE(out.signature.pattern.at("is_compound").get<bool>());
    EXPECT_EQ(out.signature.pattern.at("port_standard").get<std::string>(),
              std::string("SAE_J518_Code61"));
    EXPECT_EQ(out.signature.pattern.at("dash_size").get<std::string>(),
              std::string("-12"));
    EXPECT_GT(out.signature.pattern.at("derived_volume_removed").get<double>(), 0.0);
}

// ─── 5. recognize replays metadata at confidence 1.0 ──────────────────────
TEST(SkillSaeJ518Code61, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(120.0, 120.0, 25.0);
    auto out   = skill::sae_j518_port_code61::apply(*stock, goodInput());

    auto cands = skill::sae_j518_port_code61::recognize(*out.workpiece);
    ASSERT_GE(cands.size(), 1u);
    EXPECT_NEAR(cands.front().confidence, 1.0, 1e-9);
    EXPECT_EQ(cands.front().recovered_params["dash_size"].get<std::string>(),
              std::string("-12"));
}
