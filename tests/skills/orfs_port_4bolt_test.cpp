// @lat: [[process/test-strategy#hydraulic ports — orfs_port_4bolt]]
//
// Tests:
//   1. apply removes real volume; port_dia matches central table.
//   2. validate rejects unknown dash_size.
//   3. validate rejects insufficient material.
//   4. signature compound + standard + bolt_count=4.
//   5. recognize metadata replay at confidence 1.0.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/_hydraulic_ports.hpp"
#include "skills/orfs_port_4bolt.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

using namespace koocadcam;

namespace {

double volumeOf(const TopoDS_Shape& s) {
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}

skill::orfs_port_4bolt::Input goodInput()
{
    skill::orfs_port_4bolt::Input in;
    in.face_id     = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.center_x_mm = 50.0;
    in.center_y_mm = 50.0;
    in.axis_dir    = gp_Dir(0, 0, -1);
    in.dash_size   = "-8";   // port_dia = 12.70 mm
    in.depth_mm    = 12.0;
    return in;
}

}  // namespace

// ─── 1. apply + port_dia matches table within 0.05 mm ────────────────────
TEST(SkillOrfsPort4Bolt, ApplyAndPortDiaMatchesTable)
{
    auto stock = skill::createCuboidStock(100.0, 100.0, 25.0);
    const double v0 = volumeOf(stock->shape());
    auto out = skill::orfs_port_4bolt::apply(*stock, goodInput());
    EXPECT_GT(v0 - volumeOf(out.workpiece->shape()), 0.0);

    const auto* spec = skill::hyd_ports::findOrfsPort("-8");
    ASSERT_NE(spec, nullptr);
    EXPECT_NEAR(out.signature.pattern.at("port_dia_mm").get<double>(),
                spec->port_dia_mm, 0.05);
    EXPECT_NEAR(out.signature.pattern.at("bolt_circle_dia_mm").get<double>(),
                spec->bolt_circle_dia_mm, 0.05);
    EXPECT_EQ(out.signature.pattern.at("bolt_count").get<int>(), 4);
}

// ─── 2. validate rejects unknown dash_size ────────────────────────────────
TEST(SkillOrfsPort4Bolt, ValidateRejectsUnknownDashSize)
{
    auto stock = skill::createCuboidStock(100.0, 100.0, 25.0);
    auto in = goodInput();
    in.dash_size = "-99";
    auto r = skill::orfs_port_4bolt::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-ORFS-CODE") { found = true; break; }
    EXPECT_TRUE(found);
    EXPECT_THROW(skill::orfs_port_4bolt::apply(*stock, in), skill::SkillError);
}

// ─── 3. validate rejects too-small face ───────────────────────────────────
TEST(SkillOrfsPort4Bolt, ValidateRejectsInsufficientMaterial)
{
    // -8 bcd = 30 mm → need ≥ 45 mm; 30 mm fails.
    auto tiny = skill::createCuboidStock(30.0, 30.0, 25.0);
    auto in = goodInput();
    in.center_x_mm = 15.0;
    in.center_y_mm = 15.0;
    auto r = skill::orfs_port_4bolt::validate(*tiny, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-ORFS-MAT") { found = true; break; }
    EXPECT_TRUE(found);
}

// ─── 4. signature compound + standard ─────────────────────────────────────
TEST(SkillOrfsPort4Bolt, SignatureCompoundAndStandard)
{
    auto stock = skill::createCuboidStock(100.0, 100.0, 25.0);
    auto out   = skill::orfs_port_4bolt::apply(*stock, goodInput());

    EXPECT_EQ(out.signature.skill_id, std::string("orfs_port_4bolt"));
    EXPECT_TRUE(out.signature.pattern.at("is_compound").get<bool>());
    EXPECT_EQ(out.signature.pattern.at("port_standard").get<std::string>(),
              std::string("ORFS_4bolt_SAE_J1453"));
    EXPECT_EQ(out.signature.pattern.at("dash_size").get<std::string>(),
              std::string("-8"));
}

// ─── 5. recognize metadata replay ─────────────────────────────────────────
TEST(SkillOrfsPort4Bolt, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(100.0, 100.0, 25.0);
    auto out   = skill::orfs_port_4bolt::apply(*stock, goodInput());

    auto cands = skill::orfs_port_4bolt::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands.front().confidence, 1.0, 1e-9);
    EXPECT_EQ(cands.front().recovered_params["dash_size"].get<std::string>(),
              std::string("-8"));
}
