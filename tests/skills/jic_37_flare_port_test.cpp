// @lat: [[process/test-strategy#hydraulic ports — jic_37_flare_port]]
//
// Tests:
//   1. apply removes real volume; bore_dia matches central table.
//   2. validate rejects unknown tube_size.
//   3. validate rejects insufficient material.
//   4. signature compound + standard + half-angle 37°.
//   5. recognize metadata replay at confidence 1.0.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/_hydraulic_ports.hpp"
#include "skills/jic_37_flare_port.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

using namespace koocadcam;

namespace {

double volumeOf(const TopoDS_Shape& s) {
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}

skill::jic_37_flare_port::Input goodInput()
{
    skill::jic_37_flare_port::Input in;
    in.face_id     = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.center_x_mm = 50.0;
    in.center_y_mm = 50.0;
    in.axis_dir    = gp_Dir(0, 0, -1);
    in.tube_size   = "-8";   // bore_dia = 11.10 mm
    return in;
}

}  // namespace

// ─── 1. apply + bore_dia matches table within 0.05 mm ─────────────────────
TEST(SkillJic37FlarePort, ApplyAndBoreDiaMatchesTable)
{
    auto stock = skill::createCuboidStock(80.0, 80.0, 25.0);
    const double v0 = volumeOf(stock->shape());
    auto out = skill::jic_37_flare_port::apply(*stock, goodInput());
    EXPECT_GT(v0 - volumeOf(out.workpiece->shape()), 0.0);

    const auto* spec = skill::hyd_ports::findJicFlare("-8");
    ASSERT_NE(spec, nullptr);
    EXPECT_NEAR(out.signature.pattern.at("bore_dia_mm").get<double>(),
                spec->bore_dia_mm, 0.05);
    EXPECT_NEAR(out.signature.pattern.at("port_dia_mm").get<double>(),
                spec->bore_dia_mm, 0.05);
    EXPECT_NEAR(out.signature.pattern.at("flare_seat_od_mm").get<double>(),
                spec->flare_seat_od_mm, 0.05);
    EXPECT_NEAR(out.signature.pattern.at("flare_half_angle_deg").get<double>(),
                37.0, 1e-6);
}

// ─── 2. validate rejects unknown tube_size ────────────────────────────────
TEST(SkillJic37FlarePort, ValidateRejectsUnknownTubeSize)
{
    auto stock = skill::createCuboidStock(80.0, 80.0, 25.0);
    auto in = goodInput();
    in.tube_size = "-99";
    auto r = skill::jic_37_flare_port::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-JIC-CODE") { found = true; break; }
    EXPECT_TRUE(found);
    EXPECT_THROW(skill::jic_37_flare_port::apply(*stock, in), skill::SkillError);
}

// ─── 3. validate rejects too-small face ───────────────────────────────────
TEST(SkillJic37FlarePort, ValidateRejectsInsufficientMaterial)
{
    // -8 flare_seat_od = 15.24 mm → need ≥ 22.86 mm; 18 mm fails.
    auto tiny = skill::createCuboidStock(18.0, 18.0, 25.0);
    auto in = goodInput();
    in.center_x_mm = 9.0;
    in.center_y_mm = 9.0;
    auto r = skill::jic_37_flare_port::validate(*tiny, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-JIC-MAT") { found = true; break; }
    EXPECT_TRUE(found);
}

// ─── 4. signature compound + standard ─────────────────────────────────────
TEST(SkillJic37FlarePort, SignatureCompoundAndStandard)
{
    auto stock = skill::createCuboidStock(80.0, 80.0, 25.0);
    auto out   = skill::jic_37_flare_port::apply(*stock, goodInput());

    EXPECT_EQ(out.signature.skill_id, std::string("jic_37_flare_port"));
    EXPECT_TRUE(out.signature.pattern.at("is_compound").get<bool>());
    EXPECT_EQ(out.signature.pattern.at("port_standard").get<std::string>(),
              std::string("JIC_37_SAE_J514"));
    EXPECT_EQ(out.signature.pattern.at("tube_size").get<std::string>(),
              std::string("-8"));
}

// ─── 5. recognize metadata replay ─────────────────────────────────────────
TEST(SkillJic37FlarePort, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(80.0, 80.0, 25.0);
    auto out   = skill::jic_37_flare_port::apply(*stock, goodInput());

    auto cands = skill::jic_37_flare_port::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands.front().confidence, 1.0, 1e-9);
    EXPECT_EQ(cands.front().recovered_params["tube_size"].get<std::string>(),
              std::string("-8"));
}
