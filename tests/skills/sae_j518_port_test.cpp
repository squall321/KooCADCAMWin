// @lat: [[process/test-strategy#hydraulic ports — sae_j518_port]]
//
// sae_j518_port — 4-bolt split-flange port (Code 61).  Sub-cuts: central
// bore + 4 bolt-pattern holes on PCD + face O-ring groove + entry chamfer.
//
// Tests:
//   1. apply removes real volume; face count changes; volume within ±20 %
//      of derived sum-of-parts.
//   2. validate rejects unknown sae_code (DFM-J518-CODE).
//   3. validate rejects thin face (DFM-J518-DEPTH).
//   4. signature carries is_compound + port_standard + size_code + derived
//      volume_removed.
//   5. recognize replays metadata at confidence 1.0.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/sae_j518_port.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

#include <cmath>

using namespace koocadcam;

namespace {

double volumeOf(const TopoDS_Shape& s) {
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}

skill::sae_j518_port::Input goodInput()
{
    skill::sae_j518_port::Input in;
    in.face_id       = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 50.0;
    in.position_y_mm = 50.0;
    in.axis_dir      = gp_Dir(0, 0, -1);
    in.sae_code      = "-12";   // thru = 25.4, PCD = 38.1, bolt = 10
    in.depth_mm      = 14.0;
    return in;
}

}  // namespace

// ─── 1. apply removes volume; face count changes ──────────────────────────
TEST(SkillSaeJ518Port, ApplyProducesVolumeChangeAndExpectedRemoval)
{
    auto stock = skill::createCuboidStock(100.0, 100.0, 25.0);
    const double v0 = volumeOf(stock->shape());
    const int    f0 = stock->faceCount();

    auto out = skill::sae_j518_port::apply(*stock, goodInput());
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    const double v1 = volumeOf(out.workpiece->shape());
    const double dv = v0 - v1;
    EXPECT_GT(dv, 0.0);
    EXPECT_NE(out.workpiece->faceCount(), f0);

    // Sum-of-parts ± 20 %.
    const double derived = out.signature.pattern.at("derived_volume_removed")
                            .get<double>();
    EXPECT_GT(derived, 0.0);
    EXPECT_NEAR(dv, derived, derived * 0.20);
}

// ─── 2. validate rejects unknown code ──────────────────────────────────────
TEST(SkillSaeJ518Port, ValidateRejectsUnknownSaeCode)
{
    auto stock = skill::createCuboidStock(100.0, 100.0, 25.0);

    auto in = goodInput();
    in.sae_code = "-32";  // not in table
    auto r = skill::sae_j518_port::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-J518-CODE") { found = true; break; }
    EXPECT_TRUE(found);
    EXPECT_THROW(skill::sae_j518_port::apply(*stock, in), skill::SkillError);
}

// ─── 3. validate rejects thin face ─────────────────────────────────────────
TEST(SkillSaeJ518Port, ValidateRejectsTooThinFace)
{
    // Stock thickness = 8 mm; depth 14 mm requires 16 mm of material.
    auto thin = skill::createCuboidStock(100.0, 100.0, 8.0);
    auto in = goodInput();
    auto r = skill::sae_j518_port::validate(*thin, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-J518-DEPTH") { found = true; break; }
    EXPECT_TRUE(found);
}

// ─── 4. Signature records compound + port_standard + size_code ─────────────
TEST(SkillSaeJ518Port, SignatureRecordsCompoundKindAndPortStandard)
{
    auto stock = skill::createCuboidStock(100.0, 100.0, 25.0);
    auto out   = skill::sae_j518_port::apply(*stock, goodInput());

    EXPECT_EQ(out.signature.skill_id, std::string("sae_j518_port"));
    EXPECT_TRUE(out.signature.pattern.at("is_compound").get<bool>());
    EXPECT_EQ(out.signature.pattern.at("port_standard").get<std::string>(),
              std::string("SAE_J518"));
    EXPECT_EQ(out.signature.pattern.at("size_code").get<std::string>(),
              std::string("-12"));
    EXPECT_GT(out.signature.pattern.at("derived_volume_removed").get<double>(), 0.0);

    // Derived spec values present.
    EXPECT_NEAR(out.signature.pattern.at("thru_dia_mm").get<double>(), 25.4, 1e-6);
    EXPECT_NEAR(out.signature.pattern.at("pcd_mm").get<double>(),     38.1, 1e-6);
    EXPECT_NEAR(out.signature.pattern.at("bolt_dia_mm").get<double>(), 10.0, 1e-6);
}

// ─── 5. recognize replays metadata at confidence 1.0 ───────────────────────
TEST(SkillSaeJ518Port, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(100.0, 100.0, 25.0);
    auto out   = skill::sae_j518_port::apply(*stock, goodInput());

    auto cands = skill::sae_j518_port::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands.front().confidence, 1.0, 1e-9);
    EXPECT_EQ(cands.front().recovered_params["sae_code"].get<std::string>(),
              std::string("-12"));
}
