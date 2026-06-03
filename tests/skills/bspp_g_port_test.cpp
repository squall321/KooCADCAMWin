// @lat: [[process/test-strategy#hydraulic ports — bspp_g_port]]
//
// bspp_g_port — BSPP / ISO 228-1 G-thread port: tap-drill bore + face
// counterbore relief + entry chamfer.
//
// Tests:
//   1. apply removes volume within ±20 % of derived sum-of-parts.
//   2. validate rejects unknown g_size (DFM-BSPP-CODE).
//   3. validate rejects thin face (DFM-BSPP-DEPTH).
//   4. signature carries is_compound + port_standard + size_code.
//   5. recognize replays metadata at confidence 1.0.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/bspp_g_port.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

#include <cmath>

using namespace koocadcam;

namespace {
double volumeOf(const TopoDS_Shape& s) {
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}

skill::bspp_g_port::Input goodInput()
{
    skill::bspp_g_port::Input in;
    in.face_id       = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 30.0;
    in.position_y_mm = 30.0;
    in.axis_dir      = gp_Dir(0, 0, -1);
    in.g_size        = "G1/4";
    in.depth_mm      = 12.0;
    return in;
}
}  // namespace

// ─── 1. apply produces volume change ──────────────────────────────────────
TEST(SkillBsppGPort, ApplyProducesVolumeChangeAndExpectedRemoval)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 20.0);
    const double v0 = volumeOf(stock->shape());
    const int    f0 = stock->faceCount();

    auto out = skill::bspp_g_port::apply(*stock, goodInput());
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    const double v1 = volumeOf(out.workpiece->shape());
    const double dv = v0 - v1;
    EXPECT_GT(dv, 0.0);
    EXPECT_NE(out.workpiece->faceCount(), f0);

    const double derived = out.signature.pattern.at("derived_volume_removed")
                            .get<double>();
    EXPECT_GT(derived, 0.0);
    EXPECT_NEAR(dv, derived, derived * 0.20);
}

// ─── 2. validate rejects unknown g_size ────────────────────────────────────
TEST(SkillBsppGPort, ValidateRejectsUnknownGSize)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 20.0);

    auto in = goodInput();
    in.g_size = "G5/4";   // not in table
    auto r = skill::bspp_g_port::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-BSPP-CODE") { found = true; break; }
    EXPECT_TRUE(found);
    EXPECT_THROW(skill::bspp_g_port::apply(*stock, in), skill::SkillError);
}

// ─── 3. validate rejects thin face ─────────────────────────────────────────
TEST(SkillBsppGPort, ValidateRejectsTooThinFace)
{
    auto thin = skill::createCuboidStock(60.0, 60.0, 8.0);
    auto in = goodInput();   // depth 12, thickness 8 → fail
    auto r = skill::bspp_g_port::validate(*thin, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-BSPP-DEPTH") { found = true; break; }
    EXPECT_TRUE(found);
}

// ─── 4. Signature records compound + port_standard + size_code ─────────────
TEST(SkillBsppGPort, SignatureRecordsCompoundKindAndPortStandard)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 20.0);
    auto out   = skill::bspp_g_port::apply(*stock, goodInput());

    EXPECT_EQ(out.signature.skill_id, std::string("bspp_g_port"));
    EXPECT_TRUE(out.signature.pattern.at("is_compound").get<bool>());
    EXPECT_EQ(out.signature.pattern.at("port_standard").get<std::string>(),
              std::string("BSPP_ISO_228"));
    EXPECT_EQ(out.signature.pattern.at("size_code").get<std::string>(),
              std::string("G1/4"));
    EXPECT_EQ(out.signature.pattern.at("subfeature_count").get<int>(), 3);

    EXPECT_NEAR(out.signature.pattern.at("thread_od_mm").get<double>(), 13.157, 1e-3);
    EXPECT_NEAR(out.signature.pattern.at("tap_drill_dia_mm").get<double>(), 11.80, 1e-3);
}

// ─── 5. recognize replays metadata at confidence 1.0 ───────────────────────
TEST(SkillBsppGPort, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 20.0);
    auto out   = skill::bspp_g_port::apply(*stock, goodInput());

    auto cands = skill::bspp_g_port::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands.front().confidence, 1.0, 1e-9);
    EXPECT_EQ(cands.front().recovered_params["g_size"].get<std::string>(),
              std::string("G1/4"));
}
