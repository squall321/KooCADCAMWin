// @lat: [[process/test-strategy#hydraulic ports — jic_37deg_flare_port]]
//
// jic_37deg_flare_port — JIC 37° flare seat: UNF bore + conical flare seat +
// inner flow passage.
//
// Tests:
//   1. apply removes volume within ±20 % of derived sum-of-parts.
//   2. validate rejects unknown jic_dash (DFM-JIC-CODE).
//   3. validate rejects thin face (DFM-JIC-DEPTH).
//   4. signature carries is_compound + port_standard + 37° flare angle.
//   5. recognize replays metadata at confidence 1.0.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/jic_37deg_flare_port.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

#include <cmath>

using namespace koocadcam;

namespace {
double volumeOf(const TopoDS_Shape& s) {
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}

skill::jic_37deg_flare_port::Input goodInput()
{
    skill::jic_37deg_flare_port::Input in;
    in.face_id       = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 40.0;
    in.position_y_mm = 40.0;
    in.axis_dir      = gp_Dir(0, 0, -1);
    in.jic_dash      = "-08";   // 3/4-16 UNF
    in.depth_mm      = 14.0;
    return in;
}
}  // namespace

// ─── 1. apply produces volume change ──────────────────────────────────────
TEST(SkillJic37FlarePort, ApplyProducesVolumeChangeAndExpectedRemoval)
{
    auto stock = skill::createCuboidStock(80.0, 80.0, 22.0);
    const double v0 = volumeOf(stock->shape());
    const int    f0 = stock->faceCount();

    auto out = skill::jic_37deg_flare_port::apply(*stock, goodInput());
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

// ─── 2. validate rejects unknown jic_dash ─────────────────────────────────
TEST(SkillJic37FlarePort, ValidateRejectsUnknownJicDash)
{
    auto stock = skill::createCuboidStock(80.0, 80.0, 22.0);
    auto in = goodInput();
    in.jic_dash = "-99";
    auto r = skill::jic_37deg_flare_port::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-JIC-CODE") { found = true; break; }
    EXPECT_TRUE(found);
    EXPECT_THROW(skill::jic_37deg_flare_port::apply(*stock, in), skill::SkillError);
}

// ─── 3. validate rejects thin face ────────────────────────────────────────
TEST(SkillJic37FlarePort, ValidateRejectsTooThinFace)
{
    auto thin = skill::createCuboidStock(80.0, 80.0, 10.0);
    auto in = goodInput();
    auto r = skill::jic_37deg_flare_port::validate(*thin, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-JIC-DEPTH") { found = true; break; }
    EXPECT_TRUE(found);
}

// ─── 4. Signature compound kind + standard + 37° flare ───────────────────
TEST(SkillJic37FlarePort, SignatureRecordsCompoundKindAndFlareAngle)
{
    auto stock = skill::createCuboidStock(80.0, 80.0, 22.0);
    auto out   = skill::jic_37deg_flare_port::apply(*stock, goodInput());

    EXPECT_EQ(out.signature.skill_id, std::string("jic_37deg_flare_port"));
    EXPECT_TRUE(out.signature.pattern.at("is_compound").get<bool>());
    EXPECT_EQ(out.signature.pattern.at("port_standard").get<std::string>(),
              std::string("JIC_37_SAE_J514"));
    EXPECT_EQ(out.signature.pattern.at("size_code").get<std::string>(),
              std::string("-08"));
    EXPECT_EQ(out.signature.pattern.at("subfeature_count").get<int>(), 3);

    EXPECT_NEAR(out.signature.pattern.at("flare_half_angle_deg").get<double>(),
                37.0, 1e-6);

    // Flare narrows from flare_od_mm to flare_min_dia_mm.
    const double fOD = out.signature.pattern.at("flare_od_mm").get<double>();
    const double fMn = out.signature.pattern.at("flare_min_dia_mm").get<double>();
    EXPECT_LT(fMn, fOD);
}

// ─── 5. recognize replays metadata at confidence 1.0 ──────────────────────
TEST(SkillJic37FlarePort, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(80.0, 80.0, 22.0);
    auto out   = skill::jic_37deg_flare_port::apply(*stock, goodInput());

    auto cands = skill::jic_37deg_flare_port::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands.front().confidence, 1.0, 1e-9);
    EXPECT_EQ(cands.front().recovered_params["jic_dash"].get<std::string>(),
              std::string("-08"));
}
