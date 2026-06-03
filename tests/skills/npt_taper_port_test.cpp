// @lat: [[process/test-strategy#hydraulic ports — npt_taper_port]]
//
// npt_taper_port — ANSI NPT taper port: cone frustum bore + counterbore
// relief + entry chamfer.
//
// Tests:
//   1. apply removes volume within ±20 % of derived sum-of-parts.
//   2. validate rejects unknown npt_size (DFM-NPT-CODE).
//   3. validate rejects thin face (DFM-NPT-DEPTH).
//   4. signature carries is_compound + port_standard + size_code + taper.
//   5. recognize replays metadata at confidence 1.0.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/npt_taper_port.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

#include <cmath>

using namespace koocadcam;

namespace {
double volumeOf(const TopoDS_Shape& s) {
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}

skill::npt_taper_port::Input goodInput()
{
    skill::npt_taper_port::Input in;
    in.face_id       = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 30.0;
    in.position_y_mm = 30.0;
    in.axis_dir      = gp_Dir(0, 0, -1);
    in.npt_size      = "1/4";
    in.depth_mm      = 12.0;
    return in;
}
}  // namespace

// ─── 1. apply produces volume change ──────────────────────────────────────
TEST(SkillNptTaperPort, ApplyProducesVolumeChangeAndExpectedRemoval)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 20.0);
    const double v0 = volumeOf(stock->shape());
    const int    f0 = stock->faceCount();

    auto out = skill::npt_taper_port::apply(*stock, goodInput());
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

// ─── 2. validate rejects unknown npt_size ─────────────────────────────────
TEST(SkillNptTaperPort, ValidateRejectsUnknownNptSize)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 20.0);
    auto in = goodInput();
    in.npt_size = "2-1/2";
    auto r = skill::npt_taper_port::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-NPT-CODE") { found = true; break; }
    EXPECT_TRUE(found);
    EXPECT_THROW(skill::npt_taper_port::apply(*stock, in), skill::SkillError);
}

// ─── 3. validate rejects thin face ────────────────────────────────────────
TEST(SkillNptTaperPort, ValidateRejectsTooThinFace)
{
    auto thin = skill::createCuboidStock(60.0, 60.0, 8.0);
    auto in = goodInput();
    auto r = skill::npt_taper_port::validate(*thin, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-NPT-DEPTH") { found = true; break; }
    EXPECT_TRUE(found);
}

// ─── 4. Signature compound kind + standard + taper ───────────────────────
TEST(SkillNptTaperPort, SignatureRecordsCompoundKindAndPortStandard)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 20.0);
    auto out   = skill::npt_taper_port::apply(*stock, goodInput());

    EXPECT_EQ(out.signature.skill_id, std::string("npt_taper_port"));
    EXPECT_TRUE(out.signature.pattern.at("is_compound").get<bool>());
    EXPECT_EQ(out.signature.pattern.at("port_standard").get<std::string>(),
              std::string("NPT_ASME_B1_20_1"));
    EXPECT_EQ(out.signature.pattern.at("size_code").get<std::string>(),
              std::string("1/4"));
    EXPECT_EQ(out.signature.pattern.at("subfeature_count").get<int>(), 3);

    // Taper per side ≈ 1.79°.
    EXPECT_NEAR(out.signature.pattern.at("taper_per_side_deg").get<double>(),
                1.7899, 1e-3);

    // Small end dia must be < major (taper actually tapers).
    const double major = out.signature.pattern.at("major_dia_mm").get<double>();
    const double small = out.signature.pattern.at("small_end_dia_mm").get<double>();
    EXPECT_LT(small, major);
}

// ─── 5. recognize replays metadata at confidence 1.0 ──────────────────────
TEST(SkillNptTaperPort, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 20.0);
    auto out   = skill::npt_taper_port::apply(*stock, goodInput());

    auto cands = skill::npt_taper_port::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands.front().confidence, 1.0, 1e-9);
    EXPECT_EQ(cands.front().recovered_params["npt_size"].get<std::string>(),
              std::string("1/4"));
}
