// @lat: [[process/test-strategy#compound feature]]
//
// banana_jack_receptacle — 3-cut compound: through hole + counterbore +
// rear DIN-471 snap-ring slot, repeated `count` times.
//
// Test cases:
//   1. ApplyProducesValidShapeWithMultipleSubFeatures
//   2. ValidateRejectsBadInput — panel too thin for compound feature
//   3. SignatureRecordsCompoundKind
//   4. RecognizeMetadataReplay
//   5. RingGrooveMatchesDIN471Catalog — Ø7 × 0.9 from the 7 mm row

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/banana_jack_receptacle.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

#include <cmath>

using namespace koocadcam;

namespace {
double volumeOf(const TopoDS_Shape& s) {
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}
}  // namespace

// ─── 1. Apply produces valid shape ────────────────────────────────────────
TEST(SkillBananaJackReceptacle, ApplyProducesValidShapeWithMultipleSubFeatures)
{
    auto stock = skill::createCuboidStock(60.0, 30.0, 10.0);  // 10 mm panel ≥ 8

    skill::banana_jack_receptacle::Input in;
    in.face          = skill::FaceByNormal{ gp_Dir(0, 0, 1), 5.0, "largest" };
    in.position_x_mm = 15.0;
    in.position_y_mm = 15.0;
    in.axis_dir      = gp_Dir(0, 0, -1);
    in.count         = 2;
    in.spacing_mm    = 19.05;
    in.direction_deg = 0.0;

    const int facesBefore = stock->faceCount();
    auto out = skill::banana_jack_receptacle::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    // Volume per jack:
    //  through (Ø4.1) × 10 mm = π × 2.05² × 10  ≈ 132 mm³
    //  counterbore additional ring (Ø9² − Ø4.1²) / 4 × 1.5 ≈ 81 mm³
    //  snap-ring slot ring (Ø7² − Ø4.1²) / 4 × 0.9 ≈ 22.7 mm³
    constexpr double pi = M_PI;
    const double perJack =
        pi * 2.05 * 2.05 * 10.0 +
        pi * (4.5 * 4.5 - 2.05 * 2.05) * 1.5 +
        pi * (3.5 * 3.5 - 2.05 * 2.05) * 0.9;
    const double approx  = perJack * 2;
    const double removed = volumeOf(stock->shape()) - volumeOf(out.workpiece->shape());
    EXPECT_NEAR(removed, approx, approx * 0.15);

    EXPECT_GT(out.workpiece->faceCount(), facesBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("banana_jack_receptacle"));
}

// ─── 2. Validate rejects too-thin panel ───────────────────────────────────
TEST(SkillBananaJackReceptacle, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(60.0, 30.0, 5.0);  // 5 mm < 8 mm min

    skill::banana_jack_receptacle::Input in;
    in.face          = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 15.0;
    in.position_y_mm = 15.0;
    in.count         = 1;

    auto r = skill::banana_jack_receptacle::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-BANANA-PANEL") { found = true; break; }
    EXPECT_TRUE(found);

    EXPECT_THROW(skill::banana_jack_receptacle::apply(*stock, in), skill::SkillError);
}

// ─── 3. Signature records compound kind ───────────────────────────────────
TEST(SkillBananaJackReceptacle, SignatureRecordsCompoundKind)
{
    auto stock = skill::createCuboidStock(60.0, 30.0, 10.0);
    skill::banana_jack_receptacle::Input in;
    in.face          = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 15.0;
    in.position_y_mm = 15.0;
    in.count         = 3;
    auto out = skill::banana_jack_receptacle::apply(*stock, in);

    const auto& pat = out.signature.pattern;
    EXPECT_TRUE(pat.at("is_compound").get<bool>());
    EXPECT_GE(pat.at("subfeature_count").get<int>(), 2);
    EXPECT_EQ(pat.at("kind").get<std::string>(),
              std::string("banana_jack_receptacle"));
    EXPECT_EQ(pat.at("jack_count").get<int>(), 3);
    EXPECT_EQ(pat.at("positions").size(), 3u);
}

// ─── 4. Recognize: metadata replay → confidence 1.0 ───────────────────────
TEST(SkillBananaJackReceptacle, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(60.0, 30.0, 10.0);
    skill::banana_jack_receptacle::Input in;
    in.face          = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 15.0;
    in.position_y_mm = 15.0;
    in.count         = 2;
    auto out = skill::banana_jack_receptacle::apply(*stock, in);

    auto cands = skill::banana_jack_receptacle::recognize(*out.workpiece);
    ASSERT_GE(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-9);
    EXPECT_EQ(cands[0].recovered_params["count"].get<int>(), 2);
}

// ─── 5. Snap-ring groove matches DIN 471 internal Ø7 catalog row ──────────
TEST(SkillBananaJackReceptacle, RingGrooveMatchesDIN471Catalog)
{
    auto stock = skill::createCuboidStock(60.0, 30.0, 10.0);
    skill::banana_jack_receptacle::Input in;
    in.face          = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 15.0;
    in.position_y_mm = 15.0;
    in.count         = 1;
    auto out = skill::banana_jack_receptacle::apply(*stock, in);

    const auto& pat = out.signature.pattern;
    // DIN 471 internal Ø7 row: groove Ø = 7.4 → 7.0 (we use 7.0), depth 0.9.
    EXPECT_NEAR(pat.at("ring_groove_dia_mm").get<double>(),  7.00, 0.05);
    EXPECT_NEAR(pat.at("ring_groove_depth_mm").get<double>(), 0.90, 0.05);
    // Bezel rule: counterbore Ø (9.0) > through Ø (4.1)
    EXPECT_GT(pat.at("counterbore_dia_mm").get<double>(),
              pat.at("through_dia_mm").get<double>());
}
