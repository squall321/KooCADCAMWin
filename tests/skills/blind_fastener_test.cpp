// @lat: [[process/test-strategy#skill round-trip]]
//
// blind_fastener skill — verifies standard-set sizing, +0.05 mm hole
// allowance, grip-length DFM, through-hole synthesis, and topology
// recognition with confidence falloff.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/blind_fastener.hpp"

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

// ─── 1. Apply makes a through-hole sized = nominal + 0.05 mm ─────────────
TEST(SkillBlindFastener, ApplyMakesThroughHole)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 5.0);

    skill::blind_fastener::Input in;
    in.entry_face     = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm  = 25.0;
    in.position_y_mm  = 25.0;
    in.nominal_dia_mm = 4.0;
    in.grip_length_mm = 5.0;          // within {0.5, 12.5}

    auto out = skill::blind_fastener::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    // hole_dia = 4.05 mm, through 5 mm thick stock.
    const double hd = 4.05;
    const double approx = M_PI * (hd/2.0) * (hd/2.0) * 5.0;
    const double removed = volumeOf(stock->shape()) - volumeOf(out.workpiece->shape());
    EXPECT_NEAR(removed, approx, approx * 0.05);

    EXPECT_EQ(out.signature.skill_id, std::string("blind_fastener"));
    EXPECT_NEAR(out.signature.params["hole_dia_mm"].get<double>(), hd, 1e-9);
    EXPECT_TRUE(out.signature.params["through_hole"].get<bool>());
}

// ─── 2. Standard-set membership & helpers ────────────────────────────────
TEST(SkillBlindFastener, StandardSetAndHelpers)
{
    const auto& set = skill::blind_fastener::standardBlindDiameters();
    EXPECT_EQ(set.size(), 4u);
    EXPECT_NEAR(set[0], 3.2, 1e-9);
    EXPECT_NEAR(set[3], 6.4, 1e-9);

    EXPECT_NEAR(skill::blind_fastener::holeDiameterFor(3.2), 3.25, 1e-9);
    EXPECT_NEAR(skill::blind_fastener::holeDiameterFor(4.8), 4.85, 1e-9);
    EXPECT_EQ  (skill::blind_fastener::holeDiameterFor(5.0), 0.0);     // not in set

    EXPECT_NEAR(skill::blind_fastener::matchStandardDia(3.25, 0.05), 3.2, 1e-9);
    EXPECT_NEAR(skill::blind_fastener::matchStandardDia(4.85, 0.05), 4.8, 1e-9);
    EXPECT_EQ  (skill::blind_fastener::matchStandardDia(7.30, 0.05), 0.0);

    const auto gr = skill::blind_fastener::gripRangeFor(4.8);
    EXPECT_NEAR(gr.min_mm,  0.5, 1e-9);
    EXPECT_NEAR(gr.max_mm, 14.5, 1e-9);
}

// ─── 3. DFM rejects non-standard nominal_dia ─────────────────────────────
TEST(SkillBlindFastener, ValidateRejectsOffsizeDia)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 5.0);

    skill::blind_fastener::Input in;
    in.entry_face     = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm  = 25.0;
    in.position_y_mm  = 25.0;
    in.nominal_dia_mm = 5.0;          // not in {3.2, 4.0, 4.8, 6.4}
    in.grip_length_mm = 5.0;

    auto r = skill::blind_fastener::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-BF-DIA") { found = true; break; }
    EXPECT_TRUE(found);
    EXPECT_THROW(skill::blind_fastener::apply(*stock, in), skill::SkillError);
}

// ─── 4. DFM rejects out-of-range grip_length ─────────────────────────────
TEST(SkillBlindFastener, ValidateRejectsOutOfRangeGrip)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 20.0);

    skill::blind_fastener::Input in;
    in.entry_face     = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm  = 25.0;
    in.position_y_mm  = 25.0;
    in.nominal_dia_mm = 3.2;
    in.grip_length_mm = 18.0;         // > 9.5 mm max for 3.2 mm rivet

    auto r = skill::blind_fastener::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-BF-GRIP") { found = true; break; }
    EXPECT_TRUE(found);
}

// ─── 5. Recognize replays from history ───────────────────────────────────
TEST(SkillBlindFastener, RecognizeReplaysFromHistory)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 5.0);

    skill::blind_fastener::Input in;
    in.entry_face     = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm  = 30.0;
    in.position_y_mm  = 15.0;
    in.nominal_dia_mm = 4.8;
    in.grip_length_mm = 5.0;

    auto out = skill::blind_fastener::apply(*stock, in);
    auto cands = skill::blind_fastener::recognize(*out.workpiece);

    ASSERT_FALSE(cands.empty());
    const auto& c = cands.front();
    EXPECT_GT(c.confidence, 0.9);
    EXPECT_NEAR(c.recovered_params["nominal_dia_mm"].get<double>(), 4.8, 1e-9);
    EXPECT_NEAR(c.recovered_params["hole_dia_mm"].get<double>(),    4.85, 1e-9);
    EXPECT_NEAR(c.recovered_params["grip_length_mm"].get<double>(), 5.0, 1e-9);
}

// ─── 6. Topology fallback matches blind rivet from bare B-rep ────────────
TEST(SkillBlindFastener, TopologyFallbackMatchesStandard)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 5.0);

    skill::blind_fastener::Input in;
    in.entry_face     = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm  = 25.0;
    in.position_y_mm  = 25.0;
    in.nominal_dia_mm = 3.2;
    in.grip_length_mm = 5.0;
    auto out = skill::blind_fastener::apply(*stock, in);

    skill::Workpiece bare(out.workpiece->shape(), "aluminum_6061");
    auto cands = skill::blind_fastener::recognize(bare);
    ASSERT_FALSE(cands.empty());
    EXPECT_NEAR(cands.front().recovered_params["nominal_dia_mm"].get<double>(),
                3.2, 1e-9);
    // Grip guess saturates to bbox extent (5 mm), within published range.
    EXPECT_NEAR(cands.front().recovered_params["grip_length_mm"].get<double>(),
                5.0, 0.05);
}

// ─── 7. Non-standard diameter NOT recognised as blind_fastener ───────────
TEST(SkillBlindFastener, RecognizeRejectsNonStandard)
{
    // A plain 5 mm cuboid stock with no hole shouldn't match anything.
    auto stock = skill::createCuboidStock(50.0, 50.0, 5.0);
    auto cands = skill::blind_fastener::recognize(*stock);
    EXPECT_TRUE(cands.empty());
}
