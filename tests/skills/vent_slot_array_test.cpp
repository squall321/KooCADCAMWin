// @lat: [[process/test-strategy#vent_slot_array]]
//
// vent_slot_array — COMPOUND, SPEC-DRIVEN feature (CUTS material).
//
// Test cases:
//   1. apply REMOVES N × slot_length × slot_width × wall_thickness mm³.
//   2. validate rejects unknown spec + free-area < 50 % case + apply throws.
//   3. signature carries spec_key + subfeature_count.
//   4. recognize returns the candidate with the right spec_key (> 50 % conf).
//   5. spec table & free-area math: VS-L pitch 8 mm, width 4 mm yields
//      exactly 0.5 free-area frac → passes; tightening to width 3 mm
//      makes free-area 0.375 < 0.5 → fails DFM-VENT-FA.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/vent_slot_array.hpp"

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

// ─── 1. apply removes the spec'd volume ─────────────────────────────────
TEST(SkillVentSlotArray, ApplyRemovesSlotVolume)
{
    // 100 × 50 × 3 mm plate (wall_thickness = 3).
    auto stock = skill::createCuboidStock(100.0, 50.0, 3.0);
    const double v0 = volumeOf(stock->shape());

    skill::vent_slot_array::Input in;
    in.face        = skill::FaceByNormal{ gp_Dir(0, 0, 1), 5.0, "largest" };
    in.spec_key    = "VS-S";        // 20 × 2 mm slots, pitch 4 mm
    in.slot_count  = 6;
    in.slot_axis   = gp_Dir(1, 0, 0);
    in.center_x_mm = 50.0;
    in.center_y_mm = 25.0;
    in.wall_thickness_mm = 3.0;

    auto out = skill::vent_slot_array::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    const double v1 = volumeOf(out.workpiece->shape());

    // Expected removed volume = N × slot_length × slot_width × wall_thickness
    //                         = 6 × 20 × 2 × 3 = 720 mm³.
    const double approx = 6.0 * 20.0 * 2.0 * 3.0;
    const double removed = v0 - v1;
    EXPECT_NEAR(removed, approx, approx * 0.05)
        << "expected removed volume ≈ " << approx << ", got " << removed;

    EXPECT_EQ(out.signature.skill_id, std::string("vent_slot_array"));
}

// ─── 2. validate rejects bad / underflow cases ──────────────────────────
TEST(SkillVentSlotArray, ValidateRejectsBadInputs)
{
    auto stock = skill::createCuboidStock(100.0, 50.0, 3.0);

    // (a) Unknown spec
    {
        skill::vent_slot_array::Input in;
        in.face       = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
        in.spec_key   = "VS-XL";        // unknown
        in.slot_count = 4;
        auto r = skill::vent_slot_array::validate(*stock, in);
        EXPECT_FALSE(r.passed);
        bool found = false;
        for (const auto& f : r.findings)
            if (f.code == "DFM-VENT-SP") { found = true; break; }
        EXPECT_TRUE(found);
        EXPECT_THROW(skill::vent_slot_array::apply(*stock, in),
                     skill::SkillError);
    }

    // (b) Free-area gate: tight pitch+narrow slots
    //     pitch 6 mm, width 2 mm, 4 slots, length 20 mm.
    //     openArea = 4 × 20 × 2 = 160; envelopeArea = 20 × (3 × 6 + 2) = 400
    //     free_frac = 160/400 = 0.40 < 0.5 → DFM-VENT-FA
    {
        skill::vent_slot_array::Input in;
        in.face            = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
        in.spec_key        = "";
        in.slot_length_mm  = 20.0;
        in.slot_width_mm   = 2.0;
        in.pitch_mm        = 6.0;
        in.slot_count      = 4;
        auto r = skill::vent_slot_array::validate(*stock, in);
        EXPECT_FALSE(r.passed);
        bool found = false;
        for (const auto& f : r.findings)
            if (f.code == "DFM-VENT-FA") { found = true; break; }
        EXPECT_TRUE(found);
        EXPECT_THROW(skill::vent_slot_array::apply(*stock, in),
                     skill::SkillError);
    }
}

// ─── 3. signature carries spec_key + subfeature_count ───────────────────
TEST(SkillVentSlotArray, SignatureCarriesSpec)
{
    auto stock = skill::createCuboidStock(100.0, 80.0, 3.0);

    skill::vent_slot_array::Input in;
    in.face        = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.spec_key    = "VS-M";
    in.slot_count  = 7;
    in.center_x_mm = 50.0;
    in.center_y_mm = 40.0;
    in.wall_thickness_mm = 3.0;
    auto out = skill::vent_slot_array::apply(*stock, in);

    EXPECT_EQ(out.signature.pattern.at("kind").get<std::string>(),
              std::string("vent_slot_array"));
    EXPECT_EQ(out.signature.pattern.at("is_compound").get<bool>(), true);
    EXPECT_EQ(out.signature.pattern.at("subfeature_count").get<int>(), 7);
    EXPECT_EQ(out.signature.pattern.at("spec_key").get<std::string>(),
              std::string("VS-M"));
    EXPECT_NEAR(out.signature.pattern.at("slot_length_mm").get<double>(), 40.0, 1e-6);
    EXPECT_NEAR(out.signature.pattern.at("slot_width_mm").get<double>(),   3.0, 1e-6);
    EXPECT_NEAR(out.signature.pattern.at("pitch_mm").get<double>(),        6.0, 1e-6);
}

// ─── 4. recognize finds spec ────────────────────────────────────────────
TEST(SkillVentSlotArray, RecognizeRecoversSpec)
{
    auto stock = skill::createCuboidStock(100.0, 80.0, 3.0);

    skill::vent_slot_array::Input in;
    in.face        = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.spec_key    = "VS-S";
    in.slot_count  = 4;
    in.center_x_mm = 50.0;
    in.center_y_mm = 40.0;
    in.wall_thickness_mm = 3.0;
    auto out = skill::vent_slot_array::apply(*stock, in);

    auto cands = skill::vent_slot_array::recognize(*out.workpiece);
    ASSERT_GE(cands.size(), 1u);
    EXPECT_GT(cands.front().confidence, 0.5);
    EXPECT_EQ(cands.front().recovered_params["spec_key"].get<std::string>(),
              std::string("VS-S"));
    EXPECT_EQ(cands.front().recovered_params["slot_count"].get<int>(), 4);
}

// ─── 5. spec table & free-area math ─────────────────────────────────────
TEST(SkillVentSlotArray, SpecTableCorrectness)
{
    auto s = skill::vent_slot_array::slotSpecFor("VS-S");
    EXPECT_NEAR(s.slot_length_mm, 20.0, 1e-9);
    EXPECT_NEAR(s.slot_width_mm,   2.0, 1e-9);
    EXPECT_NEAR(s.pitch_mm,        4.0, 1e-9);
    // VS-S free-area frac = 2/4 = 0.5 (at the threshold; passes).

    auto m = skill::vent_slot_array::slotSpecFor("VS-M");
    EXPECT_NEAR(m.slot_length_mm, 40.0, 1e-9);
    EXPECT_NEAR(m.slot_width_mm,   3.0, 1e-9);

    auto l = skill::vent_slot_array::slotSpecFor("VS-L");
    EXPECT_NEAR(l.pitch_mm,        8.0, 1e-9);
    EXPECT_NEAR(l.slot_width_mm,   4.0, 1e-9);
    // VS-L free-area frac = 4/8 = 0.5 → passes.

    auto bad = skill::vent_slot_array::slotSpecFor("VS-XL");
    EXPECT_NEAR(bad.pitch_mm, 0.0, 1e-9);
}
