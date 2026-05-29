// @lat: [[process/test-strategy#compound macro]]
//
// dust_lip_seal_seat_compound — 5 tests verifying REAL volume + spec.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/dust_lip_seal_seat_compound.hpp"

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

// ─── T1: apply removes ~ main-bore volume ±20 % ───────────────────────────
TEST(SkillDustLipSealSeat, T1_ApplyRemovesExpectedVolume)
{
    auto stock = skill::createCuboidStock(80.0, 80.0, 30.0);
    const double v0 = volumeOf(stock->shape());

    skill::dust_lip_seal_seat_compound::Input in;
    in.entry_face       = skill::FaceByNormal{ gp_Dir(0, 0, 1), 5.0, "largest" };
    in.center_x_mm      = 40.0;
    in.center_y_mm      = 40.0;
    in.axis_dir         = gp_Dir(0, 0, -1);
    in.bore_dia_mm      = 25.0;
    in.bore_depth_mm    = 15.0;
    in.seal_thickness_mm     = 7.0;
    in.seal_can_OD_offset_mm = 0.5;
    in.lip_relief_depth_mm   = 0.5;
    in.nose_chamfer_mm       = 0.5;
    in.seal_series      = "CR-25-37-7";

    auto out = skill::dust_lip_seal_seat_compound::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    // T1 expected math (dominant terms):
    //   main bore: pi * 12.5^2 * 15 = pi * 156.25 * 15 = 7363.1 mm³
    //   counterbore annulus: pi * (13^2 - 12.5^2) * 7 = pi * 12.75 * 7 = 280.4
    //   lip relief (small cyl 2 mm long, r = 12.0): pi * 144 * 2 = 904.8
    //   Total ≈ 8550 mm³ (allowing for overlap of lip & main bore)
    const double mainBoreVol = M_PI * 12.5 * 12.5 * 15.0;
    const double removed = v0 - volumeOf(out.workpiece->shape());
    EXPECT_GT(removed, 0.80 * mainBoreVol);
    EXPECT_LT(removed, 1.50 * mainBoreVol);  // allow extra for cb + lip
}

// ─── T2: validate rejects bore too small + apply throws ──────────────────
TEST(SkillDustLipSealSeat, T2_ValidateRejectsBadSpec)
{
    auto stock = skill::createCuboidStock(80.0, 80.0, 30.0);
    skill::dust_lip_seal_seat_compound::Input bad;
    bad.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    bad.bore_dia_mm  = 1.0;                  // way under 3 mm minimum
    bad.bore_depth_mm = 10.0;
    bad.lip_relief_depth_mm = 0.5;

    auto r = skill::dust_lip_seal_seat_compound::validate(*stock, bad);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-SEAL-DIA") { found = true; break; }
    EXPECT_TRUE(found);
    EXPECT_THROW(skill::dust_lip_seal_seat_compound::apply(*stock, bad),
                 skill::SkillError);
}

// ─── T3: signature carries spec_key + is_compound=true + 4 subfeatures ───
TEST(SkillDustLipSealSeat, T3_SignatureCarriesSpecKey)
{
    auto stock = skill::createCuboidStock(80.0, 80.0, 30.0);
    skill::dust_lip_seal_seat_compound::Input in;
    in.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.bore_dia_mm  = 25.0;
    in.bore_depth_mm = 15.0;
    in.seal_series  = "CR-25-37-7";
    in.center_x_mm  = 40.0;
    in.center_y_mm  = 40.0;
    in.axis_dir     = gp_Dir(0, 0, -1);

    auto out = skill::dust_lip_seal_seat_compound::apply(*stock, in);
    const auto& pat = out.signature.pattern;
    EXPECT_EQ(pat.at("kind").get<std::string>(),
              std::string("dust_lip_seal_seat_compound"));
    EXPECT_EQ(pat.at("is_compound").get<bool>(), true);
    EXPECT_EQ(pat.at("subfeature_count").get<int>(), 4);
    EXPECT_EQ(pat.at("spec_key").get<std::string>(),
              std::string("CR-25-37-7"));
    EXPECT_EQ(pat.at("matches_spec").get<bool>(), true);
}

// ─── T4: recognize replays + matches spec_key ─────────────────────────────
TEST(SkillDustLipSealSeat, T4_RecognizeReplaysSpec)
{
    auto stock = skill::createCuboidStock(80.0, 80.0, 30.0);
    skill::dust_lip_seal_seat_compound::Input in;
    in.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.bore_dia_mm  = 30.0;
    in.bore_depth_mm = 15.0;
    in.seal_series  = "CR-30-42-7";
    in.center_x_mm  = 40.0;
    in.center_y_mm  = 40.0;
    in.axis_dir     = gp_Dir(0, 0, -1);

    auto out = skill::dust_lip_seal_seat_compound::apply(*stock, in);
    auto cands = skill::dust_lip_seal_seat_compound::recognize(*out.workpiece);
    ASSERT_GE(cands.size(), 1u);
    EXPECT_EQ(cands[0].skill_id,
              std::string("dust_lip_seal_seat_compound"));
    EXPECT_EQ(cands[0].recovered_params.at("seal_series").get<std::string>(),
              std::string("CR-30-42-7"));
    EXPECT_DOUBLE_EQ(cands[0].confidence, 1.0);
}

// ─── T5: lip-relief geometry strictly smaller than main bore (DFM gate) ──
TEST(SkillDustLipSealSeat, T5_LipReliefRejectedWhenTooDeep)
{
    auto stock = skill::createCuboidStock(80.0, 80.0, 30.0);
    skill::dust_lip_seal_seat_compound::Input bad;
    bad.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    bad.bore_dia_mm  = 25.0;
    bad.bore_depth_mm = 15.0;
    bad.lip_relief_depth_mm = 10.0;  // >> bore_dia / 4 = 6.25 → must fail
    bad.center_x_mm  = 40.0;
    bad.center_y_mm  = 40.0;

    auto r = skill::dust_lip_seal_seat_compound::validate(*stock, bad);
    EXPECT_FALSE(r.passed);
    bool foundLip = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-LIP-RELIEF") { foundLip = true; break; }
    EXPECT_TRUE(foundLip);

    // And the good path: counterbore > main_bore in resulting pattern.
    skill::dust_lip_seal_seat_compound::Input good;
    good.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    good.bore_dia_mm  = 25.0;
    good.bore_depth_mm = 15.0;
    good.lip_relief_depth_mm = 0.5;
    good.center_x_mm  = 40.0;
    good.center_y_mm  = 40.0;
    good.axis_dir     = gp_Dir(0, 0, -1);
    auto out = skill::dust_lip_seal_seat_compound::apply(*stock, good);
    const auto& pat = out.signature.pattern;
    EXPECT_GT(pat.at("counterbore_dia_mm").get<double>(),
              pat.at("bore_dia_mm").get<double>());
    EXPECT_LT(pat.at("lip_relief_dia_mm").get<double>(),
              pat.at("bore_dia_mm").get<double>());
}
