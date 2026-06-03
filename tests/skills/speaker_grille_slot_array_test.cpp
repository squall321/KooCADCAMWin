// @lat: [[process/test-strategy#speaker_grille_slot_array]]
//
// speaker_grille_slot_array — phone speaker slot array (5 tests).
//
// Cases:
//   1. ApplyRemovesVolumeMatchesNxSlotVolume — Δvol ≈ N × L × W × D ±20%
//   2. ValidateRejectsZeroCount
//   3. ValidateRejectsPitchOverlap
//   4. SignatureCarriesPatternFlag
//   5. RecognizeReplaysHistory

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/speaker_grille_slot_array.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

#include <cmath>

using namespace koocadcam;

namespace {

double volumeOf(const TopoDS_Shape& s)
{
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}

int findTopFace(const skill::Workpiece& wp)
{
    int    bestId   = -1;
    double bestArea = 0.0;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFacePlanar(i)) continue;
        gp_Dir n; try { n = wp.faceNormal(i); } catch (...) { continue; }
        if (n.Z() < 0.9) continue;
        const double a = wp.faceArea(i);
        if (a > bestArea) { bestArea = a; bestId = i; }
    }
    return bestId;
}

skill::speaker_grille_slot_array::Input good(int faceId)
{
    skill::speaker_grille_slot_array::Input in;
    in.face_id        = faceId;
    in.center_x_mm    = 30.0;
    in.center_y_mm    = 20.0;
    in.slot_count     = 6;
    in.slot_length_mm = 4.0;
    in.slot_width_mm  = 0.8;
    in.pitch_mm       = 1.5;
    in.depth_through  = 1.0;
    return in;
}

}  // namespace

// ─── 1. Δvol ≈ N × slot volume ±20% ──────────────────────────────────────
TEST(SkillSpeakerGrilleSlotArray, ApplyRemovesVolumeMatchesNxSlotVolume)
{
    auto stock = skill::createCuboidStock(60.0, 40.0, 10.0);
    const int topId = findTopFace(*stock);
    ASSERT_GE(topId, 0);

    auto in = good(topId);
    const double v0 = volumeOf(stock->shape());
    auto out = skill::speaker_grille_slot_array::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    const double v1 = volumeOf(out.workpiece->shape());
    const double actual = v0 - v1;

    const double per = in.slot_length_mm * in.slot_width_mm * in.depth_through;
    const double expected = per * in.slot_count;
    EXPECT_NEAR(actual, expected, expected * 0.20)
        << "actual=" << actual << " expected=" << expected;
}

// ─── 2. Validate rejects slot_count = 0 ──────────────────────────────────
TEST(SkillSpeakerGrilleSlotArray, ValidateRejectsZeroCount)
{
    auto stock = skill::createCuboidStock(60.0, 40.0, 10.0);
    const int topId = findTopFace(*stock);
    auto in = good(topId);
    in.slot_count = 0;
    auto r = skill::speaker_grille_slot_array::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_THROW(skill::speaker_grille_slot_array::apply(*stock, in),
                 skill::SkillError);
}

// ─── 3. Validate rejects pitch ≤ slot_width ──────────────────────────────
TEST(SkillSpeakerGrilleSlotArray, ValidateRejectsPitchOverlap)
{
    auto stock = skill::createCuboidStock(60.0, 40.0, 10.0);
    const int topId = findTopFace(*stock);
    auto in = good(topId);
    in.pitch_mm = in.slot_width_mm;  // exactly equal — would overlap edge-to-edge
    auto r = skill::speaker_grille_slot_array::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool foundCode = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-SLOT-OVERLAP") foundCode = true;
    EXPECT_TRUE(foundCode);
}

// ─── 4. Signature carries periodic-pattern flag ──────────────────────────
TEST(SkillSpeakerGrilleSlotArray, SignatureCarriesPatternFlag)
{
    auto stock = skill::createCuboidStock(60.0, 40.0, 10.0);
    const int topId = findTopFace(*stock);
    auto in = good(topId);
    auto out = skill::speaker_grille_slot_array::apply(*stock, in);

    EXPECT_EQ(out.signature.skill_id, std::string("speaker_grille_slot_array"));
    EXPECT_TRUE(out.signature.pattern["is_compound"].get<bool>());
    EXPECT_TRUE(out.signature.pattern["is_phone_feature"].get<bool>());
    EXPECT_TRUE(out.signature.pattern["is_periodic_pattern"].get<bool>());
    EXPECT_EQ(out.signature.pattern["slot_count"].get<int>(), in.slot_count);
}

// ─── 5. Recognize replays history ────────────────────────────────────────
TEST(SkillSpeakerGrilleSlotArray, RecognizeReplaysHistory)
{
    auto stock = skill::createCuboidStock(60.0, 40.0, 10.0);
    const int topId = findTopFace(*stock);
    auto in = good(topId);
    auto out = skill::speaker_grille_slot_array::apply(*stock, in);

    auto cands = skill::speaker_grille_slot_array::recognize(*out.workpiece);
    ASSERT_GE(cands.size(), 1u);
    EXPECT_EQ(cands.front().skill_id,
              std::string("speaker_grille_slot_array"));
    EXPECT_GE(cands.front().confidence, 0.9);
}
