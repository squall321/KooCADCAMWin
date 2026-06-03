// @lat: [[process/test-strategy#speaker_grille_micro_array]]
//
// speaker_grille_micro_array — recess frame + micro-hole array (5 tests).
//
// Cases:
//   1. ApplyHexPatternProducesPositiveHoleCount
//   2. ApplyGridPatternProducesPositiveHoleCount
//   3. ValidateRejectsTooSmallHole          (< 0.3 mm)
//   4. ValidateRejectsBadPattern            (not hex/grid)
//   5. SignatureCarriesHoleCountAndPattern

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/speaker_grille_micro_array.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

using namespace koocadcam;

namespace {

double volumeOf(const TopoDS_Shape& s)
{
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}

int findTopFace(const skill::Workpiece& wp)
{
    int bestId = -1; double bestArea = 0.0;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFacePlanar(i)) continue;
        gp_Dir n; try { n = wp.faceNormal(i); } catch (...) { continue; }
        if (n.Z() < 0.9) continue;
        const double a = wp.faceArea(i);
        if (a > bestArea) { bestArea = a; bestId = i; }
    }
    return bestId;
}

skill::speaker_grille_micro_array::Input good(int faceId, const std::string& pat)
{
    skill::speaker_grille_micro_array::Input in;
    in.face_id          = faceId;
    in.center_x_mm      = 30.0;
    in.center_y_mm      = 20.0;
    in.grille_length_mm = 8.0;
    in.grille_width_mm  = 3.0;
    in.hole_dia_mm      = 0.8;
    in.hole_pitch_mm    = 1.2;
    in.pattern          = pat;
    return in;
}

}  // namespace

// ─── 1. Hex pattern produces > 0 holes ───────────────────────────────────
TEST(SkillSpeakerGrilleMicroArray, ApplyHexPatternProducesPositiveHoleCount)
{
    auto stock = skill::createCuboidStock(60.0, 40.0, 1.5);
    const int topId = findTopFace(*stock);
    ASSERT_GE(topId, 0);

    const double v0 = volumeOf(stock->shape());
    auto in = good(topId, "hex");
    auto out = skill::speaker_grille_micro_array::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    const double v1 = volumeOf(out.workpiece->shape());

    EXPECT_LT(v1, v0);
    EXPECT_GT(out.signature.pattern["hole_count"].get<int>(), 0);
    EXPECT_EQ(out.signature.pattern["pattern_type"].get<std::string>(),
              std::string("hex"));
}

// ─── 2. Grid pattern produces > 0 holes ──────────────────────────────────
TEST(SkillSpeakerGrilleMicroArray, ApplyGridPatternProducesPositiveHoleCount)
{
    auto stock = skill::createCuboidStock(60.0, 40.0, 1.5);
    const int topId = findTopFace(*stock);
    auto in = good(topId, "grid");
    auto out = skill::speaker_grille_micro_array::apply(*stock, in);
    EXPECT_GT(out.signature.pattern["hole_count"].get<int>(), 0);
    EXPECT_EQ(out.signature.pattern["pattern_type"].get<std::string>(),
              std::string("grid"));
}

// ─── 3. Validate rejects sub-0.3 mm hole ─────────────────────────────────
TEST(SkillSpeakerGrilleMicroArray, ValidateRejectsTooSmallHole)
{
    auto stock = skill::createCuboidStock(60.0, 40.0, 1.5);
    const int topId = findTopFace(*stock);
    auto in = good(topId, "hex");
    in.hole_dia_mm   = 0.2;
    in.hole_pitch_mm = 0.7;
    auto r = skill::speaker_grille_micro_array::validate(*stock, in);
    EXPECT_FALSE(r.passed);
}

// ─── 4. Validate rejects unknown pattern ─────────────────────────────────
TEST(SkillSpeakerGrilleMicroArray, ValidateRejectsBadPattern)
{
    auto stock = skill::createCuboidStock(60.0, 40.0, 1.5);
    const int topId = findTopFace(*stock);
    auto in = good(topId, "spiral");
    auto r = skill::speaker_grille_micro_array::validate(*stock, in);
    EXPECT_FALSE(r.passed);
}

// ─── 5. Signature carries hole count + pattern ───────────────────────────
TEST(SkillSpeakerGrilleMicroArray, SignatureCarriesHoleCountAndPattern)
{
    auto stock = skill::createCuboidStock(60.0, 40.0, 1.5);
    const int topId = findTopFace(*stock);
    auto in = good(topId, "hex");
    auto out = skill::speaker_grille_micro_array::apply(*stock, in);

    EXPECT_EQ(out.signature.skill_id,
              std::string("speaker_grille_micro_array"));
    EXPECT_TRUE(out.signature.pattern["is_compound"].get<bool>());
    EXPECT_TRUE(out.signature.pattern["is_phone_feature"].get<bool>());
    EXPECT_EQ(out.signature.pattern["phone_feature_type"].get<std::string>(),
              std::string("speaker_grille"));
    EXPECT_GT(out.signature.pattern["hole_count"].get<int>(), 0);
    EXPECT_NEAR(out.signature.pattern["hole_dia_mm"].get<double>(), 0.8, 1e-6);
}
