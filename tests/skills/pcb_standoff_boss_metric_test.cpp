// @lat: [[process/test-strategy#pcb_standoff_boss_metric]]
//
// pcb_standoff_boss_metric — boss + tapped hole compound (5 tests).
//
// Cases:
//   1. ApplyAddsBossAndCutsPilot — Δvol > 0 (boss − pilot), faces grow
//   2. ValidateRejectsUnknownThread
//   3. ValidateRejectsZeroBossHeight
//   4. SignatureCarriesCompoundAndPhoneFlags
//   5. RecognizeReplaysHistory

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/pcb_standoff_boss_metric.hpp"

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

skill::pcb_standoff_boss_metric::Input good(int faceId)
{
    skill::pcb_standoff_boss_metric::Input in;
    in.face_id         = faceId;
    in.center_x_mm     = 30.0;
    in.center_y_mm     = 20.0;
    in.boss_height_mm  = 2.0;
    in.thread_size_key = "M2";
    in.thread_depth_mm = 1.4;
    return in;
}

}  // namespace

// ─── 1. Boss adds material, pilot removes — net positive ─────────────────
TEST(SkillPcbStandoffBossMetric, ApplyAddsBossAndCutsPilot)
{
    auto stock = skill::createCuboidStock(60.0, 40.0, 8.0);
    const int topId = findTopFace(*stock);
    ASSERT_GE(topId, 0);

    const double v0 = volumeOf(stock->shape());
    auto in = good(topId);
    auto out = skill::pcb_standoff_boss_metric::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    const double v1 = volumeOf(out.workpiece->shape());

    // Boss (≈ Ø2.4, h 2 mm) > pilot (Ø1.6, h 1.4 mm): net add.
    EXPECT_GT(v1, v0);
    EXPECT_GT(out.workpiece->faceCount(), stock->faceCount());
}

// ─── 2. Validate rejects unknown thread spec ─────────────────────────────
TEST(SkillPcbStandoffBossMetric, ValidateRejectsUnknownThread)
{
    auto stock = skill::createCuboidStock(60.0, 40.0, 8.0);
    const int topId = findTopFace(*stock);
    auto in = good(topId);
    in.thread_size_key = "M99";
    auto r = skill::pcb_standoff_boss_metric::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_THROW(skill::pcb_standoff_boss_metric::apply(*stock, in),
                 skill::SkillError);
}

// ─── 3. Validate rejects zero boss height ────────────────────────────────
TEST(SkillPcbStandoffBossMetric, ValidateRejectsZeroBossHeight)
{
    auto stock = skill::createCuboidStock(60.0, 40.0, 8.0);
    const int topId = findTopFace(*stock);
    auto in = good(topId);
    in.boss_height_mm = 0.0;
    auto r = skill::pcb_standoff_boss_metric::validate(*stock, in);
    EXPECT_FALSE(r.passed);
}

// ─── 4. Signature carries compound + phone flags ─────────────────────────
TEST(SkillPcbStandoffBossMetric, SignatureCarriesCompoundAndPhoneFlags)
{
    auto stock = skill::createCuboidStock(60.0, 40.0, 8.0);
    const int topId = findTopFace(*stock);
    auto in = good(topId);
    auto out = skill::pcb_standoff_boss_metric::apply(*stock, in);

    EXPECT_EQ(out.signature.skill_id, std::string("pcb_standoff_boss_metric"));
    EXPECT_TRUE(out.signature.pattern["is_compound"].get<bool>());
    EXPECT_TRUE(out.signature.pattern["is_phone_feature"].get<bool>());
    EXPECT_EQ(out.signature.pattern["phone_feature_type"].get<std::string>(),
              std::string("pcb_standoff_boss"));
    EXPECT_EQ(out.signature.pattern["thread_size_key"].get<std::string>(),
              std::string("M2"));
    // M2 pilot from local small-screw table = 1.6 mm
    EXPECT_NEAR(out.signature.pattern["pilot_dia_mm"].get<double>(), 1.6, 0.05);
    EXPECT_GT(out.signature.pattern["added_volume_mm3"].get<double>(),
              out.signature.pattern["removed_volume_mm3"].get<double>());
}

// ─── 5. Recognize replays history ────────────────────────────────────────
TEST(SkillPcbStandoffBossMetric, RecognizeReplaysHistory)
{
    auto stock = skill::createCuboidStock(60.0, 40.0, 8.0);
    const int topId = findTopFace(*stock);
    auto in = good(topId);
    auto out = skill::pcb_standoff_boss_metric::apply(*stock, in);

    auto cands = skill::pcb_standoff_boss_metric::recognize(*out.workpiece);
    ASSERT_GE(cands.size(), 1u);
    EXPECT_EQ(cands.front().skill_id,
              std::string("pcb_standoff_boss_metric"));
    EXPECT_GE(cands.front().confidence, 0.7);
}
