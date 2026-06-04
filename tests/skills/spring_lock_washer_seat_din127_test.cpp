// @lat: [[process/test-strategy#compound macro — spring_lock_washer_seat_din127]]

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/_retaining_rings.hpp"
#include "skills/spring_lock_washer_seat_din127.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

#include <cmath>

using namespace koocadcam;

namespace {
double volumeOf(const TopoDS_Shape& s) {
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}
constexpr double kClearance_mm = 0.4;
}

// ─── T1: apply cuts ~ seat volume (M6 → washer OD 11.8) ──────────────────
TEST(SkillSpringLockWasherSeatDin127, T1_ApplyRemovesExpectedVolume)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    const double v0 = volumeOf(stock->shape());

    skill::spring_lock_washer_seat_din127::Input in;
    in.face_id         = skill::FaceByNormal{ gp_Dir(0,0,1), 5.0, "largest" };
    in.center_x_mm     = 20.0;
    in.center_y_mm     = 20.0;
    in.axis_dir        = gp_Dir(0,0,-1);
    in.thread_size_key = "M6";
    in.depth_extra_mm  = 0.5;

    auto out = skill::spring_lock_washer_seat_din127::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    const auto* spec = skill::retaining_rings::findDin127("M6");
    ASSERT_NE(spec, nullptr);
    const double seatR     = (spec->outer_dia_mm + kClearance_mm) / 2.0;
    const double seatDepth = spec->thickness_mm + 0.5;
    const double expected  = M_PI * seatR * seatR * seatDepth;

    const double removed = v0 - volumeOf(out.workpiece->shape());
    EXPECT_GT(removed, 0.80 * expected);
    EXPECT_LT(removed, 1.20 * expected);
}

// ─── T2: validate rejects unknown thread size ─────────────────────────────
TEST(SkillSpringLockWasherSeatDin127, T2_ValidateRejectsUnknownSpec)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    skill::spring_lock_washer_seat_din127::Input bad;
    bad.face_id         = skill::FaceByNormal{ gp_Dir(0,0,1), 5.0, "largest" };
    bad.center_x_mm     = 20.0;
    bad.center_y_mm     = 20.0;
    bad.thread_size_key = "M99";   // not in DIN 127 table

    auto r = skill::spring_lock_washer_seat_din127::validate(*stock, bad);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-DIN127") { found = true; break; }
    EXPECT_TRUE(found);

    EXPECT_THROW(skill::spring_lock_washer_seat_din127::apply(*stock, bad),
                 skill::SkillError);
}

// ─── T3: signature pattern ────────────────────────────────────────────────
TEST(SkillSpringLockWasherSeatDin127, T3_SignaturePattern)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    skill::spring_lock_washer_seat_din127::Input in;
    in.face_id         = skill::FaceByNormal{ gp_Dir(0,0,1), 5.0, "largest" };
    in.center_x_mm     = 20.0;
    in.center_y_mm     = 20.0;
    in.axis_dir        = gp_Dir(0,0,-1);
    in.thread_size_key = "M8";
    in.depth_extra_mm  = 0.5;

    auto out = skill::spring_lock_washer_seat_din127::apply(*stock, in);
    const auto& pat = out.signature.pattern;
    EXPECT_EQ(pat.at("kind").get<std::string>(),
              std::string("spring_lock_washer_seat_din127"));
    EXPECT_EQ(pat.at("is_compound").get<bool>(), true);
    EXPECT_EQ(pat.at("ring_standard").get<std::string>(),
              std::string("DIN 127"));
    EXPECT_EQ(pat.at("ring_size_key").get<std::string>(), std::string("M8"));
    EXPECT_GT(pat.at("ring_volume_mm3").get<double>(), 0.0);
}

// ─── T4: recognize replays via metadata ───────────────────────────────────
TEST(SkillSpringLockWasherSeatDin127, T4_RecognizeReplays)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    skill::spring_lock_washer_seat_din127::Input in;
    in.face_id         = skill::FaceByNormal{ gp_Dir(0,0,1), 5.0, "largest" };
    in.center_x_mm     = 20.0;
    in.center_y_mm     = 20.0;
    in.axis_dir        = gp_Dir(0,0,-1);
    in.thread_size_key = "M5";
    in.depth_extra_mm  = 0.3;

    auto out = skill::spring_lock_washer_seat_din127::apply(*stock, in);
    auto cands = skill::spring_lock_washer_seat_din127::recognize(*out.workpiece);
    ASSERT_GE(cands.size(), 1u);
    EXPECT_EQ(cands[0].skill_id,
              std::string("spring_lock_washer_seat_din127"));
    EXPECT_EQ(cands[0].recovered_params.at("thread_size_key").get<std::string>(),
              std::string("M5"));
    EXPECT_DOUBLE_EQ(cands[0].confidence, 1.0);
}

// ─── T5: central table values match DIN 127 ─────────────────────────────
TEST(SkillSpringLockWasherSeatDin127, T5_CentralTable)
{
    EXPECT_EQ(skill::retaining_rings::kDin127.size(), 9u);
    const auto* m6 = skill::retaining_rings::findDin127("M6");
    ASSERT_NE(m6, nullptr);
    EXPECT_NEAR(m6->inner_dia_mm,   6.1,  1e-6);
    EXPECT_NEAR(m6->outer_dia_mm,  11.8,  1e-6);
    EXPECT_NEAR(m6->thickness_mm,   1.6,  1e-6);
    EXPECT_NEAR(m6->gap_height_mm,  2.0,  1e-6);
    EXPECT_EQ(skill::retaining_rings::findDin127("M99"), nullptr);
}
