// @lat: [[process/test-strategy#compound macro]]
//
// bore_and_finish — drill → bore → ream sequence.  Currently the ream
// step is implemented as a final bore_cylindrical pass (see fallback note
// in the skill's .cpp).
//
// Test cases:
//   1. apply produces a hole at the final_diameter with right volume.
//   2. validate rejects final_dia < 1.5 mm (insufficient margin).
//   3. signature carries tool sequence + tolerance class.
//   4. recognize finds the precision bore at confidence ≈ 0.7.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/bore_and_finish.hpp"

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

// ─── 1. apply produces a hole at final_diameter ────────────────────────────
TEST(SkillBoreAndFinish, ApplyCreatesFinishedBore)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 20.0);

    skill::bore_and_finish::Input in;
    in.entry_face       = skill::FaceByNormal{ gp_Dir(0, 0, 1), 5.0, "largest" };
    in.position_x_mm    = 25.0;
    in.position_y_mm    = 25.0;
    in.axis_dir         = gp_Dir(0, 0, -1);
    in.final_diameter_mm = 8.0;
    in.depth_mm         = 10.0;
    in.tolerance_class  = "H7";

    auto out = skill::bore_and_finish::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    // Final hole volume = π × (final/2)² × depth.
    const double approx = M_PI * 4.0 * 4.0 * 10.0;
    const double diff   = volumeOf(stock->shape()) - volumeOf(out.workpiece->shape());
    // Three Boolean passes leave slight overlap rings; loose tolerance.
    EXPECT_NEAR(diff, approx, approx * 0.08);

    EXPECT_EQ(out.signature.skill_id, std::string("bore_and_finish"));
    EXPECT_EQ(out.signature.pattern.at("kind").get<std::string>(),
              std::string("bore_and_finish"));
    EXPECT_EQ(out.signature.pattern.at("tolerance_class").get<std::string>(),
              std::string("H7"));
    EXPECT_NEAR(out.signature.pattern.at("final_diameter_mm").get<double>(),
                8.0, 1e-6);
}

// ─── 2. validate rejects final_dia < 1.5 ───────────────────────────────────
TEST(SkillBoreAndFinish, ValidateRejectsTinyDiameter)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 20.0);

    skill::bore_and_finish::Input in;
    in.entry_face        = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm     = 25.0;
    in.position_y_mm     = 25.0;
    in.final_diameter_mm = 1.0;     // < 1.5 → not enough margin
    in.depth_mm          = 5.0;

    auto r = skill::bore_and_finish::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-MARGIN") { found = true; break; }
    EXPECT_TRUE(found);

    EXPECT_THROW(skill::bore_and_finish::apply(*stock, in), skill::SkillError);
}

// ─── 3. validate rejects negative depth ────────────────────────────────────
TEST(SkillBoreAndFinish, ValidateRejectsBadDepth)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 20.0);
    skill::bore_and_finish::Input in;
    in.entry_face        = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm     = 25.0;
    in.position_y_mm     = 25.0;
    in.final_diameter_mm = 8.0;
    in.depth_mm          = 0.0;

    auto r = skill::bore_and_finish::validate(*stock, in);
    EXPECT_FALSE(r.passed);
}

// ─── 4. Tool sequence is recorded in signature ─────────────────────────────
TEST(SkillBoreAndFinish, SignatureRecordsToolSequence)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 20.0);

    skill::bore_and_finish::Input in;
    in.entry_face        = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm     = 25.0;
    in.position_y_mm     = 25.0;
    in.final_diameter_mm = 10.0;
    in.depth_mm          = 8.0;
    auto out = skill::bore_and_finish::apply(*stock, in);

    const auto& ex = out.signature.tooling.extra;
    ASSERT_TRUE(ex.contains("tool_sequence"));
    const auto& seq = ex.at("tool_sequence");
    EXPECT_EQ(seq.size(), 3u);
    EXPECT_EQ(seq[0].at("tool_type").get<std::string>(), std::string("drill"));
    EXPECT_EQ(seq[1].at("tool_type").get<std::string>(), std::string("boring_bar"));
    EXPECT_EQ(seq[2].at("tool_type").get<std::string>(), std::string("reamer"));
}

// ─── 5. Recognize finds the precision bore at ~0.7 confidence ──────────────
TEST(SkillBoreAndFinish, RecognizeFindsPrecisionBore)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 20.0);
    skill::bore_and_finish::Input in;
    in.entry_face        = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm     = 25.0;
    in.position_y_mm     = 25.0;
    in.final_diameter_mm = 8.0;
    in.depth_mm          = 10.0;
    auto out = skill::bore_and_finish::apply(*stock, in);

    auto cands = skill::bore_and_finish::recognize(*out.workpiece);
    ASSERT_GE(cands.size(), 1u);
    const auto& c = cands.front();
    EXPECT_EQ(c.skill_id, std::string("bore_and_finish"));
    EXPECT_NEAR(c.recovered_params["final_diameter_mm"].get<double>(), 8.0, 1e-3);
    EXPECT_NEAR(c.confidence, 0.7, 0.05);
}
