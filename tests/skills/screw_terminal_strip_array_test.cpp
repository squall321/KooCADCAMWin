// @lat: [[process/test-strategy#screw_terminal_strip_array]]

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/screw_terminal_strip_array.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

#include <cmath>

using namespace koocadcam;

namespace {
double volumeOf(const TopoDS_Shape& s) {
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}
constexpr double kPi = 3.14159265358979323846;
}  // namespace

// ─── 1. apply REMOVES per-position drill volume ─────────────────────────
TEST(SkillScrewTerminal, ApplyRemovesHoleVolume)
{
    auto stock = skill::createCuboidStock(100.0, 30.0, 5.0);
    const double v0 = volumeOf(stock->shape());

    skill::screw_terminal_strip_array::Input in;
    in.face_id           = 0;
    in.strip_origin_x_mm = 10.0;
    in.strip_origin_y_mm = 15.0;
    in.position_count    = 8;
    in.position_pitch_mm = 5.08;             // Phoenix MKDS 0.2"
    in.screw_thread_size = "M3";             // pilot dia 2.5 mm
    in.strip_length_mm   = 7 * 5.08 + 5.0;   // fits 8 positions

    auto out = skill::screw_terminal_strip_array::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    const double v1 = volumeOf(out.workpiece->shape());

    const double perHole = kPi * 1.25 * 1.25 * 5.0;   // r=1.25 mm, h=5 mm
    const double expected = 8.0 * perHole;
    EXPECT_NEAR(v0 - v1, expected, expected * 0.10)
        << "expected " << expected << ", got " << v0 - v1;

    EXPECT_EQ(out.signature.skill_id,
              std::string("screw_terminal_strip_array"));
}

// ─── 2. validate rejects bad pitch + bad thread key ─────────────────────
TEST(SkillScrewTerminal, ValidateRejectsBadInputs)
{
    auto stock = skill::createCuboidStock(100.0, 30.0, 5.0);

    {   // (a) pitch too tight (< 4 mm)
        skill::screw_terminal_strip_array::Input in;
        in.strip_origin_x_mm = 5.0;
        in.position_count = 5; in.position_pitch_mm = 3.0;
        in.screw_thread_size = "M3"; in.strip_length_mm = 30.0;
        auto r = skill::screw_terminal_strip_array::validate(*stock, in);
        EXPECT_FALSE(r.passed);
        bool found = false;
        for (const auto& f : r.findings)
            if (f.code == "DFM-STRIP-PITCH") found = true;
        EXPECT_TRUE(found);
        EXPECT_THROW(skill::screw_terminal_strip_array::apply(*stock, in),
                     skill::SkillError);
    }
    {   // (b) unknown thread key
        skill::screw_terminal_strip_array::Input in;
        in.strip_origin_x_mm = 5.0;
        in.position_count = 5; in.position_pitch_mm = 5.08;
        in.screw_thread_size = "M99"; in.strip_length_mm = 30.0;
        auto r = skill::screw_terminal_strip_array::validate(*stock, in);
        EXPECT_FALSE(r.passed);
        bool found = false;
        for (const auto& f : r.findings)
            if (f.code == "DFM-THREAD-KEY") found = true;
        EXPECT_TRUE(found);
    }
}

// ─── 3. signature pattern carries compound + counts ─────────────────────
TEST(SkillScrewTerminal, SignatureCarriesCompound)
{
    auto stock = skill::createCuboidStock(100.0, 30.0, 5.0);
    skill::screw_terminal_strip_array::Input in;
    in.strip_origin_x_mm = 10.0; in.strip_origin_y_mm = 15.0;
    in.position_count = 6; in.position_pitch_mm = 7.5;
    in.screw_thread_size = "M3"; in.strip_length_mm = 50.0;
    auto out = skill::screw_terminal_strip_array::apply(*stock, in);
    EXPECT_TRUE(out.signature.pattern.at("is_compound").get<bool>());
    EXPECT_EQ(out.signature.pattern.at("electronics_feature_type").get<std::string>(),
              std::string("screw_terminal_strip"));
    EXPECT_EQ(out.signature.pattern.at("position_count").get<int>(), 6);
    EXPECT_EQ(out.signature.pattern.at("subfeature_count").get<int>(), 6);
    EXPECT_NEAR(out.signature.pattern.at("position_pitch_mm").get<double>(),
                7.5, 1e-6);
}

// ─── 4. recognize from metadata ──────────────────────────────────────────
TEST(SkillScrewTerminal, RecognizeFromMetadata)
{
    auto stock = skill::createCuboidStock(100.0, 30.0, 5.0);
    skill::screw_terminal_strip_array::Input in;
    in.strip_origin_x_mm = 10.0; in.strip_origin_y_mm = 15.0;
    in.position_count = 5; in.position_pitch_mm = 10.16;
    in.screw_thread_size = "M4"; in.strip_length_mm = 50.0;
    auto out = skill::screw_terminal_strip_array::apply(*stock, in);

    auto cands = skill::screw_terminal_strip_array::recognize(*out.workpiece);
    ASSERT_GE(cands.size(), 1u);
    EXPECT_GE(cands.front().confidence, 0.9);
    EXPECT_EQ(cands.front().recovered_params.at("position_count").get<int>(), 5);
}

// ─── 5. all three standard pitches build cleanly ────────────────────────
TEST(SkillScrewTerminal, StandardPitchesBuild)
{
    for (double pitch : {5.08, 7.5, 10.16}) {
        auto stock = skill::createCuboidStock(150.0, 30.0, 5.0);
        skill::screw_terminal_strip_array::Input in;
        in.strip_origin_x_mm = 10.0; in.strip_origin_y_mm = 15.0;
        in.position_count = 4; in.position_pitch_mm = pitch;
        in.screw_thread_size = "M3"; in.strip_length_mm = 4 * pitch + 5.0;
        auto out = skill::screw_terminal_strip_array::apply(*stock, in);
        EXPECT_FALSE(out.workpiece->shape().IsNull()) << "pitch=" << pitch;
    }
}
