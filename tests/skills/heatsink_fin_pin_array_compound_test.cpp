// @lat: [[process/test-strategy#heatsink_fin_pin_array_compound]]

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/heatsink_fin_pin_array_compound.hpp"

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

// ─── 1. apply ADDS the right pin-fin volume (grid) ──────────────────────
TEST(SkillHeatsinkPinArray, ApplyAddsPinVolumeGrid)
{
    auto stock = skill::createCuboidStock(80.0, 80.0, 5.0);
    const double v0 = volumeOf(stock->shape());

    skill::heatsink_fin_pin_array_compound::Input in;
    in.face_id        = 0;
    in.fin_pitch_x_mm = 5.0;
    in.fin_pitch_y_mm = 5.0;
    in.fin_count_x    = 6;
    in.fin_count_y    = 6;
    in.pin_dia_mm     = 2.0;
    in.pin_height_mm  = 8.0;
    in.pattern        = "grid";
    in.origin_x_mm    = 10.0;
    in.origin_y_mm    = 10.0;

    auto out = skill::heatsink_fin_pin_array_compound::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    const double v1 = volumeOf(out.workpiece->shape());
    const double expected =
        6.0 * 6.0 * kPi * 1.0 * 1.0 * 8.0;  // 36 × π × r² × h
    const double added = v1 - v0;
    EXPECT_NEAR(added, expected, expected * 0.10)
        << "expected added pin volume ≈ " << expected << ", got " << added;

    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_EQ(out.signature.skill_id,
              std::string("heatsink_fin_pin_array_compound"));
}

// ─── 2. validate rejects bad pin dia, pattern, AR ───────────────────────
TEST(SkillHeatsinkPinArray, ValidateRejectsBadInputs)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 5.0);

    {  // (a) pin too thin
        skill::heatsink_fin_pin_array_compound::Input in;
        in.fin_pitch_x_mm = 5.0; in.fin_pitch_y_mm = 5.0;
        in.fin_count_x = 3;      in.fin_count_y = 3;
        in.pin_dia_mm = 0.4;     in.pin_height_mm = 4.0;
        in.pattern    = "grid";
        auto r = skill::heatsink_fin_pin_array_compound::validate(*stock, in);
        EXPECT_FALSE(r.passed);
        bool found = false;
        for (const auto& f : r.findings)
            if (f.code == "DFM-PIN-DIA") found = true;
        EXPECT_TRUE(found);
        EXPECT_THROW(skill::heatsink_fin_pin_array_compound::apply(*stock, in),
                     skill::SkillError);
    }
    {  // (b) bad pattern
        skill::heatsink_fin_pin_array_compound::Input in;
        in.fin_pitch_x_mm = 5.0; in.fin_pitch_y_mm = 5.0;
        in.fin_count_x = 3;      in.fin_count_y = 3;
        in.pin_dia_mm = 2.0;     in.pin_height_mm = 4.0;
        in.pattern    = "triangle";
        auto r = skill::heatsink_fin_pin_array_compound::validate(*stock, in);
        EXPECT_FALSE(r.passed);
        bool found = false;
        for (const auto& f : r.findings)
            if (f.code == "DFM-PIN-LAYOUT") found = true;
        EXPECT_TRUE(found);
    }
    {  // (c) AR too high
        skill::heatsink_fin_pin_array_compound::Input in;
        in.fin_pitch_x_mm = 5.0; in.fin_pitch_y_mm = 5.0;
        in.fin_count_x = 3;      in.fin_count_y = 3;
        in.pin_dia_mm = 1.0;     in.pin_height_mm = 20.0;   // AR=20
        in.pattern    = "grid";
        auto r = skill::heatsink_fin_pin_array_compound::validate(*stock, in);
        EXPECT_FALSE(r.passed);
        bool found = false;
        for (const auto& f : r.findings)
            if (f.code == "DFM-PIN-AR") found = true;
        EXPECT_TRUE(found);
    }
}

// ─── 3. signature pattern carries electronics_feature_type + counts ─────
TEST(SkillHeatsinkPinArray, SignatureCarriesCompoundFlags)
{
    auto stock = skill::createCuboidStock(80.0, 80.0, 5.0);
    skill::heatsink_fin_pin_array_compound::Input in;
    in.fin_pitch_x_mm = 6.0; in.fin_pitch_y_mm = 6.0;
    in.fin_count_x = 5;      in.fin_count_y = 4;
    in.pin_dia_mm = 2.5;     in.pin_height_mm = 10.0;
    in.pattern    = "hex";
    in.origin_x_mm = 10.0;   in.origin_y_mm = 10.0;

    auto out = skill::heatsink_fin_pin_array_compound::apply(*stock, in);
    EXPECT_TRUE(out.signature.pattern.at("is_compound").get<bool>());
    EXPECT_EQ(out.signature.pattern.at("electronics_feature_type").get<std::string>(),
              std::string("pin_fin_heat_sink"));
    EXPECT_EQ(out.signature.pattern.at("pattern_layout").get<std::string>(),
              std::string("hex"));
    EXPECT_EQ(out.signature.pattern.at("subfeature_count").get<int>(), 20);
    EXPECT_EQ(out.signature.pattern.at("pin_count_x").get<int>(), 5);
    EXPECT_EQ(out.signature.pattern.at("pin_count_y").get<int>(), 4);
}

// ─── 4. recognize replays metadata ───────────────────────────────────────
TEST(SkillHeatsinkPinArray, RecognizeFromMetadata)
{
    auto stock = skill::createCuboidStock(80.0, 80.0, 5.0);
    skill::heatsink_fin_pin_array_compound::Input in;
    in.fin_pitch_x_mm = 5.0; in.fin_pitch_y_mm = 5.0;
    in.fin_count_x = 4;      in.fin_count_y = 4;
    in.pin_dia_mm = 2.0;     in.pin_height_mm = 6.0;
    in.pattern    = "grid";
    in.origin_x_mm = 10.0;   in.origin_y_mm = 10.0;
    auto out = skill::heatsink_fin_pin_array_compound::apply(*stock, in);

    auto cands = skill::heatsink_fin_pin_array_compound::recognize(*out.workpiece);
    ASSERT_GE(cands.size(), 1u);
    EXPECT_GE(cands.front().confidence, 0.9);
    EXPECT_EQ(cands.front().recovered_params.at("pattern").get<std::string>(),
              std::string("grid"));
}

// ─── 5. hex pattern produces same pin count as grid ──────────────────────
TEST(SkillHeatsinkPinArray, HexPatternMatchesPinCount)
{
    auto stock = skill::createCuboidStock(80.0, 80.0, 5.0);
    skill::heatsink_fin_pin_array_compound::Input in;
    in.fin_pitch_x_mm = 5.0; in.fin_pitch_y_mm = 5.0;
    in.fin_count_x = 5;      in.fin_count_y = 4;
    in.pin_dia_mm = 1.5;     in.pin_height_mm = 5.0;
    in.pattern    = "hex";
    in.origin_x_mm = 10.0;   in.origin_y_mm = 10.0;
    auto out = skill::heatsink_fin_pin_array_compound::apply(*stock, in);
    EXPECT_EQ(out.signature.pattern.at("subfeature_count").get<int>(), 20);
}
