// @lat: [[process/test-strategy#skill round-trip]]
//
// dot_peen_mark skill — 5×7 dot-matrix glyph rendering tests.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/dot_peen_mark.hpp"

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
}  // namespace

// ── 1. Apply punches a matrix of dots ────────────────────────────────────
TEST(SkillDotPeenMark, ApplyPunchesDots)
{
    auto stock = skill::createCuboidStock(60.0, 40.0, 10.0);
    const double volBefore = volumeOf(stock->shape());

    skill::dot_peen_mark::Input in;
    in.entry_face     = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm  = 5.0;
    in.position_y_mm  = 15.0;
    in.direction_xy   = gp_Dir(1, 0, 0);
    in.text           = "OK";
    in.font_size_mm   = 4.0;
    in.dot_dia_mm     = 0.3;
    in.dot_depth_mm   = 0.2;
    in.dot_spacing_mm = 0.5;

    auto out = skill::dot_peen_mark::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    const double removed = volBefore - volumeOf(out.workpiece->shape());
    EXPECT_GT(removed, 0.0);

    // dot_count must be sensible: < (kNCols * kNRows * text.size()) = 70.
    const int dotCount = out.signature.pattern["dot_count"].get<int>();
    EXPECT_GT(dotCount, 0);
    EXPECT_LE(dotCount, 5 * 7 * 2);

    EXPECT_EQ(out.signature.skill_id, std::string("dot_peen_mark"));
    EXPECT_EQ(out.signature.pattern["geometry"].get<std::string>(), "dot_matrix");
    EXPECT_EQ(out.signature.tooling.tool_type, std::string("peen_stylus"));
}

// ── 2. DFM-PEEN-DIA rejects too-narrow stylus ────────────────────────────
TEST(SkillDotPeenMark, ValidateRejectsTooSmallDot)
{
    auto stock = skill::createCuboidStock(50.0, 40.0, 10.0);

    skill::dot_peen_mark::Input in;
    in.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.text         = "X";
    in.font_size_mm = 3.0;
    in.dot_dia_mm   = 0.05;  // < 0.1
    in.dot_depth_mm = 0.1;

    auto r = skill::dot_peen_mark::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-PEEN-DIA") { found = true; break; }
    EXPECT_TRUE(found);
    EXPECT_THROW(skill::dot_peen_mark::apply(*stock, in), skill::SkillError);
}

// ── 3. DFM-PEEN-DEPTH rejects too-shallow dot ────────────────────────────
TEST(SkillDotPeenMark, ValidateRejectsTooShallow)
{
    auto stock = skill::createCuboidStock(50.0, 40.0, 10.0);

    skill::dot_peen_mark::Input in;
    in.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.text         = "X";
    in.font_size_mm = 3.0;
    in.dot_dia_mm   = 0.3;
    in.dot_depth_mm = 0.01;  // < 0.05

    auto r = skill::dot_peen_mark::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-PEEN-DEPTH") { found = true; break; }
    EXPECT_TRUE(found);
}

// ── 4. Empty text rejected ───────────────────────────────────────────────
TEST(SkillDotPeenMark, ValidateRejectsEmptyText)
{
    auto stock = skill::createCuboidStock(50.0, 40.0, 10.0);

    skill::dot_peen_mark::Input in;
    in.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.text         = "";
    in.font_size_mm = 3.0;
    in.dot_dia_mm   = 0.3;
    in.dot_depth_mm = 0.2;

    auto r = skill::dot_peen_mark::validate(*stock, in);
    EXPECT_FALSE(r.passed);
}

// ── 5. Recognize replays metadata ────────────────────────────────────────
TEST(SkillDotPeenMark, RecognizeReplaysMetadata)
{
    auto stock = skill::createCuboidStock(60.0, 40.0, 10.0);

    skill::dot_peen_mark::Input in;
    in.entry_face     = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm  = 10.0;
    in.position_y_mm  = 20.0;
    in.text           = "AB1";
    in.font_size_mm   = 3.5;
    in.dot_dia_mm     = 0.4;
    in.dot_depth_mm   = 0.25;
    in.dot_spacing_mm = 0.6;

    auto out = skill::dot_peen_mark::apply(*stock, in);
    auto cands = skill::dot_peen_mark::recognize(*out.workpiece);

    ASSERT_FALSE(cands.empty());
    EXPECT_EQ(cands[0].skill_id, std::string("dot_peen_mark"));
    EXPECT_EQ(cands[0].recovered_params["text"], "AB1");
    EXPECT_NEAR(cands[0].recovered_params["dot_dia_mm"].get<double>(), 0.4, 1e-6);
    EXPECT_NEAR(cands[0].recovered_params["dot_depth_mm"].get<double>(), 0.25, 1e-6);
    EXPECT_GT(cands[0].confidence, 0.9);
}

// ── 6. Multi-character text yields more dots than single ─────────────────
TEST(SkillDotPeenMark, MoreCharsMoreDots)
{
    auto stock = skill::createCuboidStock(80.0, 40.0, 10.0);

    skill::dot_peen_mark::Input base;
    base.entry_face     = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    base.position_x_mm  = 5.0;
    base.position_y_mm  = 20.0;
    base.font_size_mm   = 3.0;
    base.dot_dia_mm     = 0.3;
    base.dot_depth_mm   = 0.2;
    base.dot_spacing_mm = 0.5;

    auto inOne = base;  inOne.text = "X";
    auto outOne = skill::dot_peen_mark::apply(*stock, inOne);

    auto inThree = base;  inThree.text = "XYZ";
    auto outThree = skill::dot_peen_mark::apply(*stock, inThree);

    const int dotsOne   = outOne.signature.pattern["dot_count"].get<int>();
    const int dotsThree = outThree.signature.pattern["dot_count"].get<int>();
    EXPECT_GT(dotsThree, dotsOne);
}
