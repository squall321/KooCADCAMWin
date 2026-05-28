// @lat: [[process/test-strategy#skill round-trip]]
//
// engrave_text skill — spot-cut approximation tests.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/engrave_text.hpp"

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

// ── 1. Apply removes glyph-extrusion volumes ─────────────────────────────
//
// Slice 6: engrave_text uses real glyph extrusion.  For text "KOO" at
// font_size = 3 mm and depth = 0.2 mm, each glyph removes roughly
// (fill_ratio × font_size² × depth) mm³.  The glyph table is stencil-
// style with ~30-60% fill — we accept a generous band [0.5, 6] mm³ total
// to absorb both the glyph-extrusion path (high volume) and any spot
// fallback for glyphs that fail (low volume per char).  The IMPORTANT
// invariant is: SOME material was removed AND the result is non-null.
TEST(SkillEngraveText, ApplyRemovesPerCharSpot)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::engrave_text::Input in;
    in.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 10.0;
    in.position_y_mm = 25.0;
    in.direction     = gp_Dir(1, 0, 0);
    in.text          = "KOO";
    in.font_size_mm  = 3.0;
    in.stroke_width_mm = 0.5;
    in.depth_mm      = 0.2;

    auto out = skill::engrave_text::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    const double removed = volumeOf(stock->shape()) - volumeOf(out.workpiece->shape());
    // Lower bound: at least one spot's worth (~0.04 mm³).
    // Upper bound: 3 × (font_size² × depth) = 3 × 9 × 0.2 = 5.4 mm³ if every
    // glyph were a fully-filled square.  Real glyphs fill ~30-60% so 2-3 mm³
    // typical, but accept up to 6 for safety.
    EXPECT_GT(removed, 0.03);
    EXPECT_LT(removed, 6.0);

    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_EQ(out.signature.skill_id, std::string("engrave_text"));
    EXPECT_EQ(out.signature.pattern["char_count"].get<size_t>(), 3u);
    // The signature should report a glyph-extrusion-related geometry tag.
    const std::string geom =
        out.signature.pattern["geometry"].get<std::string>();
    EXPECT_TRUE(geom == "glyph_extrusion" ||
                geom == "glyph_with_spot_fallback" ||
                geom == "spot_approximation");
}

// ── 2. DFM-019 rejects sub-min stroke width ──────────────────────────────
TEST(SkillEngraveText, ValidateRejectsThinStroke)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::engrave_text::Input in;
    in.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 10.0;
    in.position_y_mm = 25.0;
    in.text          = "X";
    in.font_size_mm  = 3.0;
    in.stroke_width_mm = 0.10;  // < 0.15
    in.depth_mm      = 0.2;

    auto r = skill::engrave_text::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool dfm019Found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-019") { dfm019Found = true; break; }
    EXPECT_TRUE(dfm019Found);
    EXPECT_THROW(skill::engrave_text::apply(*stock, in), skill::SkillError);
}

// ── 3. Depth out of range rejected ───────────────────────────────────────
TEST(SkillEngraveText, ValidateRejectsOutOfRangeDepth)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::engrave_text::Input in;
    in.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 10.0;
    in.position_y_mm = 25.0;
    in.text          = "X";
    in.font_size_mm  = 3.0;
    in.stroke_width_mm = 0.2;
    in.depth_mm      = 2.0;   // > 0.5

    auto r = skill::engrave_text::validate(*stock, in);
    EXPECT_FALSE(r.passed);
}

// ── 4. Empty text rejected ───────────────────────────────────────────────
TEST(SkillEngraveText, ValidateRejectsEmptyText)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::engrave_text::Input in;
    in.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.text          = "";
    in.stroke_width_mm = 0.2;
    in.depth_mm      = 0.2;

    auto r = skill::engrave_text::validate(*stock, in);
    EXPECT_FALSE(r.passed);
}

// ── 5. Recognize replays metadata ────────────────────────────────────────
TEST(SkillEngraveText, RecognizeReplaysMetadata)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::engrave_text::Input in;
    in.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 10.0;
    in.position_y_mm = 25.0;
    in.text          = "HELLO";
    in.font_size_mm  = 2.0;
    in.stroke_width_mm = 0.3;
    in.depth_mm      = 0.15;

    auto out = skill::engrave_text::apply(*stock, in);
    auto cands = skill::engrave_text::recognize(*out.workpiece);

    ASSERT_FALSE(cands.empty());
    EXPECT_EQ(cands[0].recovered_params["text"], "HELLO");
    EXPECT_NEAR(cands[0].recovered_params["font_size_mm"].get<double>(), 2.0, 1e-6);
    EXPECT_NEAR(cands[0].recovered_params["depth_mm"].get<double>(), 0.15, 1e-6);
    EXPECT_GT(cands[0].confidence, 0.9);
}
