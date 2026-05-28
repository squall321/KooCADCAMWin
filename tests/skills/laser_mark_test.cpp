// @lat: [[process/test-strategy#skill round-trip]]
//
// laser_mark skill — shallow glyph extrusion / spot-cylinder fallback tests.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/laser_mark.hpp"

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

// ── 1. Apply removes a tiny glyph volume ─────────────────────────────────
TEST(SkillLaserMark, ApplyRemovesShallowGlyphVolume)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::laser_mark::Input in;
    in.entry_face    = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 10.0;
    in.position_y_mm = 25.0;
    in.direction_xy  = gp_Dir(1, 0, 0);
    in.text          = "AB";
    in.font_size_mm  = 3.0;
    in.mark_depth_um = 50.0;  // 50 µm = 0.05 mm

    auto out = skill::laser_mark::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    const double removed = volumeOf(stock->shape()) - volumeOf(out.workpiece->shape());
    // 2 glyphs × ~0.3 fill × 9 mm² × 0.05 mm ≈ 0.27 mm³ upper-ish; lower
    // bound just need > 0.
    EXPECT_GT(removed, 0.0);
    EXPECT_LT(removed, 5.0);

    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_EQ(out.signature.skill_id, std::string("laser_mark"));
    EXPECT_EQ(out.signature.pattern["char_count"].get<size_t>(), 2u);
    EXPECT_NEAR(out.signature.pattern["mark_depth_um"].get<double>(), 50.0, 1e-6);
}

// ── 2. DFM-LASER-DEPTH rejects out-of-range depth ────────────────────────
TEST(SkillLaserMark, ValidateRejectsOutOfRangeDepth)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    // Too shallow.
    skill::laser_mark::Input tooShallow;
    tooShallow.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    tooShallow.text         = "X";
    tooShallow.font_size_mm = 2.0;
    tooShallow.mark_depth_um = 1.0;  // < 5 µm
    auto r1 = skill::laser_mark::validate(*stock, tooShallow);
    EXPECT_FALSE(r1.passed);
    bool found1 = false;
    for (const auto& f : r1.findings)
        if (f.code == "DFM-LASER-DEPTH") { found1 = true; break; }
    EXPECT_TRUE(found1);
    EXPECT_THROW(skill::laser_mark::apply(*stock, tooShallow), skill::SkillError);

    // Too deep.
    skill::laser_mark::Input tooDeep = tooShallow;
    tooDeep.mark_depth_um = 300.0;  // > 200 µm
    auto r2 = skill::laser_mark::validate(*stock, tooDeep);
    EXPECT_FALSE(r2.passed);
}

// ── 3. DFM-LASER-FONT rejects too-small font size ────────────────────────
TEST(SkillLaserMark, ValidateRejectsTinyFont)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::laser_mark::Input in;
    in.entry_face    = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.text          = "HI";
    in.font_size_mm  = 0.3;  // < 0.5
    in.mark_depth_um = 30.0;

    auto r = skill::laser_mark::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-LASER-FONT") { found = true; break; }
    EXPECT_TRUE(found);
}

// ── 4. Empty text rejected ───────────────────────────────────────────────
TEST(SkillLaserMark, ValidateRejectsEmptyText)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::laser_mark::Input in;
    in.entry_face    = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.text          = "";
    in.font_size_mm  = 2.0;
    in.mark_depth_um = 30.0;

    auto r = skill::laser_mark::validate(*stock, in);
    EXPECT_FALSE(r.passed);
}

// ── 5. Recognize replays metadata ────────────────────────────────────────
TEST(SkillLaserMark, RecognizeReplaysMetadata)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::laser_mark::Input in;
    in.entry_face    = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 5.0;
    in.position_y_mm = 25.0;
    in.text          = "SN12";
    in.font_size_mm  = 2.5;
    in.mark_depth_um = 40.0;

    auto out = skill::laser_mark::apply(*stock, in);
    auto cands = skill::laser_mark::recognize(*out.workpiece);

    ASSERT_FALSE(cands.empty());
    EXPECT_EQ(cands[0].skill_id, std::string("laser_mark"));
    EXPECT_EQ(cands[0].recovered_params["text"], "SN12");
    EXPECT_NEAR(cands[0].recovered_params["font_size_mm"].get<double>(), 2.5, 1e-6);
    EXPECT_NEAR(cands[0].recovered_params["mark_depth_um"].get<double>(), 40.0, 1e-6);
    EXPECT_GT(cands[0].confidence, 0.9);
}

// ── 6. Tooling metadata populated correctly ─────────────────────────────
TEST(SkillLaserMark, ToolingMetadataLaserMarker)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::laser_mark::Input in;
    in.entry_face    = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 10.0;
    in.position_y_mm = 25.0;
    in.text          = "OK";
    in.font_size_mm  = 2.0;
    in.mark_depth_um = 30.0;

    auto out = skill::laser_mark::apply(*stock, in);
    EXPECT_EQ(out.signature.tooling.tool_type, std::string("laser_marker"));
    EXPECT_EQ(out.signature.tooling.tool_material, std::string("fiber_laser"));
    EXPECT_EQ(out.signature.tooling.flute_count, 0);
}
