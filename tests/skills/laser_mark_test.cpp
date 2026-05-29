// @lat: [[process/test-strategy#skill round-trip]]
//
// laser_mark skill — verifies REAL geometric impression (shallow rectangular
// glyph cuts) plus DFM gates and dual-path recognition.

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

// ── 1. Apply removes a shallow glyph volume (REAL geometric change) ──────
//
// Volume-delta math:
//   2 glyphs × 0.30 fill fraction × (3 mm)² × 0.05 mm depth ≈ 0.27 mm³.
//   We assert > 1e-5 mm³ (well above OCCT noise) and < 5 mm³ (sanity cap).
TEST(SkillLaserMark, ApplyRemovesShallowGlyphVolume)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);
    const double volBefore = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();

    skill::laser_mark::Input in;
    in.entry_face    = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 10.0;
    in.position_y_mm = 25.0;
    in.direction_xy  = gp_Dir(1, 0, 0);
    in.text          = "AB";
    in.font_size_mm  = 3.0;
    in.mark_depth_um = 50.0;  // 50 µm — satisfies aluminum (default 6061) minimum

    auto out = skill::laser_mark::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    const double removed = volBefore - volumeOf(out.workpiece->shape());
    // Real geometric change — expect a strictly positive sub-mm³ removal.
    EXPECT_GT(removed, 1e-5);
    EXPECT_LT(removed, 5.0);

    // Face count should INCREASE (glyph cuts add new walls + bottom faces).
    EXPECT_GT(out.workpiece->faceCount(), faceBefore);

    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_EQ(out.signature.skill_id, std::string("laser_mark"));
    EXPECT_EQ(out.signature.pattern["char_count"].get<size_t>(), 2u);
    EXPECT_NEAR(out.signature.pattern["mark_depth_um"].get<double>(), 50.0, 1e-6);

    // Signature must carry glyph count + non-zero stock_removed_mm3.
    const int glyphsBuilt = out.signature.pattern["glyphs_built"].get<int>();
    const int spotFallbk  = out.signature.pattern["spot_fallbacks"].get<int>();
    EXPECT_EQ(glyphsBuilt + spotFallbk, 2);  // 2 chars → 2 cutters total
    EXPECT_GT(out.signature.tooling.stock_removed_mm3, 0.0);
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

// ── 3. DFM-LASER-MAT enforces aluminum ≥ 50 µm, steel ≥ 20 µm ────────────
//
// Engineering basis: ASM Davis "Aluminum and Aluminum Alloys" 1993 §15
// (Al₂O₃ clearance) and Reactive Engineering Marking Guide §4 (steel
// oxide-layer minimum).
TEST(SkillLaserMark, ValidateRejectsAluminumTooShallow)
{
    // createCuboidStock defaults to material "aluminum_6061" — see Stock.cpp.
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::laser_mark::Input in;
    in.entry_face    = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.text          = "X";
    in.font_size_mm  = 2.0;
    in.mark_depth_um = 30.0;  // > 5 µm absolute, < 50 µm aluminum minimum

    auto r = skill::laser_mark::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool foundMat = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-LASER-MAT") { foundMat = true; break; }
    EXPECT_TRUE(foundMat);
}

TEST(SkillLaserMark, ValidateAcceptsSteelAt25um)
{
    // Build a steel workpiece directly.
    auto stockAlu = skill::createCuboidStock(50.0, 50.0, 10.0);
    skill::Workpiece steelWp(stockAlu->shape(), "stainless_steel_304");

    skill::laser_mark::Input in;
    in.entry_face    = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.text          = "S1";
    in.font_size_mm  = 2.0;
    in.mark_depth_um = 25.0;   // ≥ 20 µm steel minimum

    auto r = skill::laser_mark::validate(steelWp, in);
    EXPECT_TRUE(r.passed);
}

// ── 4. Empty text rejected ───────────────────────────────────────────────
TEST(SkillLaserMark, ValidateRejectsEmptyText)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::laser_mark::Input in;
    in.entry_face    = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.text          = "";
    in.font_size_mm  = 2.0;
    in.mark_depth_um = 60.0;

    auto r = skill::laser_mark::validate(*stock, in);
    EXPECT_FALSE(r.passed);
}

// ── 5. Recognize: metadata replay PRIMARY at 1.0, geometric FALLBACK at 0.4
TEST(SkillLaserMark, RecognizeMetadataPrimaryGeometricFallback)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::laser_mark::Input in;
    in.entry_face    = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 5.0;
    in.position_y_mm = 25.0;
    in.text          = "SN12";
    in.font_size_mm  = 2.5;
    in.mark_depth_um = 60.0;  // ≥ 50 µm aluminum minimum

    auto out = skill::laser_mark::apply(*stock, in);

    // With metadata present → primary candidate is metadata replay at 1.0.
    auto cands = skill::laser_mark::recognize(*out.workpiece);
    ASSERT_FALSE(cands.empty());
    EXPECT_EQ(cands[0].skill_id, std::string("laser_mark"));
    EXPECT_EQ(cands[0].recovered_params["text"], "SN12");
    EXPECT_NEAR(cands[0].recovered_params["font_size_mm"].get<double>(), 2.5, 1e-6);
    EXPECT_NEAR(cands[0].recovered_params["mark_depth_um"].get<double>(), 60.0, 1e-6);
    EXPECT_GT(cands[0].confidence, 0.9);
    EXPECT_EQ(cands[0].matched_geometry["source"].get<std::string>(),
              std::string("metadata_replay"));

    // Strip metadata → exercise geometric fallback path at confidence 0.4.
    skill::Workpiece nakedWp(out.workpiece->shape(), out.workpiece->material());
    auto geomCands = skill::laser_mark::recognize(nakedWp);
    ASSERT_FALSE(geomCands.empty());
    EXPECT_EQ(geomCands[0].matched_geometry["source"].get<std::string>(),
              std::string("geometric_fallback"));
    EXPECT_NEAR(geomCands[0].confidence, 0.4, 1e-9);
    EXPECT_GT(geomCands[0].matched_geometry["shallow_pockets_count"].get<int>(), 0);
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
    in.mark_depth_um = 60.0;  // satisfies aluminum minimum

    auto out = skill::laser_mark::apply(*stock, in);
    EXPECT_EQ(out.signature.tooling.tool_type, std::string("laser_marker"));
    EXPECT_EQ(out.signature.tooling.tool_material, std::string("fiber_laser"));
    EXPECT_EQ(out.signature.tooling.flute_count, 0);
    EXPECT_GT(out.signature.tooling.stock_removed_mm3, 0.0);
}
