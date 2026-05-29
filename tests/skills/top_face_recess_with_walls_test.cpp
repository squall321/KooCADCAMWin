// @lat: [[process/test-strategy#skill round-trip]]
//
// top_face_recess_with_walls — REAL-behavior tests for the compound recess +
// walls + corner fillet macro.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/top_face_recess_with_walls.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

#include <cmath>

using namespace koocadcam;

namespace {

double volumeOf(const TopoDS_Shape& s)
{
    GProp_GProps p;
    BRepGProp::VolumeProperties(s, p);
    return p.Mass();
}

skill::top_face_recess_with_walls::Input goodInput()
{
    skill::top_face_recess_with_walls::Input in;
    in.entry_face       = skill::FaceByNormal{ gp_Dir(0, 0, 1), 5.0, "largest" };
    in.spec_class       = "large_pcb";        // wall T=1.5, H=3.0, R=1.0
    in.center_x_mm      = 30.0;
    in.center_y_mm      = 30.0;
    in.inner_length_mm  = 20.0;
    in.inner_width_mm   = 15.0;
    in.recess_depth_mm  = 2.0;
    return in;
}

}  // namespace

// ─── Test 1: apply real net-volume change matches recess+wall math ───────
TEST(SkillTopFaceRecessWithWalls, ApplyNetVolumeMatchesRecessAndWalls)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 10.0);
    ASSERT_FALSE(stock->shape().IsNull());

    auto in = goodInput();
    const double V0 = volumeOf(stock->shape());

    auto out = skill::top_face_recess_with_walls::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    const double T = skill::top_face_recess_with_walls::specWallThicknessFor("large_pcb");
    const double H = skill::top_face_recess_with_walls::specWallHeightFor   ("large_pcb");
    EXPECT_NEAR(T, 1.5, 1e-9);
    EXPECT_NEAR(H, 3.0, 1e-9);

    // Pocket REMOVED: IL × IW × depth = 20 × 15 × 2 = 600
    const double pocketVol = in.inner_length_mm * in.inner_width_mm * in.recess_depth_mm;
    // Walls ADDED: 2 walls (long, along Y) of (T × (IW+2T) × H) + 2 walls (short, along X) of (IL × T × H).
    const double wallVol   = 2.0 * (in.inner_width_mm + 2.0 * T) * T * H
                           + 2.0 * in.inner_length_mm * T * H;
    const double expected  = wallVol - pocketVol;   // delta = (V_new - V_old)

    const double delta = volumeOf(out.workpiece->shape()) - V0;
    // Corner fillets eat a small sliver of the walls — accept 8 %.
    EXPECT_NEAR(delta, expected, std::abs(expected) * 0.08 + 5.0);
}

// ─── Test 2: validate rejects unknown spec + auto-detect fit failure ─────
TEST(SkillTopFaceRecessWithWalls, ValidateRejectsUnknownSpecAndFitFailure)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 10.0);

    // (a) Unknown spec_class
    {
        auto in = goodInput();
        in.spec_class = "M99_pcb";
        auto r = skill::top_face_recess_with_walls::validate(*stock, in);
        EXPECT_FALSE(r.passed);
        bool found = false;
        for (const auto& f : r.findings)
            if (f.code == "DFM-RECESS-UNKNOWN") { found = true; break; }
        EXPECT_TRUE(found);
        EXPECT_THROW(skill::top_face_recess_with_walls::apply(*stock, in), skill::SkillError);
    }

    // (b) Auto-detect fit failure: footprint larger than workpiece bbox.
    {
        auto in = goodInput();
        in.inner_length_mm = 100.0;     // exceeds 60 mm workpiece
        auto r = skill::top_face_recess_with_walls::validate(*stock, in);
        EXPECT_FALSE(r.passed);
        bool found = false;
        for (const auto& f : r.findings)
            if (f.code == "DFM-RECESS-FIT") { found = true; break; }
        EXPECT_TRUE(found);
        EXPECT_THROW(skill::top_face_recess_with_walls::apply(*stock, in), skill::SkillError);
    }
}

// ─── Test 3: signature carries spec_class + derived params ───────────────
TEST(SkillTopFaceRecessWithWalls, SignatureCarriesSpecKeyAndParams)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 10.0);
    auto out = skill::top_face_recess_with_walls::apply(*stock, goodInput());
    const auto& sig = out.signature;

    EXPECT_EQ(sig.params.value("spec_class", std::string()), "large_pcb");
    EXPECT_EQ(sig.pattern.value("kind",         std::string()), "top_face_recess_with_walls");
    EXPECT_TRUE(sig.pattern.value("is_compound", false));
    EXPECT_EQ(sig.pattern.value("subfeature_count", 0), 5);
    EXPECT_NEAR(sig.pattern.value("wall_thickness_mm", 0.0), 1.5, 1e-9);
    EXPECT_NEAR(sig.pattern.value("wall_height_mm",    0.0), 3.0, 1e-9);
    EXPECT_NEAR(sig.pattern.value("fillet_r_mm",       0.0), 1.0, 1e-9);
    EXPECT_NEAR(sig.pattern.value("outer_length_mm",   0.0), 23.0, 1e-9);   // 20 + 2×1.5
}

// ─── Test 4: recognize returns the candidate with the right spec ─────────
TEST(SkillTopFaceRecessWithWalls, RecognizeReturnsCandidateWithSpecKey)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 10.0);
    auto out = skill::top_face_recess_with_walls::apply(*stock, goodInput());
    auto cands = skill::top_face_recess_with_walls::recognize(*out.workpiece);
    ASSERT_FALSE(cands.empty());
    const auto& c = cands.front();
    EXPECT_EQ(c.skill_id, std::string("top_face_recess_with_walls"));
    EXPECT_GT(c.confidence, 0.5);
    EXPECT_EQ(c.recovered_params.value("spec_class", std::string()), "large_pcb");
}

// ─── Test 5: spec table correctness for all 4 entries ────────────────────
TEST(SkillTopFaceRecessWithWalls, SpecTableLookupCorrectness)
{
    EXPECT_NEAR(skill::top_face_recess_with_walls::specWallThicknessFor("small_pcb"),   1.0, 1e-9);
    EXPECT_NEAR(skill::top_face_recess_with_walls::specWallThicknessFor("large_pcb"),   1.5, 1e-9);
    EXPECT_NEAR(skill::top_face_recess_with_walls::specWallThicknessFor("gasket_seat"), 2.0, 1e-9);
    EXPECT_NEAR(skill::top_face_recess_with_walls::specWallThicknessFor("lid_seat"),    1.5, 1e-9);
    EXPECT_NEAR(skill::top_face_recess_with_walls::specWallThicknessFor("UNKNOWN"),     0.0, 1e-9);

    EXPECT_NEAR(skill::top_face_recess_with_walls::specWallHeightFor("small_pcb"),   2.0, 1e-9);
    EXPECT_NEAR(skill::top_face_recess_with_walls::specWallHeightFor("lid_seat"),    4.0, 1e-9);
    EXPECT_NEAR(skill::top_face_recess_with_walls::specFilletRFor   ("small_pcb"),   0.5, 1e-9);
    EXPECT_NEAR(skill::top_face_recess_with_walls::specFilletRFor   ("lid_seat"),    1.0, 1e-9);
}
