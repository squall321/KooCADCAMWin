// @lat: [[process/test-strategy#skill round-trip]]
//
// partition_wall_with_passthrough — REAL-behavior tests for the wall+hole
// compound macro.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/partition_wall_with_passthrough.hpp"

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

skill::partition_wall_with_passthrough::Input goodInput()
{
    skill::partition_wall_with_passthrough::Input in;
    in.entry_face        = skill::FaceByNormal{ gp_Dir(0, 0, 1), 5.0, "largest" };
    in.spec_class        = "connector_pass";    // 12 × 8 mm cutout
    in.center_x_mm       = 30.0;
    in.center_y_mm       = 30.0;
    in.wall_length_mm    = 40.0;
    in.wall_thickness_mm = 3.0;
    in.wall_height_mm    = 12.0;
    in.cut_offset_x_mm   = 0.0;                 // centred
    in.cut_offset_z_mm   = 0.0;                 // auto: centred Z
    in.length_dir        = gp_Dir(1, 0, 0);
    return in;
}

}  // namespace

// ─── Test 1: apply net volume = wall_added - cutout_removed ──────────────
TEST(SkillPartitionWallWithPassthrough, ApplyNetVolumeMatchesWallMinusCutout)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 10.0);
    ASSERT_FALSE(stock->shape().IsNull());

    auto in = goodInput();
    const double V0 = volumeOf(stock->shape());

    auto out = skill::partition_wall_with_passthrough::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    const double Wc = skill::partition_wall_with_passthrough::specCutoutWFor("connector_pass");
    const double Hc = skill::partition_wall_with_passthrough::specCutoutHFor("connector_pass");
    EXPECT_NEAR(Wc, 12.0, 1e-9);
    EXPECT_NEAR(Hc,  8.0, 1e-9);

    const double wallVol = in.wall_length_mm * in.wall_thickness_mm * in.wall_height_mm;
    const double cutVol  = Wc * in.wall_thickness_mm * Hc;
    const double expected = wallVol - cutVol;

    const double delta = volumeOf(out.workpiece->shape()) - V0;
    EXPECT_NEAR(delta, expected, std::abs(expected) * 0.05);
}

// ─── Test 2: validate rejects unknown spec + auto-strap fail ─────────────
TEST(SkillPartitionWallWithPassthrough, ValidateRejectsUnknownSpecAndStrapFailure)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 10.0);

    // (a) Unknown spec
    {
        auto in = goodInput();
        in.spec_class = "M99_pass";
        auto r = skill::partition_wall_with_passthrough::validate(*stock, in);
        EXPECT_FALSE(r.passed);
        bool found = false;
        for (const auto& f : r.findings)
            if (f.code == "DFM-PASS-UNKNOWN") { found = true; break; }
        EXPECT_TRUE(found);
        EXPECT_THROW(skill::partition_wall_with_passthrough::apply(*stock, in), skill::SkillError);
    }

    // (b) Auto-detect strap failure: connector_pass (12×8), 10 mm wall, but
    //     cutout pushed up to leave only 0.5 mm strap above it.
    //     cut_offset_z = 5.5 → cutout_top_z = 5.5 + 4 = 9.5 → top_strap = 0.5.
    {
        auto in = goodInput();
        in.wall_height_mm  = 10.0;
        in.cut_offset_z_mm = 5.5;          // pushes cutout to wall top
        auto r = skill::partition_wall_with_passthrough::validate(*stock, in);
        EXPECT_FALSE(r.passed);
        bool found = false;
        for (const auto& f : r.findings)
            if (f.code == "DFM-PASS-STRAP") { found = true; break; }
        EXPECT_TRUE(found);
        EXPECT_THROW(skill::partition_wall_with_passthrough::apply(*stock, in), skill::SkillError);
    }
}

// ─── Test 3: signature carries spec_class + derived strap values ─────────
TEST(SkillPartitionWallWithPassthrough, SignatureCarriesSpecKeyAndStraps)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 10.0);
    auto out = skill::partition_wall_with_passthrough::apply(*stock, goodInput());
    const auto& sig = out.signature;
    EXPECT_EQ(sig.params.value("spec_class", std::string()), "connector_pass");
    EXPECT_EQ(sig.pattern.value("kind",         std::string()), "partition_wall_with_passthrough");
    EXPECT_TRUE(sig.pattern.value("is_compound", false));
    EXPECT_EQ(sig.pattern.value("subfeature_count", 0), 2);
    EXPECT_NEAR(sig.pattern.value("cutout_w_mm", 0.0), 12.0, 1e-9);
    EXPECT_NEAR(sig.pattern.value("cutout_h_mm", 0.0),  8.0, 1e-9);
    // Top strap = wall_height - (cut_offset_z + cutout_h/2).
    // cut_offset_z auto = (12-8)/2 = 2.  top_strap = 12 - (2 + 4) = 6.
    EXPECT_NEAR(sig.pattern.value("top_strap_mm", 0.0), 6.0, 1e-6);
}

// ─── Test 4: recognize returns the candidate with the right spec ─────────
TEST(SkillPartitionWallWithPassthrough, RecognizeReturnsCandidateWithSpecKey)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 10.0);
    auto out = skill::partition_wall_with_passthrough::apply(*stock, goodInput());
    auto cands = skill::partition_wall_with_passthrough::recognize(*out.workpiece);
    ASSERT_FALSE(cands.empty());
    const auto& c = cands.front();
    EXPECT_EQ(c.skill_id, std::string("partition_wall_with_passthrough"));
    EXPECT_GT(c.confidence, 0.5);
    EXPECT_EQ(c.recovered_params.value("spec_class", std::string()), "connector_pass");
}

// ─── Test 5: spec table lookup correctness ───────────────────────────────
TEST(SkillPartitionWallWithPassthrough, SpecTableLookupCorrectness)
{
    EXPECT_NEAR(skill::partition_wall_with_passthrough::specCutoutWFor("cable_pass"),      6.0, 1e-9);
    EXPECT_NEAR(skill::partition_wall_with_passthrough::specCutoutWFor("connector_pass"), 12.0, 1e-9);
    EXPECT_NEAR(skill::partition_wall_with_passthrough::specCutoutWFor("airflow_louver"), 20.0, 1e-9);
    EXPECT_NEAR(skill::partition_wall_with_passthrough::specCutoutWFor("usb_c_pass"),      9.0, 1e-9);
    EXPECT_NEAR(skill::partition_wall_with_passthrough::specCutoutWFor("hdmi_pass"),      16.0, 1e-9);
    EXPECT_NEAR(skill::partition_wall_with_passthrough::specCutoutWFor("UNKNOWN"),         0.0, 1e-9);

    EXPECT_NEAR(skill::partition_wall_with_passthrough::specCutoutHFor("cable_pass"),      4.0, 1e-9);
    EXPECT_NEAR(skill::partition_wall_with_passthrough::specCutoutHFor("airflow_louver"),  3.0, 1e-9);
    EXPECT_NEAR(skill::partition_wall_with_passthrough::specCutoutHFor("hdmi_pass"),       7.0, 1e-9);
}
