// @lat: [[process/test-strategy#skill round-trip]]
//
// clinch_nut skill — verifies sheet-thickness sensing, barrel-size lookup,
// through-hole synthesis, and DFM bounds.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/clinch_nut.hpp"

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

}  // namespace

// ─── 1. Apply pierces a through-hole sized for the clinch barrel ─────────
TEST(SkillClinchNut, ApplyCreatesBarrelHole)
{
    auto stock = skill::createCuboidStock(100.0, 60.0, 1.5);     // 1.5 mm sheet

    skill::clinch_nut::Input in;
    in.entry_face        = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm     = 30.0;
    in.position_y_mm     = 25.0;
    in.thread_size       = "M4";
    in.sheet_thickness_mm = 0.0;     // auto-sense from stock

    auto out = skill::clinch_nut::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    // M4 → 4.5 mm barrel, 1.5 mm sheet → volume = π × 2.25² × 1.5
    const double barrel = 4.5;
    const double approx = M_PI * (barrel/2.0) * (barrel/2.0) * 1.5;
    const double removed = volumeOf(stock->shape()) - volumeOf(out.workpiece->shape());
    EXPECT_NEAR(removed, approx, approx * 0.05);

    EXPECT_EQ(out.signature.skill_id, std::string("clinch_nut"));
    EXPECT_NEAR(out.signature.params["barrel_dia_mm"].get<double>(), barrel, 1e-9);
    EXPECT_NEAR(out.signature.params["sheet_thickness_mm"].get<double>(), 1.5, 1e-9);
    EXPECT_FALSE(out.signature.pattern["bottom_planar_face_present"].get<bool>());
}

// ─── 2. Barrel-size lookup ───────────────────────────────────────────────
TEST(SkillClinchNut, BarrelSizeLookup)
{
    EXPECT_NEAR(skill::clinch_nut::clinchBarrelDiameter("M3"), 3.5, 1e-9);
    EXPECT_NEAR(skill::clinch_nut::clinchBarrelDiameter("M4"), 4.5, 1e-9);
    EXPECT_NEAR(skill::clinch_nut::clinchBarrelDiameter("M5"), 5.5, 1e-9);
    EXPECT_NEAR(skill::clinch_nut::clinchBarrelDiameter("M6"), 6.5, 1e-9);
    EXPECT_NEAR(skill::clinch_nut::clinchBarrelDiameter("M8"), 8.5, 1e-9);
    EXPECT_EQ  (skill::clinch_nut::clinchBarrelDiameter("M99"), 0.0);

    // Reverse lookup
    EXPECT_EQ(skill::clinch_nut::threadSizeForBarrel(4.5, 0.05), std::string("M4"));
    EXPECT_EQ(skill::clinch_nut::threadSizeForBarrel(8.5, 0.05), std::string("M8"));
    EXPECT_TRUE(skill::clinch_nut::threadSizeForBarrel(7.0, 0.05).empty());
}

// ─── 3. DFM rejects too-thick "sheet" stock ──────────────────────────────
TEST(SkillClinchNut, ValidateRejectsThickSheet)
{
    // 5 mm thick is well past 3.0 mm clinch limit.
    auto thick = skill::createCuboidStock(80.0, 80.0, 5.0);

    skill::clinch_nut::Input in;
    in.entry_face       = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm    = 40.0;
    in.position_y_mm    = 40.0;
    in.thread_size      = "M5";
    in.sheet_thickness_mm = 0.0;     // auto → sensed 5 mm

    auto r = skill::clinch_nut::validate(*thick, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-CN-SHEET") { found = true; break; }
    EXPECT_TRUE(found);
    EXPECT_THROW(skill::clinch_nut::apply(*thick, in), skill::SkillError);
}

// ─── 4. DFM warns on sensed-vs-input mismatch ────────────────────────────
TEST(SkillClinchNut, ValidateWarnsOnMismatch)
{
    auto stock = skill::createCuboidStock(80.0, 80.0, 1.5);

    skill::clinch_nut::Input in;
    in.entry_face       = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm    = 40.0;
    in.position_y_mm    = 40.0;
    in.thread_size      = "M3";
    in.sheet_thickness_mm = 2.5;     // disagrees with sensed 1.5 mm

    auto r = skill::clinch_nut::validate(*stock, in);
    // Mismatch is a WARNING — passed is still true (no errors).
    EXPECT_TRUE(r.passed);
    bool warned = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-CN-MISMATCH" && f.severity == "warning") {
            warned = true; break;
        }
    EXPECT_TRUE(warned);
}

// ─── 5. Recognize finds clinch nut from history ──────────────────────────
TEST(SkillClinchNut, RecognizeReplaysFromHistory)
{
    auto stock = skill::createCuboidStock(80.0, 60.0, 2.0);

    skill::clinch_nut::Input in;
    in.entry_face       = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm    = 30.0;
    in.position_y_mm    = 25.0;
    in.thread_size      = "M5";

    auto out = skill::clinch_nut::apply(*stock, in);
    auto cands = skill::clinch_nut::recognize(*out.workpiece);

    ASSERT_FALSE(cands.empty());
    const auto& c = cands.front();
    EXPECT_GT(c.confidence, 0.9);
    EXPECT_EQ(c.recovered_params["thread_size"].get<std::string>(), std::string("M5"));
    EXPECT_NEAR(c.recovered_params["barrel_dia_mm"].get<double>(),     5.5, 1e-9);
    EXPECT_NEAR(c.recovered_params["sheet_thickness_mm"].get<double>(), 2.0, 1e-9);
}

// ─── 6. Topology fallback recognises clinch hole in bare sheet ───────────
TEST(SkillClinchNut, TopologyFallbackBareSheet)
{
    auto stock = skill::createCuboidStock(80.0, 60.0, 1.2);

    skill::clinch_nut::Input in;
    in.entry_face       = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm    = 40.0;
    in.position_y_mm    = 30.0;
    in.thread_size      = "M3";          // barrel = 3.5 mm
    auto out = skill::clinch_nut::apply(*stock, in);

    // Discard history → forces topology fallback.
    skill::Workpiece bare(out.workpiece->shape(), "aluminum_6061");
    auto cands = skill::clinch_nut::recognize(bare);
    ASSERT_FALSE(cands.empty());
    EXPECT_EQ(cands.front().recovered_params["thread_size"].get<std::string>(),
              std::string("M3"));
    EXPECT_NEAR(cands.front().recovered_params["sheet_thickness_mm"].get<double>(),
                1.2, 1e-2);
}

// ─── 7. Topology fallback rejects holes in thick stock ───────────────────
TEST(SkillClinchNut, TopologyRejectsThickStock)
{
    auto thick = skill::createCuboidStock(50.0, 50.0, 10.0);     // not a sheet
    auto cands = skill::clinch_nut::recognize(*thick);
    EXPECT_TRUE(cands.empty());
}
