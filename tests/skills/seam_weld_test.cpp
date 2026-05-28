// @lat: [[process/test-strategy#skill round-trip]]
//
// seam_weld skill — 5-case sweep.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/seam_weld.hpp"

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

// ─── 1. Apply: bead fuses onto top, volume increases by stadium area × h ──
TEST(SkillSeamWeld, ApplyAddsStadiumBead)
{
    auto stock = skill::createCuboidStock(80.0, 40.0, 10.0);
    const double volBefore = volumeOf(stock->shape());

    skill::seam_weld::Input in;
    in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1), 5.0, "largest" };
    in.start_x_mm = 20.0; in.start_y_mm = 20.0;
    in.end_x_mm   = 60.0; in.end_y_mm   = 20.0;
    in.bead_width_mm  = 6.0;
    in.bead_height_mm = 1.0;

    auto out = skill::seam_weld::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    // Stadium volume = (π r² + L × w) × h
    const double r = 3.0;
    const double L = 40.0;
    const double approxAdded = (M_PI * r * r + L * 6.0) * 1.0;
    const double volAfter = volumeOf(out.workpiece->shape());
    const double added = volAfter - volBefore;
    EXPECT_GT(added, approxAdded * 0.5);
    EXPECT_LT(added, approxAdded * 1.5);

    EXPECT_EQ(out.signature.skill_id, std::string("seam_weld"));
    EXPECT_EQ(out.signature.tooling.tool_type, std::string("seam_welder"));
}

// ─── 2. DFM: bead too narrow rejected ─────────────────────────────────────
TEST(SkillSeamWeld, ValidateRejectsBadWidth)
{
    auto stock = skill::createCuboidStock(80.0, 40.0, 10.0);

    skill::seam_weld::Input in;
    in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.start_x_mm = 20.0; in.start_y_mm = 20.0;
    in.end_x_mm   = 60.0; in.end_y_mm   = 20.0;
    in.bead_width_mm  = 1.5;     // < 3
    in.bead_height_mm = 0.5;

    auto r = skill::seam_weld::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_THROW(skill::seam_weld::apply(*stock, in), skill::SkillError);
}

// ─── 3. DFM: tall bead (height > 0.4 × width) rejected ────────────────────
TEST(SkillSeamWeld, ValidateRejectsTallBead)
{
    auto stock = skill::createCuboidStock(80.0, 40.0, 10.0);

    skill::seam_weld::Input in;
    in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.start_x_mm = 20.0; in.start_y_mm = 20.0;
    in.end_x_mm   = 60.0; in.end_y_mm   = 20.0;
    in.bead_width_mm  = 4.0;
    in.bead_height_mm = 3.0;     // 3 / 4 > 0.4

    auto r = skill::seam_weld::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-WELD-RATIO") { found = true; break; }
    EXPECT_TRUE(found);
}

// ─── 4. Recognize via metadata ────────────────────────────────────────────
TEST(SkillSeamWeld, RecognizeViaMetadata)
{
    auto stock = skill::createCuboidStock(80.0, 40.0, 10.0);

    skill::seam_weld::Input in;
    in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.start_x_mm = 20.0; in.start_y_mm = 20.0;
    in.end_x_mm   = 60.0; in.end_y_mm   = 20.0;
    in.bead_width_mm  = 5.0;
    in.bead_height_mm = 1.0;

    auto out = skill::seam_weld::apply(*stock, in);
    auto cands = skill::seam_weld::recognize(*out.workpiece);

    ASSERT_FALSE(cands.empty());
    EXPECT_GT(cands[0].confidence, 0.9);
    EXPECT_NEAR(cands[0].recovered_params["bead_width_mm"].get<double>(), 5.0, 1e-3);
    EXPECT_NEAR(cands[0].recovered_params["bead_height_mm"].get<double>(), 1.0, 1e-3);
}

// ─── 5. Diagonal seam (45° orientation math) ──────────────────────────────
TEST(SkillSeamWeld, DiagonalSeam)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::seam_weld::Input in;
    in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.start_x_mm = 15.0; in.start_y_mm = 15.0;
    in.end_x_mm   = 35.0; in.end_y_mm   = 35.0;
    in.bead_width_mm  = 4.0;
    in.bead_height_mm = 0.8;

    auto out = skill::seam_weld::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    const double vBefore = volumeOf(stock->shape());
    const double vAfter  = volumeOf(out.workpiece->shape());
    EXPECT_GT(vAfter, vBefore);
}

// ─── 6. Tooling metadata records additive volume (negative removal) ───────
TEST(SkillSeamWeld, ToolingMetaRecordsAdditive)
{
    auto stock = skill::createCuboidStock(80.0, 40.0, 10.0);
    skill::seam_weld::Input in;
    in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.start_x_mm = 10.0; in.start_y_mm = 20.0;
    in.end_x_mm   = 70.0; in.end_y_mm   = 20.0;
    in.bead_width_mm  = 6.0;
    in.bead_height_mm = 1.0;

    auto out = skill::seam_weld::apply(*stock, in);
    EXPECT_LT(out.signature.tooling.stock_removed_mm3, 0.0)
        << "additive process should record negative stock_removed_mm3";
}
