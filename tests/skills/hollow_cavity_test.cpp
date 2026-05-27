// @lat: [[process/test-strategy#skill round-trip]]
//
// hollow_cavity — bulk-remove interior material from a planar entry face,
// inset by a wall_thickness perimeter.  Slice-1: cuboid stock → rectangular
// cavity; cylindrical stock → cylindrical cavity.
//
// Cases (5):
//   1. apply on cuboid produces a rectangular cavity of the expected XY size.
//   2. apply on cylindrical stock produces a cylindrical cavity.
//   3. DFM-001 warns when wall_thickness < 0.4 mm.
//   4. recognize finds a cavity bottom and recovers wall_thickness & depth.
//   5. Edge: wall_thickness too large (≥ half the XY extent) → SkillError.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/hollow_cavity.hpp"

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

// ─── 1. Cuboid stock: rectangular cavity ──────────────────────────────────
TEST(SkillHollowCavity, ApplyOnCuboidRemovesInterior)
{
    auto stock = skill::createCuboidStock(40.0, 30.0, 10.0);
    ASSERT_FALSE(stock->shape().IsNull());
    const double stockVol = volumeOf(stock->shape());

    skill::hollow_cavity::Input in;
    in.entry_face        = skill::FaceByNormal{ gp_Dir(0, 0, 1), 5.0, "largest" };
    in.wall_thickness_mm = 2.0;
    in.depth_mm          = 5.0;

    auto out = skill::hollow_cavity::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    // Cavity XY footprint = (40 - 4) × (30 - 4) = 36 × 26 = 936 mm²
    // Volume removed ≈ 936 × 5 = 4680 mm³
    const double approx = (40.0 - 4.0) * (30.0 - 4.0) * 5.0;
    const double diff = stockVol - volumeOf(out.workpiece->shape());
    EXPECT_NEAR(diff, approx, approx * 0.02);

    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_EQ(out.signature.skill_id, std::string("hollow_cavity"));
    EXPECT_EQ(out.signature.pattern["cavity_shape"].get<std::string>(),
              std::string("rectangular"));
}

// ─── 2. Cylindrical stock: cylindrical cavity ─────────────────────────────
TEST(SkillHollowCavity, ApplyOnCylindricalRemovesInterior)
{
    auto stock = skill::createCylindricalStock(30.0, 10.0);
    ASSERT_FALSE(stock->shape().IsNull());
    const double stockVol = volumeOf(stock->shape());

    skill::hollow_cavity::Input in;
    in.entry_face        = skill::FaceByNormal{ gp_Dir(0, 0, 1), 5.0, "largest" };
    in.wall_thickness_mm = 1.5;
    in.depth_mm          = 6.0;

    auto out = skill::hollow_cavity::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    // Cavity is cylindrical: radius = 15 - 1.5 = 13.5 mm.  Vol ≈ π × 13.5² × 6
    const double cavityR = 15.0 - 1.5;
    const double approx = M_PI * cavityR * cavityR * 6.0;
    const double diff = stockVol - volumeOf(out.workpiece->shape());
    EXPECT_NEAR(diff, approx, approx * 0.02);

    EXPECT_EQ(out.signature.pattern["cavity_shape"].get<std::string>(),
              std::string("cylindrical"));
}

// ─── 3. DFM-001 warning on thin walls ─────────────────────────────────────
TEST(SkillHollowCavity, ValidateWarnsOnThinWalls)
{
    auto stock = skill::createCuboidStock(40.0, 30.0, 10.0);

    skill::hollow_cavity::Input in;
    in.entry_face        = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.wall_thickness_mm = 0.3;     // < 0.4 mm baseline
    in.depth_mm          = 4.0;

    auto r = skill::hollow_cavity::validate(*stock, in);
    // It's a warning, so passed remains true.
    EXPECT_TRUE(r.passed);
    bool warn = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-001" && f.severity == "warning") { warn = true; break; }
    EXPECT_TRUE(warn);
}

// ─── 4. Recognize recovers wall_thickness & depth ─────────────────────────
TEST(SkillHollowCavity, RecognizeRecoversParams)
{
    auto stock = skill::createCuboidStock(50.0, 40.0, 12.0);

    skill::hollow_cavity::Input in;
    in.entry_face        = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.wall_thickness_mm = 3.0;
    in.depth_mm          = 6.0;

    auto out = skill::hollow_cavity::apply(*stock, in);
    auto cands = skill::hollow_cavity::recognize(*out.workpiece);
    ASSERT_GE(cands.size(), 1u);

    const auto& c = cands.front();
    EXPECT_GT(c.confidence, 0.7);
    EXPECT_NEAR(c.recovered_params["wall_thickness_mm"].get<double>(),
                in.wall_thickness_mm, 0.05);
    EXPECT_NEAR(c.recovered_params["depth_mm"].get<double>(),
                in.depth_mm, 0.05);
    EXPECT_EQ(c.recovered_params["cavity_shape"].get<std::string>(),
              std::string("rectangular"));
}

// ─── 5. Edge: wall_thickness too large → SkillError ───────────────────────
TEST(SkillHollowCavity, ValidateRejectsOversizeWall)
{
    auto stock = skill::createCuboidStock(20.0, 20.0, 10.0);

    skill::hollow_cavity::Input in;
    in.entry_face        = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    // 2 × wall_thickness = 22 mm > stock width (20 mm) → no room for cavity.
    in.wall_thickness_mm = 11.0;
    in.depth_mm          = 4.0;

    // DFM itself passes (no rule encodes "fits in stock"), but synthesis
    // should throw a SkillError when the cavity bbox collapses.
    EXPECT_THROW(skill::hollow_cavity::apply(*stock, in), skill::SkillError);
}
