// @lat: [[process/test-strategy#compound cable_grommet_pass_through]]
//
// Verifies REAL multi-cut geometry: through bore + retention groove +
// entry chamfer per spec table.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/cable_grommet_pass_through.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

#include <cmath>

using namespace koocadcam;

namespace {
double volumeOf(const TopoDS_Shape& s) {
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}
}  // namespace

// ─── T1. apply: volume delta matches GR-8 spec (±20%) ───────────────────
TEST(SkillCableGrommet, ApplyRealGeometryVolumeAndFaces)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 5.0);
    const int    stockFaces = stock->faceCount();
    const double stockVol   = volumeOf(stock->shape());

    skill::cable_grommet_pass_through::Input in;
    in.entry_face      = skill::FaceByNormal{ gp_Dir(0, 0, 1), 5.0, "largest" };
    in.axis_dir        = gp_Dir(0, 0, -1);
    in.position_x_mm   = 30.0;
    in.position_y_mm   = 30.0;
    in.grommet_size    = "GR-8";

    auto out = skill::cable_grommet_pass_through::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    // GR-8: bore Ø 8.0 (r=4), through 5 mm → π·16·5 = 251.3
    // Plus retention groove annulus: (r=4.5)²−(r=4)² = 20.25−16 = 4.25; × π × 0.7 ≈ 9.34
    // Plus chamfer cone frustum ≈ (1/3)π·h·(R²+Rr+r²) ≈ small ≈ 5.8
    // Total ≈ 266
    const double rBore = 4.0;
    const double rGroove = 4.5;
    const double thickness = 5.0;
    const double approx =
        M_PI * rBore * rBore * thickness +
        M_PI * (rGroove * rGroove - rBore * rBore) * 0.7;
    const double removed = stockVol - volumeOf(out.workpiece->shape());
    EXPECT_NEAR(removed, approx, approx * 0.20);

    EXPECT_GT(out.workpiece->faceCount(), stockFaces);
}

// ─── T2. validate rejects unknown grommet size + apply throws ───────────
TEST(SkillCableGrommet, ValidateRejectsUnknownSpec)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 5.0);

    skill::cable_grommet_pass_through::Input in;
    in.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 30.0;
    in.position_y_mm = 30.0;
    in.grommet_size  = "GR-XXL";   // unknown

    auto r = skill::cable_grommet_pass_through::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool foundSpec = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-CONN-SIZE") { foundSpec = true; break; }
    EXPECT_TRUE(foundSpec);

    EXPECT_THROW(skill::cable_grommet_pass_through::apply(*stock, in),
                 skill::SkillError);
}

// ─── T3. signature carries spec key + is_compound + 3 subs ──────────────
TEST(SkillCableGrommet, SignatureCarriesSpecAndCompoundFlag)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 6.0);

    skill::cable_grommet_pass_through::Input in;
    in.entry_face    = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.axis_dir      = gp_Dir(0, 0, -1);
    in.position_x_mm = 30.0;
    in.position_y_mm = 30.0;
    in.grommet_size  = "GR-10";

    auto out = skill::cable_grommet_pass_through::apply(*stock, in);
    EXPECT_TRUE(out.signature.pattern.at("is_compound").get<bool>());
    EXPECT_EQ(out.signature.pattern.at("kind").get<std::string>(),
              std::string("cable_grommet_pass_through"));
    EXPECT_EQ(out.signature.pattern.at("grommet_size").get<std::string>(),
              std::string("GR-10"));
    EXPECT_EQ(out.signature.pattern.at("subfeature_count").get<int>(), 3);
}

// ─── T4. recognize returns ≥1 candidate with the right spec key ─────────
TEST(SkillCableGrommet, RecognizeReturnsCandidate)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 5.0);

    skill::cable_grommet_pass_through::Input in;
    in.entry_face    = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.axis_dir      = gp_Dir(0, 0, -1);
    in.position_x_mm = 30.0;
    in.position_y_mm = 30.0;
    in.grommet_size  = "GR-12";

    auto out = skill::cable_grommet_pass_through::apply(*stock, in);
    auto cands = skill::cable_grommet_pass_through::recognize(*out.workpiece);
    ASSERT_FALSE(cands.empty());
    EXPECT_DOUBLE_EQ(cands.front().confidence, 1.0);
    EXPECT_EQ(cands.front().skill_id,
              std::string("cable_grommet_pass_through"));
    EXPECT_EQ(
        cands.front().recovered_params.at("grommet_size").get<std::string>(),
        std::string("GR-12"));
}

// ─── T5. SPEC TABLE: GR-16 has larger bore than GR-6 ────────────────────
TEST(SkillCableGrommet, SpecTableBoreDiameterScalesWithSize)
{
    auto stock = skill::createCuboidStock(80.0, 80.0, 6.0);

    skill::cable_grommet_pass_through::Input small_in;
    small_in.entry_face    = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    small_in.axis_dir      = gp_Dir(0, 0, -1);
    small_in.position_x_mm = 20.0;
    small_in.position_y_mm = 20.0;
    small_in.grommet_size  = "GR-6";
    auto small_out = skill::cable_grommet_pass_through::apply(*stock, small_in);

    skill::cable_grommet_pass_through::Input large_in;
    large_in.entry_face    = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    large_in.axis_dir      = gp_Dir(0, 0, -1);
    large_in.position_x_mm = 20.0;
    large_in.position_y_mm = 20.0;
    large_in.grommet_size  = "GR-16";
    auto large_out = skill::cable_grommet_pass_through::apply(*stock, large_in);

    // GR-6 bore = 6.0 mm; GR-16 = 16.0 mm
    EXPECT_NEAR(small_out.signature.pattern.at("bore_dia_mm").get<double>(),
                6.0, 1e-3);
    EXPECT_NEAR(large_out.signature.pattern.at("bore_dia_mm").get<double>(),
                16.0, 1e-3);
    EXPECT_GT(large_out.signature.pattern.at("bore_dia_mm").get<double>(),
              small_out.signature.pattern.at("bore_dia_mm").get<double>());
}
