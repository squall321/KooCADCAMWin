// @lat: [[process/test-strategy#skill round-trip]]
//
// bioink_print — REAL geometric layered scaffold rebuild.
// Cases (5):
//   1. ApplyBuildsCylindricalScaffold
//   2. ValidateRejectsBadInput
//   3. SignatureRecordsScaffoldDims
//   4. RecognizeGeometricHeuristic
//   5. UnknownBioinkEmitsInfoButProceeds

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/bioink_print.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using namespace koocadcam;

namespace {

double volumeOf(const TopoDS_Shape& s)
{
    GProp_GProps p;
    BRepGProp::VolumeProperties(s, p);
    return p.Mass();
}

bool hasFinding(const skill::DFMReport& r, const std::string& code)
{
    for (const auto& f : r.findings) if (f.code == code) return true;
    return false;
}

}  // namespace

// ─── 1. Apply rebuilds blank as a cylindrical scaffold ─────────────────────
//
// Input bbox 20 × 20 × 5 mm; scaffold dia = min(20, 20) = 20 mm; scaffold
// height = min(20 default layers × 0.2 mm = 4 mm,  bbox 5 mm) → 4 mm.
// Expected volume = π·10²·4 = 1256.64 mm³.
// Right cylinder has exactly 3 topological faces.
TEST(SkillBioinkPrint, ApplyBuildsCylindricalScaffold)
{
    auto stock = skill::createCuboidStock(20.0, 20.0, 5.0, "pla");

    skill::bioink_print::Input in;
    in.bioink_type         = "gelma";
    in.cell_density_per_ml = 1.0e6;
    in.layer_height_um     = 200.0;

    auto out = skill::bioink_print::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_EQ(out.workpiece->faceCount(), 3);
    EXPECT_EQ(out.workpiece->features().size(), 1u);

    const double expectedV = M_PI * 100.0 * 4.0;        // 1256.637 mm³
    EXPECT_NEAR(volumeOf(out.workpiece->shape()), expectedV, 1.0);

    // Additive convention: stock_removed_mm3 is negative & equals -scaffold V.
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, -expectedV, 1.0);
}

// ─── 2. Validate rejects bad input ─────────────────────────────────────────
TEST(SkillBioinkPrint, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(20.0, 20.0, 5.0);

    skill::bioink_print::Input tooThin;
    tooThin.bioink_type         = "gelma";
    tooThin.cell_density_per_ml = 1.0e6;
    tooThin.layer_height_um     = 10.0;       // < 50 μm
    {
        auto r = skill::bioink_print::validate(*stock, tooThin);
        EXPECT_FALSE(r.passed);
        EXPECT_TRUE(hasFinding(r, "DFM-BIOINK-LAYER"));
    }
    EXPECT_THROW(skill::bioink_print::apply(*stock, tooThin), skill::SkillError);

    skill::bioink_print::Input tooDense;
    tooDense.bioink_type         = "gelma";
    tooDense.cell_density_per_ml = 1.0e10;    // > 1e8
    tooDense.layer_height_um     = 200.0;
    {
        auto r = skill::bioink_print::validate(*stock, tooDense);
        EXPECT_FALSE(r.passed);
        EXPECT_TRUE(hasFinding(r, "DFM-BIOINK-DENSITY"));
    }

    skill::bioink_print::Input empty;
    empty.bioink_type         = "";
    empty.cell_density_per_ml = 1.0e6;
    empty.layer_height_um     = 200.0;
    {
        auto r = skill::bioink_print::validate(*stock, empty);
        EXPECT_FALSE(r.passed);
        EXPECT_TRUE(hasFinding(r, "DFM-INPUT"));
    }
}

// ─── 3. Signature records scaffold geometry params ─────────────────────────
TEST(SkillBioinkPrint, SignatureRecordsScaffoldDims)
{
    auto stock = skill::createCuboidStock(15.0, 15.0, 6.0);

    skill::bioink_print::Input in;
    in.bioink_type         = "collagen";
    in.cell_density_per_ml = 5.0e6;
    in.layer_height_um     = 300.0;

    auto out = skill::bioink_print::apply(*stock, in);

    EXPECT_EQ(out.signature.skill_id, std::string("bioink_print"));
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("bioink_print"));
    EXPECT_EQ(out.signature.pattern["bioink_type"].get<std::string>(),
              std::string("collagen"));
    EXPECT_TRUE(out.signature.pattern["geometric_change"].get<bool>());

    EXPECT_NEAR(out.signature.pattern["scaffold_dia_mm"].get<double>(),
                15.0, 1e-6);
    // Default 20 layers × 0.3 mm = 6 mm — matches bbox cap.
    EXPECT_EQ(out.signature.pattern["layer_count"].get<int>(), 20);
    EXPECT_NEAR(out.signature.pattern["scaffold_height_mm"].get<double>(),
                6.0, 1e-6);
    const double expectedV = M_PI * 7.5 * 7.5 * 6.0;     // 1060.29
    EXPECT_NEAR(out.signature.pattern["scaffold_volume_mm3"].get<double>(),
                expectedV, 1e-3);
}

// ─── 4. Recognize geometric heuristic on a raw scaffold ────────────────────
TEST(SkillBioinkPrint, RecognizeGeometricHeuristic)
{
    auto stock = skill::createCuboidStock(15.0, 15.0, 6.0);

    skill::bioink_print::Input in;
    in.bioink_type         = "alginate";
    in.cell_density_per_ml = 2.0e6;
    in.layer_height_um     = 250.0;

    auto out   = skill::bioink_print::apply(*stock, in);

    // Metadata replay confidence 1.0.
    auto cands = skill::bioink_print::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["bioink_type"].get<std::string>(),
              std::string("alginate"));

    // Stripped of metadata → geometric heuristic kicks in.
    skill::Workpiece raw(out.workpiece->shape());
    auto cands2 = skill::bioink_print::recognize(raw);
    ASSERT_EQ(cands2.size(), 1u);
    EXPECT_GE(cands2[0].confidence, 0.3);
    EXPECT_LE(cands2[0].confidence, 0.7);
    EXPECT_NEAR(cands2[0].matched_geometry["scaffold_dia_mm"].get<double>(),
                15.0, 1e-6);
}

// ─── 5. Unknown bioink emits info but proceeds ─────────────────────────────
TEST(SkillBioinkPrint, UnknownBioinkEmitsInfoButProceeds)
{
    auto stock = skill::createCuboidStock(10.0, 10.0, 4.0);

    skill::bioink_print::Input in;
    in.bioink_type         = "exotic_bioink_q";
    in.cell_density_per_ml = 1.0e6;
    in.layer_height_um     = 200.0;

    auto r = skill::bioink_print::validate(*stock, in);
    EXPECT_TRUE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-BIOINK-VIABILITY"));

    auto out = skill::bioink_print::apply(*stock, in);
    EXPECT_EQ(out.signature.tooling.tool_type,
              std::string("bioprinter_extruder"));
    EXPECT_GT(out.signature.tooling.tool_dia_mm, 0.0);
}
