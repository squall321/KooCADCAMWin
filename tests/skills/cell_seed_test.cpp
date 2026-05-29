// @lat: [[process/test-strategy#skill round-trip]]
//
// cell_seed — seed cells onto an existing scaffold.  Metadata-only.
// Cases (5):
//   1. ApplyClonesGeometryUnchanged
//   2. ValidateRejectsBadInput
//   3. SignatureRecordsKindAndParams
//   4. RecognizeMetadataReplay
//   5. LowViabilityEmitsInfoButProceeds

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/cell_seed.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

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

// ─── 1. Apply clones geometry unchanged ───────────────────────────────────
TEST(SkillCellSeed, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(25.0, 25.0, 5.0);
    const double volBefore = volumeOf(stock->shape());
    const int faceBefore   = stock->faceCount();
    const int edgeBefore   = stock->edgeCount();

    skill::cell_seed::Input in;
    in.cell_type         = "fibroblast";
    in.density_cells_cm2 = 1.0e5;
    in.viability_pct     = 95.0;

    auto out = skill::cell_seed::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. Validate rejects bad input ─────────────────────────────────────────
TEST(SkillCellSeed, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(20.0, 20.0, 3.0);

    // Empty cell type.
    skill::cell_seed::Input empty;
    empty.cell_type         = "";
    empty.density_cells_cm2 = 1.0e5;
    empty.viability_pct     = 95.0;
    {
        auto r = skill::cell_seed::validate(*stock, empty);
        EXPECT_FALSE(r.passed);
        EXPECT_TRUE(hasFinding(r, "DFM-INPUT"));
    }
    EXPECT_THROW(skill::cell_seed::apply(*stock, empty), skill::SkillError);

    // Out-of-range density.
    skill::cell_seed::Input lowDens;
    lowDens.cell_type         = "msc";
    lowDens.density_cells_cm2 = 1.0e1;       // < 1e2
    lowDens.viability_pct     = 95.0;
    {
        auto r = skill::cell_seed::validate(*stock, lowDens);
        EXPECT_FALSE(r.passed);
        EXPECT_TRUE(hasFinding(r, "DFM-SEED-DENSITY"));
    }

    // Out-of-range viability.
    skill::cell_seed::Input badVia;
    badVia.cell_type         = "msc";
    badVia.density_cells_cm2 = 1.0e5;
    badVia.viability_pct     = 120.0;        // > 100
    {
        auto r = skill::cell_seed::validate(*stock, badVia);
        EXPECT_FALSE(r.passed);
        EXPECT_TRUE(hasFinding(r, "DFM-SEED-VIABILITY"));
    }
}

// ─── 3. Signature records kind + key params ────────────────────────────────
TEST(SkillCellSeed, SignatureRecordsKindAndParams)
{
    auto stock = skill::createCuboidStock(15.0, 15.0, 3.0);

    skill::cell_seed::Input in;
    in.cell_type         = "hepatocyte";
    in.density_cells_cm2 = 2.5e5;
    in.viability_pct     = 92.0;

    auto out = skill::cell_seed::apply(*stock, in);

    EXPECT_EQ(out.signature.skill_id, std::string("cell_seed"));
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("cell_seed"));
    EXPECT_EQ(out.signature.pattern["cell_type"].get<std::string>(),
              std::string("hepatocyte"));
    EXPECT_NEAR(out.signature.pattern["density_cells_cm2"].get<double>(),
                2.5e5, 1e-3);
    EXPECT_NEAR(out.signature.pattern["viability_pct"].get<double>(),
                92.0, 1e-9);
    EXPECT_FALSE(out.signature.pattern["geometric_change"].get<bool>());
}

// ─── 4. Recognize metadata replay ──────────────────────────────────────────
TEST(SkillCellSeed, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(15.0, 15.0, 3.0);

    skill::cell_seed::Input in;
    in.cell_type         = "endothelial";
    in.density_cells_cm2 = 1.5e5;
    in.viability_pct     = 88.0;

    auto out   = skill::cell_seed::apply(*stock, in);
    auto cands = skill::cell_seed::recognize(*out.workpiece);

    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["cell_type"].get<std::string>(),
              std::string("endothelial"));
    EXPECT_NEAR(cands[0].recovered_params["density_cells_cm2"].get<double>(),
                1.5e5, 1e-3);

    // Stripped of metadata → recognize empty.
    skill::Workpiece raw(out.workpiece->shape());
    auto cands2 = skill::cell_seed::recognize(raw);
    EXPECT_TRUE(cands2.empty());
}

// ─── 5. Specific: low viability emits info but proceeds ────────────────────
TEST(SkillCellSeed, LowViabilityEmitsInfoButProceeds)
{
    auto stock = skill::createCuboidStock(10.0, 10.0, 2.0);

    skill::cell_seed::Input in;
    in.cell_type         = "fibroblast";
    in.density_cells_cm2 = 1.0e5;
    in.viability_pct     = 50.0;        // < 70 → info

    auto r = skill::cell_seed::validate(*stock, in);
    EXPECT_TRUE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-SEED-VIABILITY-LOW"));

    auto out = skill::cell_seed::apply(*stock, in);
    EXPECT_EQ(out.signature.tooling.tool_type,
              std::string("automated_pipette"));
}
