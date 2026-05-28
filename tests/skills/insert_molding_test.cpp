// @lat: [[process/test-strategy#skill round-trip]]
//
// insert_molding — additive thin-shell encapsulation of an insert.
//
// Cases (5):
//   1. apply handles input + adds volume (shell wraps the bbox).
//   2. validate rejects bad input (shell_thickness ≤ 0).
//   3. signature records kind + insert_material + resin_material.
//   4. recognize() replays metadata.
//   5. specific assertion: shell_thickness ≥ shell_thickness_mm × insert bbox
//      — i.e. new bbox extents grow by exactly 2 × shell_thickness on each axis.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/insert_molding.hpp"

#include <BRepBndLib.hxx>
#include <BRepGProp.hxx>
#include <Bnd_Box.hxx>
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

// ─── 1. Apply handles input — adds shell volume ────────────────────────────
TEST(SkillInsertMolding, ApplyHandlesInput)
{
    auto stock = skill::createCuboidStock(20.0, 20.0, 10.0, "brass_360");
    const double insertVol = volumeOf(stock->shape());

    skill::insert_molding::Input in;
    in.insert_material    = "brass_360";
    in.resin_material     = "abs";
    in.shell_thickness_mm = 2.0;

    auto out = skill::insert_molding::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    const double newVol = volumeOf(out.workpiece->shape());
    // Outer bbox is (20+4) × (20+4) × (10+4) = 24 × 24 × 14 = 8064.
    // Shell vol ≈ 8064 - 4000 = 4064.
    const double expectedOuter = 24.0 * 24.0 * 14.0;
    EXPECT_NEAR(newVol, expectedOuter, expectedOuter * 0.01)
        << "fused insert+shell should fill the expanded bbox";
    EXPECT_GT(newVol, insertVol);
}

// ─── 2. Validate rejects bad input ─────────────────────────────────────────
TEST(SkillInsertMolding, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(20.0, 20.0, 10.0);

    // Sub-zero thickness.
    skill::insert_molding::Input bad;
    bad.insert_material    = "brass_360";
    bad.resin_material     = "abs";
    bad.shell_thickness_mm = 0.0;

    auto r = skill::insert_molding::validate(*stock, bad);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-INSERT-SHELL" && f.severity == "error") {
            found = true;
            break;
        }
    EXPECT_TRUE(found);
    EXPECT_THROW(skill::insert_molding::apply(*stock, bad), skill::SkillError);

    // Missing material.
    skill::insert_molding::Input badMat;
    badMat.insert_material    = "";        // missing
    badMat.resin_material     = "abs";
    badMat.shell_thickness_mm = 2.0;
    auto r2 = skill::insert_molding::validate(*stock, badMat);
    EXPECT_FALSE(r2.passed);
}

// ─── 3. Signature records kind + materials ─────────────────────────────────
TEST(SkillInsertMolding, SignatureRecordsKindAndMaterials)
{
    auto stock = skill::createCuboidStock(20.0, 20.0, 10.0, "brass_360");

    skill::insert_molding::Input in;
    in.insert_material    = "brass_360";
    in.resin_material     = "abs";
    in.shell_thickness_mm = 2.0;

    auto out = skill::insert_molding::apply(*stock, in);

    EXPECT_EQ(out.signature.skill_id, std::string("insert_molding"));
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("insert_molding"));
    EXPECT_EQ(out.signature.pattern["insert_material"].get<std::string>(),
              std::string("brass_360"));
    EXPECT_EQ(out.signature.pattern["resin_material"].get<std::string>(),
              std::string("abs"));
    EXPECT_NEAR(out.signature.pattern["shell_thickness_mm"].get<double>(),
                2.0, 1e-9);
    EXPECT_TRUE(out.signature.pattern["additive"].get<bool>());
    EXPECT_TRUE(out.signature.pattern["encapsulates_insert"].get<bool>());
    EXPECT_EQ(out.signature.tooling.tool_type, std::string("insert_mold"));
}

// ─── 4. Recognize via metadata replay ──────────────────────────────────────
TEST(SkillInsertMolding, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(20.0, 20.0, 10.0);

    skill::insert_molding::Input in;
    in.insert_material    = "stainless_316";
    in.resin_material     = "nylon";
    in.shell_thickness_mm = 1.5;

    auto out = skill::insert_molding::apply(*stock, in);
    auto cands = skill::insert_molding::recognize(*out.workpiece);

    ASSERT_EQ(cands.size(), 1u);
    EXPECT_GT(cands[0].confidence, 0.99);
    EXPECT_EQ(cands[0].recovered_params["insert_material"].get<std::string>(),
              std::string("stainless_316"));
    EXPECT_EQ(cands[0].recovered_params["resin_material"].get<std::string>(),
              std::string("nylon"));

    // Stripping metadata kills recognition (geometry alone is ambiguous).
    skill::Workpiece raw(out.workpiece->shape());
    auto cands2 = skill::insert_molding::recognize(raw);
    EXPECT_TRUE(cands2.empty());
}

// ─── 5. Specific assertion: bbox grows by exactly 2 × shell_thickness ─────
TEST(SkillInsertMolding, BboxGrowsByTwiceShellThickness)
{
    auto stock = skill::createCuboidStock(20.0, 20.0, 10.0);

    // Insert bbox extents.
    Bnd_Box bbIn;
    BRepBndLib::AddOptimal(stock->shape(), bbIn);
    double ixMin, iyMin, izMin, ixMax, iyMax, izMax;
    bbIn.Get(ixMin, iyMin, izMin, ixMax, iyMax, izMax);
    const double idx = ixMax - ixMin;
    const double idy = iyMax - iyMin;
    const double idz = izMax - izMin;

    skill::insert_molding::Input in;
    in.insert_material    = "brass_360";
    in.resin_material     = "abs";
    in.shell_thickness_mm = 3.0;
    auto out = skill::insert_molding::apply(*stock, in);

    Bnd_Box bbOut;
    BRepBndLib::AddOptimal(out.workpiece->shape(), bbOut);
    double oxMin, oyMin, ozMin, oxMax, oyMax, ozMax;
    bbOut.Get(oxMin, oyMin, ozMin, oxMax, oyMax, ozMax);
    const double odx = oxMax - oxMin;
    const double ody = oyMax - oyMin;
    const double odz = ozMax - ozMin;

    EXPECT_NEAR(odx - idx, 2.0 * in.shell_thickness_mm, 0.05);
    EXPECT_NEAR(ody - idy, 2.0 * in.shell_thickness_mm, 0.05);
    EXPECT_NEAR(odz - idz, 2.0 * in.shell_thickness_mm, 0.05);
}
