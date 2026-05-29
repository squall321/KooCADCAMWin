// @lat: [[process/test-strategy#skill round-trip]]
//
// block_assembly — shipyard block fit-up + weld.  Metadata-only contract.
// Covers:
//   1. apply preserves geometry verbatim (clone + signature stamp)
//   2. validate rejects bad input (mass_t <= 0)
//   3. signature records kind + block_id + mass_t + weld_meters
//   4. recognize round-trips from stamped metadata
//   5. mass_t > 4000 t emits DFM-BLOCK-MASS info (grand-block — goliath needed)

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/block_assembly.hpp"

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

// ─── 1. Apply clones geometry unchanged ──────────────────────────────────
TEST(SkillBlockAssembly, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0, "marine_steel");
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    skill::block_assembly::Input in;
    in.block_id    = "BLK-101";
    in.mass_t      = 250.0;
    in.weld_meters = 600.0;

    auto out = skill::block_assembly::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. validate rejects bad input ───────────────────────────────────────
TEST(SkillBlockAssembly, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0, "marine_steel");

    skill::block_assembly::Input in;
    in.block_id    = "BLK-101";
    in.mass_t      = 0.0;     // invalid
    in.weld_meters = 100.0;

    auto r = skill::block_assembly::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-INPUT"));
    EXPECT_THROW(skill::block_assembly::apply(*stock, in), skill::SkillError);

    // Negative weld_meters also blocks.
    in.mass_t      = 100.0;
    in.weld_meters = -1.0;
    auto r2 = skill::block_assembly::validate(*stock, in);
    EXPECT_FALSE(r2.passed);
}

// ─── 3. signature records kind + key params ──────────────────────────────
TEST(SkillBlockAssembly, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0, "marine_steel");

    skill::block_assembly::Input in;
    in.block_id    = "BLK-7A";
    in.mass_t      = 320.0;
    in.weld_meters = 750.0;

    auto out = skill::block_assembly::apply(*stock, in);
    EXPECT_EQ(out.signature.skill_id, std::string("block_assembly"));
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("block_assembly"));
    EXPECT_EQ(out.signature.pattern["block_id"].get<std::string>(),
              std::string("BLK-7A"));
    EXPECT_NEAR(out.signature.pattern["mass_t"].get<double>(), 320.0, 1e-9);
    EXPECT_NEAR(out.signature.pattern["weld_meters"].get<double>(), 750.0, 1e-9);
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
}

// ─── 4. recognize replays metadata ───────────────────────────────────────
TEST(SkillBlockAssembly, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0, "marine_steel");

    skill::block_assembly::Input in;
    in.block_id    = "BLK-9";
    in.mass_t      = 180.0;
    in.weld_meters = 420.0;
    auto out = skill::block_assembly::apply(*stock, in);

    auto cands = skill::block_assembly::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["block_id"].get<std::string>(),
              std::string("BLK-9"));

    skill::Workpiece raw(out.workpiece->shape());
    auto cands2 = skill::block_assembly::recognize(raw);
    EXPECT_TRUE(cands2.empty());
}

// ─── 5. Grand block triggers DFM-BLOCK-MASS info but does NOT block ──────
TEST(SkillBlockAssembly, GrandBlockEmitsInfoNotError)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0, "marine_steel");

    skill::block_assembly::Input in;
    in.block_id    = "GBLK-mega";
    in.mass_t      = 4500.0;
    in.weld_meters = 8000.0;

    auto r = skill::block_assembly::validate(*stock, in);
    EXPECT_TRUE(r.passed)
        << "grand block (mass > 4000 t) should emit info, not block";
    EXPECT_TRUE(hasFinding(r, "DFM-BLOCK-MASS"));
    EXPECT_NO_THROW(skill::block_assembly::apply(*stock, in));
}
