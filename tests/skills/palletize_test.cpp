// @lat: [[process/test-strategy#skill round-trip]]
//
// palletize — 5-case sweep covering identity geometry, DFM rejection of
// non-positive rows/cols/layers, metadata replay, and parts_per_pallet
// arithmetic.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/palletize.hpp"

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

}  // namespace

// ── 1. Apply: geometry is COMPLETELY unchanged ───────────────────────────
TEST(SkillPalletize, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();

    skill::palletize::Input in;
    in.rows             = 4;
    in.cols             = 4;
    in.layers           = 3;
    in.pallet_length_mm = 1200.0;
    in.pallet_width_mm  = 1000.0;

    auto out = skill::palletize::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_EQ(out.signature.skill_id, std::string("palletize"));
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ── 2. DFM: rows / cols / layers < 1 or non-positive pallet size rejected ─
TEST(SkillPalletize, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::palletize::Input zeroRows;
    zeroRows.rows = 0; zeroRows.cols = 4; zeroRows.layers = 3;
    zeroRows.pallet_length_mm = 1200.0; zeroRows.pallet_width_mm = 1000.0;
    EXPECT_FALSE(skill::palletize::validate(*stock, zeroRows).passed);
    EXPECT_THROW(skill::palletize::apply(*stock, zeroRows), skill::SkillError);

    skill::palletize::Input zeroCols = zeroRows;
    zeroCols.rows = 4; zeroCols.cols = 0;
    EXPECT_FALSE(skill::palletize::validate(*stock, zeroCols).passed);

    skill::palletize::Input zeroLayers = zeroRows;
    zeroLayers.rows = 4; zeroLayers.cols = 4; zeroLayers.layers = 0;
    EXPECT_FALSE(skill::palletize::validate(*stock, zeroLayers).passed);

    skill::palletize::Input zeroLen;
    zeroLen.rows = 4; zeroLen.cols = 4; zeroLen.layers = 3;
    zeroLen.pallet_length_mm = 0.0; zeroLen.pallet_width_mm = 1000.0;
    EXPECT_FALSE(skill::palletize::validate(*stock, zeroLen).passed);

    skill::palletize::Input negWid = zeroLen;
    negWid.pallet_length_mm = 1200.0; negWid.pallet_width_mm = -100.0;
    EXPECT_FALSE(skill::palletize::validate(*stock, negWid).passed);
}

// ── 3. Signature records kind + pallet_pattern + pallet_size ─────────────
TEST(SkillPalletize, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::palletize::Input in;
    in.rows             = 5;
    in.cols             = 3;
    in.layers           = 2;
    in.pallet_length_mm = 800.0;
    in.pallet_width_mm  = 600.0;

    auto out = skill::palletize::apply(*stock, in);
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("palletize"));
    EXPECT_EQ(out.signature.pattern["pallet_pattern"]["rows"].get<int>(),   5);
    EXPECT_EQ(out.signature.pattern["pallet_pattern"]["cols"].get<int>(),   3);
    EXPECT_EQ(out.signature.pattern["pallet_pattern"]["layers"].get<int>(), 2);
    EXPECT_EQ(out.signature.pattern["parts_per_pallet"].get<int>(), 30);
    EXPECT_EQ(out.signature.pattern["pallet_size"].size(), 2u);
    EXPECT_NEAR(out.signature.pattern["pallet_size"][0].get<double>(),
                800.0, 1e-9);
    EXPECT_NEAR(out.signature.pattern["pallet_size"][1].get<double>(),
                600.0, 1e-9);
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
}

// ── 4. Recognize via metadata replay ─────────────────────────────────────
TEST(SkillPalletize, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::palletize::Input in;
    in.rows             = 3;
    in.cols             = 3;
    in.layers           = 2;
    in.pallet_length_mm = 1200.0;
    in.pallet_width_mm  = 1000.0;

    auto out = skill::palletize::apply(*stock, in);
    auto cands = skill::palletize::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["rows"].get<int>(),   3);
    EXPECT_EQ(cands[0].recovered_params["cols"].get<int>(),   3);
    EXPECT_EQ(cands[0].recovered_params["layers"].get<int>(), 2);

    skill::Workpiece raw(out.workpiece->shape());
    EXPECT_TRUE(skill::palletize::recognize(raw).empty());
}

// ── 5. parts_per_pallet = rows × cols × layers (specific assertion) ──────
TEST(SkillPalletize, PartsPerPalletIsProduct)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::palletize::Input in;
    in.rows             = 6;
    in.cols             = 8;
    in.layers           = 4;
    in.pallet_length_mm = 1200.0;
    in.pallet_width_mm  = 1000.0;

    auto out = skill::palletize::apply(*stock, in);
    EXPECT_EQ(out.signature.pattern["parts_per_pallet"].get<int>(),
              6 * 8 * 4);    // 192

    // est_cycle_time_s = 3 * 192 = 576 s
    EXPECT_NEAR(out.signature.tooling.est_cycle_time_s, 576.0, 1e-9);

    // Oversize pallet (1500 × 1200) → info finding fires
    skill::palletize::Input big = in;
    big.pallet_length_mm = 1500.0;
    big.pallet_width_mm  = 1200.0;
    auto r = skill::palletize::validate(*stock, big);
    EXPECT_TRUE(r.passed);
    bool info = false;
    for (const auto& f : r.findings) {
        if (f.code == "DFM-PALLET-SIZE" && f.severity == "info") {
            info = true; break;
        }
    }
    EXPECT_TRUE(info);
}
