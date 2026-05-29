// @lat: [[process/test-strategy#skill round-trip]]
//
// inventory_count — cycle-count / physical inventory metadata skill.
// 5-case sweep:
//   1. apply clones geometry unchanged
//   2. validate rejects bad input
//   3. signature records kind + lot_id + expected_qty + actual_qty + variance_pct
//   4. recognize metadata replay
//   5. specific: disposition transitions (ok / shortage / overage / critical)
//      and DFM info findings at the right thresholds

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/inventory_count.hpp"

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

bool hasFinding(const skill::DFMReport& r, const std::string& code)
{
    for (const auto& f : r.findings) if (f.code == code) return true;
    return false;
}

skill::inventory_count::Input goodInput()
{
    skill::inventory_count::Input in;
    in.lot_id       = "LOT-CYCLE-001";
    in.expected_qty = 100;
    in.actual_qty   = 100;
    return in;
}

}  // namespace

// ─── 1. Apply clones geometry unchanged ──────────────────────────────────
TEST(SkillInventoryCount, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    auto out = skill::inventory_count::apply(*stock, goodInput());
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. Validate rejects bad input ───────────────────────────────────────
TEST(SkillInventoryCount, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    // Empty lot_id → error.
    auto in1 = goodInput();
    in1.lot_id = "";
    auto r1 = skill::inventory_count::validate(*stock, in1);
    EXPECT_FALSE(r1.passed);
    EXPECT_TRUE(hasFinding(r1, "DFM-INPUT"));
    EXPECT_THROW(skill::inventory_count::apply(*stock, in1),
                 skill::SkillError);

    // Non-positive expected_qty → error.
    auto in2 = goodInput();
    in2.expected_qty = 0;
    auto r2 = skill::inventory_count::validate(*stock, in2);
    EXPECT_FALSE(r2.passed);
    EXPECT_TRUE(hasFinding(r2, "DFM-INPUT"));

    // Negative actual_qty → error.
    auto in3 = goodInput();
    in3.actual_qty = -5;
    auto r3 = skill::inventory_count::validate(*stock, in3);
    EXPECT_FALSE(r3.passed);
    EXPECT_TRUE(hasFinding(r3, "DFM-INPUT"));
}

// ─── 3. Signature records kind + key params ──────────────────────────────
TEST(SkillInventoryCount, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::inventory_count::Input in;
    in.lot_id       = "LOT-2026-Z9";
    in.expected_qty = 500;
    in.actual_qty   = 495;          // -1.0% variance, no info findings

    auto out = skill::inventory_count::apply(*stock, in);

    EXPECT_EQ(out.signature.skill_id, std::string("inventory_count"));
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("inventory_count"));
    EXPECT_EQ(out.signature.pattern["lot_id"].get<std::string>(),
              std::string("LOT-2026-Z9"));
    EXPECT_EQ(out.signature.pattern["expected_qty"].get<int>(), 500);
    EXPECT_EQ(out.signature.pattern["actual_qty"].get<int>(),   495);
    EXPECT_EQ(out.signature.pattern["variance_qty"].get<int>(),  -5);
    EXPECT_NEAR(out.signature.pattern["variance_pct"].get<double>(),
                -1.0, 1e-9);
    EXPECT_FALSE(out.signature.pattern["geometric_change"].get<bool>());
}

// ─── 4. Recognize metadata replay ────────────────────────────────────────
TEST(SkillInventoryCount, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);
    auto out = skill::inventory_count::apply(*stock, goodInput());

    auto cands = skill::inventory_count::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].skill_id, std::string("inventory_count"));
    EXPECT_EQ(cands[0].recovered_params["lot_id"].get<std::string>(),
              std::string("LOT-CYCLE-001"));
    EXPECT_EQ(cands[0].recovered_params["expected_qty"].get<int>(), 100);
    EXPECT_EQ(cands[0].recovered_params["actual_qty"].get<int>(),   100);

    // Stripped workpiece → recognize empty.
    skill::Workpiece raw(out.workpiece->shape(), out.workpiece->material());
    EXPECT_TRUE(skill::inventory_count::recognize(raw).empty());
}

// ─── 5. Specific: disposition transitions across thresholds ──────────────
TEST(SkillInventoryCount, DispositionAndFindingsByThreshold)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    // Exact match — disposition "ok", no info findings.
    auto ok = goodInput();
    auto rOk = skill::inventory_count::validate(*stock, ok);
    EXPECT_TRUE(rOk.passed);
    EXPECT_FALSE(hasFinding(rOk, "DFM-IC-SHORTAGE"));
    EXPECT_FALSE(hasFinding(rOk, "DFM-IC-OVERAGE"));
    EXPECT_FALSE(hasFinding(rOk, "DFM-IC-CRITICAL"));
    auto outOk = skill::inventory_count::apply(*stock, ok);
    EXPECT_EQ(outOk.signature.pattern["disposition"].get<std::string>(),
              std::string("ok"));

    // Shortage (97/100 = -3%) → DFM-IC-SHORTAGE, no CRITICAL.
    skill::inventory_count::Input shortInp;
    shortInp.lot_id       = "LOT-SHORT";
    shortInp.expected_qty = 100;
    shortInp.actual_qty   = 97;
    auto rS = skill::inventory_count::validate(*stock, shortInp);
    EXPECT_TRUE(rS.passed);
    EXPECT_TRUE(hasFinding(rS, "DFM-IC-SHORTAGE"));
    EXPECT_FALSE(hasFinding(rS, "DFM-IC-CRITICAL"));
    auto outS = skill::inventory_count::apply(*stock, shortInp);
    EXPECT_EQ(outS.signature.pattern["disposition"].get<std::string>(),
              std::string("shortage"));
    EXPECT_NEAR(outS.signature.pattern["variance_pct"].get<double>(),
                -3.0, 1e-9);

    // Overage (105/100 = +5%) → DFM-IC-OVERAGE, no CRITICAL.
    skill::inventory_count::Input overInp;
    overInp.lot_id       = "LOT-OVER";
    overInp.expected_qty = 100;
    overInp.actual_qty   = 105;
    auto rO = skill::inventory_count::validate(*stock, overInp);
    EXPECT_TRUE(rO.passed);
    EXPECT_TRUE(hasFinding(rO, "DFM-IC-OVERAGE"));
    EXPECT_FALSE(hasFinding(rO, "DFM-IC-CRITICAL"));
    auto outO = skill::inventory_count::apply(*stock, overInp);
    EXPECT_EQ(outO.signature.pattern["disposition"].get<std::string>(),
              std::string("overage"));

    // Critical shortage (80/100 = -20%) → SHORTAGE + CRITICAL.
    skill::inventory_count::Input crit;
    crit.lot_id       = "LOT-CRIT";
    crit.expected_qty = 100;
    crit.actual_qty   = 80;
    auto rC = skill::inventory_count::validate(*stock, crit);
    EXPECT_TRUE(rC.passed);
    EXPECT_TRUE(hasFinding(rC, "DFM-IC-SHORTAGE"));
    EXPECT_TRUE(hasFinding(rC, "DFM-IC-CRITICAL"));
    auto outC = skill::inventory_count::apply(*stock, crit);
    EXPECT_EQ(outC.signature.pattern["disposition"].get<std::string>(),
              std::string("critical_shortage"));
    EXPECT_NEAR(std::fabs(outC.signature.pattern["variance_pct"].get<double>()),
                20.0, 1e-9);
}
