// @lat: [[process/test-strategy#skill round-trip]]
//
// batch_release — lot disposition skill.  Covers:
//   1. apply clones geometry unchanged
//   2. validate rejects bad input
//   3. signature records kind + lot_id + lot_size + accept_count + class
//   4. recognize metadata replay
//   5. specific: yield computed correctly + low-yield DFM info finding

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/batch_release.hpp"

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
TEST(SkillBatchRelease, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    skill::batch_release::Input in;
    in.lot_id        = "LOT-2026-001";
    in.lot_size      = 1000;
    in.accept_count  = 995;
    in.release_class = "A";

    auto out = skill::batch_release::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. Validate rejects bad input ───────────────────────────────────────
TEST(SkillBatchRelease, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    // Empty lot_id → error.
    skill::batch_release::Input in;
    in.lot_id        = "";          // INVALID
    in.lot_size      = 100;
    in.accept_count  = 99;
    in.release_class = "A";
    auto r = skill::batch_release::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-BR-LOTID"));
    EXPECT_THROW(skill::batch_release::apply(*stock, in),
                 skill::SkillError);

    // accept_count > lot_size → error.
    skill::batch_release::Input in2;
    in2.lot_id        = "LOT-Z";
    in2.lot_size      = 50;
    in2.accept_count  = 100;
    in2.release_class = "A";
    auto r2 = skill::batch_release::validate(*stock, in2);
    EXPECT_FALSE(r2.passed);
    EXPECT_TRUE(hasFinding(r2, "DFM-INPUT"));

    // Unknown release_class → error.
    skill::batch_release::Input in3;
    in3.lot_id        = "LOT-Y";
    in3.lot_size      = 50;
    in3.accept_count  = 50;
    in3.release_class = "D";        // INVALID
    auto r3 = skill::batch_release::validate(*stock, in3);
    EXPECT_FALSE(r3.passed);
    EXPECT_TRUE(hasFinding(r3, "DFM-BR-CLASS"));
}

// ─── 3. Signature records kind + key params ──────────────────────────────
TEST(SkillBatchRelease, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::batch_release::Input in;
    in.lot_id        = "LOT-2026-XYZ";
    in.lot_size      = 500;
    in.accept_count  = 498;
    in.release_class = "A";

    auto out = skill::batch_release::apply(*stock, in);
    EXPECT_EQ(out.signature.skill_id, std::string("batch_release"));
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("batch_release"));
    EXPECT_EQ(out.signature.pattern["lot_id"].get<std::string>(),
              std::string("LOT-2026-XYZ"));
    EXPECT_EQ(out.signature.pattern["lot_size"].get<int>(),      500);
    EXPECT_EQ(out.signature.pattern["accept_count"].get<int>(),  498);
    EXPECT_EQ(out.signature.pattern["release_class"].get<std::string>(),
              std::string("A"));
    EXPECT_FALSE(out.signature.pattern["geometric_change"].get<bool>());
}

// ─── 4. Recognize metadata replay ────────────────────────────────────────
TEST(SkillBatchRelease, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::batch_release::Input in;
    in.lot_id        = "LOT-REP";
    in.lot_size      = 200;
    in.accept_count  = 199;
    in.release_class = "A";
    auto out = skill::batch_release::apply(*stock, in);

    auto cands = skill::batch_release::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].skill_id, std::string("batch_release"));
    EXPECT_EQ(cands[0].recovered_params["lot_id"].get<std::string>(),
              std::string("LOT-REP"));
    EXPECT_EQ(cands[0].recovered_params["lot_size"].get<int>(), 200);
    EXPECT_EQ(cands[0].recovered_params["release_class"].get<std::string>(),
              std::string("A"));

    // Workpiece without metadata → recognize empty.
    skill::Workpiece raw(out.workpiece->shape(), out.workpiece->material());
    EXPECT_TRUE(skill::batch_release::recognize(raw).empty());
}

// ─── 5. Specific: yield computed correctly + low-yield DFM info ──────────
TEST(SkillBatchRelease, YieldComputedAndLowYieldInfo)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    // High yield (99/100) — no DFM-BR-YIELD-LOW finding.
    skill::batch_release::Input high;
    high.lot_id        = "LOT-HI";
    high.lot_size      = 100;
    high.accept_count  = 99;
    high.release_class = "A";
    auto rH = skill::batch_release::validate(*stock, high);
    EXPECT_TRUE(rH.passed);
    EXPECT_FALSE(hasFinding(rH, "DFM-BR-YIELD-LOW"));
    auto outH = skill::batch_release::apply(*stock, high);
    EXPECT_NEAR(outH.signature.pattern["yield"].get<double>(),
                0.99, 1e-9);

    // Low yield (80/100 = 0.80) + class A — TWO info findings:
    // DFM-BR-YIELD-LOW + DFM-BR-CLASS-MISMATCH.
    skill::batch_release::Input low;
    low.lot_id        = "LOT-LO";
    low.lot_size      = 100;
    low.accept_count  = 80;
    low.release_class = "A";
    auto rL = skill::batch_release::validate(*stock, low);
    EXPECT_TRUE(rL.passed);          // info-only
    EXPECT_TRUE(hasFinding(rL, "DFM-BR-YIELD-LOW"));
    EXPECT_TRUE(hasFinding(rL, "DFM-BR-CLASS-MISMATCH"));
    auto outL = skill::batch_release::apply(*stock, low);
    EXPECT_NEAR(outL.signature.pattern["yield"].get<double>(),
                0.80, 1e-9);

    // Scrap class is accepted by validate even with zero yield.
    skill::batch_release::Input scrap;
    scrap.lot_id        = "LOT-SCRAP";
    scrap.lot_size      = 100;
    scrap.accept_count  = 0;
    scrap.release_class = "scrap";
    auto rS = skill::batch_release::validate(*stock, scrap);
    EXPECT_TRUE(rS.passed);
    EXPECT_FALSE(hasFinding(rS, "DFM-BR-CLASS"));
    auto outS = skill::batch_release::apply(*stock, scrap);
    EXPECT_NEAR(outS.signature.pattern["yield"].get<double>(),
                0.0, 1e-9);
    EXPECT_EQ(outS.signature.pattern["release_class"].get<std::string>(),
              std::string("scrap"));
}
