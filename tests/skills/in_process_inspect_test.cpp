// @lat: [[process/test-strategy#skill round-trip]]
//
// in_process_inspect — sampling-plan mid-process check.  Covers:
//   1. apply clones geometry unchanged
//   2. validate rejects bad input
//   3. signature records kind + lot_number + sample_size + defect_count
//   4. recognize metadata replay
//   5. specific: lot_accepted accept/reject boundary

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/in_process_inspect.hpp"

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
TEST(SkillInProcessInspect, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    skill::in_process_inspect::Input in;
    in.lot_number   = "LOT-001";
    in.sample_size  = 20;
    in.accept_size  = 2;
    in.defect_count = 1;

    auto out = skill::in_process_inspect::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. Validate rejects bad input ───────────────────────────────────────
TEST(SkillInProcessInspect, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    // Empty lot_number → error.
    skill::in_process_inspect::Input in;
    in.lot_number   = "";       // INVALID
    in.sample_size  = 10;
    in.accept_size  = 1;
    in.defect_count = 0;
    auto r = skill::in_process_inspect::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-IPI-LOT"));
    EXPECT_THROW(skill::in_process_inspect::apply(*stock, in),
                 skill::SkillError);

    // sample_size <= 0 → error.
    skill::in_process_inspect::Input in2;
    in2.lot_number   = "LOT-X";
    in2.sample_size  = 0;
    in2.accept_size  = 0;
    in2.defect_count = 0;
    auto r2 = skill::in_process_inspect::validate(*stock, in2);
    EXPECT_FALSE(r2.passed);
    EXPECT_TRUE(hasFinding(r2, "DFM-INPUT"));

    // defect_count > sample_size → error.
    skill::in_process_inspect::Input in3;
    in3.lot_number   = "LOT-Y";
    in3.sample_size  = 5;
    in3.accept_size  = 1;
    in3.defect_count = 10;
    auto r3 = skill::in_process_inspect::validate(*stock, in3);
    EXPECT_FALSE(r3.passed);
}

// ─── 3. Signature records kind + key params ──────────────────────────────
TEST(SkillInProcessInspect, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::in_process_inspect::Input in;
    in.lot_number   = "LOT-2026-05-28";
    in.sample_size  = 32;
    in.accept_size  = 3;
    in.defect_count = 1;

    auto out = skill::in_process_inspect::apply(*stock, in);
    EXPECT_EQ(out.signature.skill_id, std::string("in_process_inspect"));
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("in_process_inspect"));
    EXPECT_EQ(out.signature.pattern["lot_number"].get<std::string>(),
              std::string("LOT-2026-05-28"));
    EXPECT_EQ(out.signature.pattern["sample_size"].get<int>(),  32);
    EXPECT_EQ(out.signature.pattern["accept_size"].get<int>(),  3);
    EXPECT_EQ(out.signature.pattern["defect_count"].get<int>(), 1);
    EXPECT_FALSE(out.signature.pattern["geometric_change"].get<bool>());
}

// ─── 4. Recognize metadata replay ────────────────────────────────────────
TEST(SkillInProcessInspect, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::in_process_inspect::Input in;
    in.lot_number   = "LOT-AAA";
    in.sample_size  = 20;
    in.accept_size  = 2;
    in.defect_count = 0;
    auto out = skill::in_process_inspect::apply(*stock, in);

    auto cands = skill::in_process_inspect::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].skill_id, std::string("in_process_inspect"));
    EXPECT_EQ(cands[0].recovered_params["lot_number"].get<std::string>(),
              std::string("LOT-AAA"));
    EXPECT_EQ(cands[0].recovered_params["sample_size"].get<int>(), 20);

    // Workpiece without metadata → recognize empty.
    skill::Workpiece raw(out.workpiece->shape(), out.workpiece->material());
    EXPECT_TRUE(skill::in_process_inspect::recognize(raw).empty());
}

// ─── 5. Specific: lot_accepted accept/reject boundary + DFM-IPI-REJECT ───
TEST(SkillInProcessInspect, LotAcceptRejectBoundaryAndDefectRate)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    // Boundary accept: defect_count == accept_size.
    skill::in_process_inspect::Input accept;
    accept.lot_number   = "LOT-A";
    accept.sample_size  = 50;
    accept.accept_size  = 2;
    accept.defect_count = 2;
    auto rAcc = skill::in_process_inspect::validate(*stock, accept);
    EXPECT_TRUE(rAcc.passed);
    EXPECT_FALSE(hasFinding(rAcc, "DFM-IPI-REJECT"));
    auto outAcc = skill::in_process_inspect::apply(*stock, accept);
    EXPECT_TRUE(outAcc.signature.pattern["lot_accepted"].get<bool>());
    EXPECT_NEAR(outAcc.signature.pattern["defect_rate"].get<double>(),
                2.0 / 50.0, 1e-9);

    // Reject: defect_count > accept_size.
    skill::in_process_inspect::Input reject;
    reject.lot_number   = "LOT-R";
    reject.sample_size  = 50;
    reject.accept_size  = 2;
    reject.defect_count = 7;
    auto rRej = skill::in_process_inspect::validate(*stock, reject);
    EXPECT_TRUE(rRej.passed);  // info-only, still passes
    EXPECT_TRUE(hasFinding(rRej, "DFM-IPI-REJECT"));
    auto outRej = skill::in_process_inspect::apply(*stock, reject);
    EXPECT_FALSE(outRej.signature.pattern["lot_accepted"].get<bool>());
}
