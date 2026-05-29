// @lat: [[process/test-strategy#skill round-trip]]
//
// shipping_document — BOL/CMR/AWB document set generator metadata skill.
// 5-case sweep:
//   1. apply clones geometry unchanged
//   2. validate rejects bad input
//   3. signature records kind + doc_type + recipient + weight_kg
//   4. recognize metadata replay
//   5. specific: mode + carrier_label derived from doc_type, AWB heavy info

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/shipping_document.hpp"

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

skill::shipping_document::Input goodInput()
{
    skill::shipping_document::Input in;
    in.doc_type  = "BOL";
    in.recipient = "Acme Distribution Co.";
    in.weight_kg = 2500.0;
    return in;
}

}  // namespace

// ─── 1. Apply clones geometry unchanged ──────────────────────────────────
TEST(SkillShippingDocument, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    auto out = skill::shipping_document::apply(*stock, goodInput());
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. Validate rejects bad input ───────────────────────────────────────
TEST(SkillShippingDocument, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    // Unknown doc_type → error.
    auto in1 = goodInput();
    in1.doc_type = "carrier_pigeon_note";
    auto r1 = skill::shipping_document::validate(*stock, in1);
    EXPECT_FALSE(r1.passed);
    EXPECT_TRUE(hasFinding(r1, "DFM-SD-DOCTYPE"));
    EXPECT_THROW(skill::shipping_document::apply(*stock, in1),
                 skill::SkillError);

    // Empty recipient → error.
    auto in2 = goodInput();
    in2.recipient = "";
    auto r2 = skill::shipping_document::validate(*stock, in2);
    EXPECT_FALSE(r2.passed);
    EXPECT_TRUE(hasFinding(r2, "DFM-INPUT"));

    // Non-positive weight → error.
    auto in3 = goodInput();
    in3.weight_kg = -10.0;
    auto r3 = skill::shipping_document::validate(*stock, in3);
    EXPECT_FALSE(r3.passed);
    EXPECT_TRUE(hasFinding(r3, "DFM-INPUT"));
}

// ─── 3. Signature records kind + key params ──────────────────────────────
TEST(SkillShippingDocument, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::shipping_document::Input in;
    in.doc_type  = "CMR";
    in.recipient = "Globex GmbH";
    in.weight_kg = 4500.0;

    auto out = skill::shipping_document::apply(*stock, in);

    EXPECT_EQ(out.signature.skill_id,
              std::string("shipping_document"));
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("shipping_document"));
    EXPECT_EQ(out.signature.pattern["doc_type"].get<std::string>(),
              std::string("CMR"));
    EXPECT_EQ(out.signature.pattern["recipient"].get<std::string>(),
              std::string("Globex GmbH"));
    EXPECT_NEAR(out.signature.pattern["weight_kg"].get<double>(),
                4500.0, 1e-9);
    EXPECT_FALSE(out.signature.pattern["geometric_change"].get<bool>());
}

// ─── 4. Recognize metadata replay ────────────────────────────────────────
TEST(SkillShippingDocument, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);
    auto out = skill::shipping_document::apply(*stock, goodInput());

    auto cands = skill::shipping_document::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].skill_id, std::string("shipping_document"));
    EXPECT_EQ(cands[0].recovered_params["doc_type"].get<std::string>(),
              std::string("BOL"));
    EXPECT_EQ(cands[0].recovered_params["recipient"].get<std::string>(),
              std::string("Acme Distribution Co."));

    // Stripped workpiece → recognize empty.
    skill::Workpiece raw(out.workpiece->shape(), out.workpiece->material());
    EXPECT_TRUE(skill::shipping_document::recognize(raw).empty());
}

// ─── 5. Specific: mode + carrier_label derived from doc_type ────────────
TEST(SkillShippingDocument, ModeAndCarrierLabelDerived)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    // AWB → air mode.
    auto awb = goodInput();
    awb.doc_type  = "AWB";
    awb.weight_kg = 1500.0;
    auto outAwb = skill::shipping_document::apply(*stock, awb);
    EXPECT_EQ(outAwb.signature.pattern["mode"].get<std::string>(),
              std::string("air"));
    EXPECT_EQ(outAwb.signature.pattern["carrier_label"].get<std::string>(),
              std::string("air_carrier"));

    // SWB → sea mode.
    auto swb = goodInput();
    swb.doc_type = "SWB";
    auto outSwb = skill::shipping_document::apply(*stock, swb);
    EXPECT_EQ(outSwb.signature.pattern["mode"].get<std::string>(),
              std::string("sea"));
    EXPECT_EQ(outSwb.signature.pattern["carrier_label"].get<std::string>(),
              std::string("sea_carrier"));

    // AWB with weight_kg > 7000 → DFM-SD-WEIGHT-AIR info finding.
    auto heavyAir = goodInput();
    heavyAir.doc_type  = "AWB";
    heavyAir.weight_kg = 9500.0;
    auto rH = skill::shipping_document::validate(*stock, heavyAir);
    EXPECT_TRUE(rH.passed);            // info-only, not error
    EXPECT_TRUE(hasFinding(rH, "DFM-SD-WEIGHT-AIR"));
    // Also triggers generic heavy-lift threshold? No (< 30000), but
    // 9500 < 30000 so DFM-SD-WEIGHT-HIGH is not raised.
    EXPECT_FALSE(hasFinding(rH, "DFM-SD-WEIGHT-HIGH"));
}
