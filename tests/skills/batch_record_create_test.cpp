// @lat: [[process/test-strategy#skill round-trip]]
//
// batch_record_create skill — 5-case sweep covering metadata-only
// synthesis, DFM rejection of empty fields, signature recording,
// recognize replay, and a specific assertion (zero params → info).

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/batch_record_create.hpp"

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

skill::batch_record_create::Input goodInput()
{
    skill::batch_record_create::Input in;
    in.lot_id                  = "L-2026-04891";
    in.operator_id             = "op-247";
    in.timestamp               = "2026-05-29T08:14:23Z";
    in.parameters_logged_count = 42;
    return in;
}

}  // namespace

// ─── 1. Apply: geometry CLONED unchanged ──────────────────────────────────
TEST(SkillBatchRecordCreate, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    auto out = skill::batch_record_create::apply(*stock, goodInput());
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("batch_record_create"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. Validate REJECTS bad input (empty lot_id, op_id, timestamp) ───────
TEST(SkillBatchRecordCreate, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto in = goodInput();
    in.lot_id = "";
    auto r = skill::batch_record_create::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-INPUT"));
    EXPECT_THROW(skill::batch_record_create::apply(*stock, in),
                 skill::SkillError);

    auto in2 = goodInput();
    in2.operator_id = "";
    auto r2 = skill::batch_record_create::validate(*stock, in2);
    EXPECT_FALSE(r2.passed);

    auto in3 = goodInput();
    in3.timestamp = "";
    auto r3 = skill::batch_record_create::validate(*stock, in3);
    EXPECT_FALSE(r3.passed);

    auto in4 = goodInput();
    in4.parameters_logged_count = -1;
    auto r4 = skill::batch_record_create::validate(*stock, in4);
    EXPECT_FALSE(r4.passed);
}

// ─── 3. Signature records kind + lot/op/timestamp/count ──────────────────
TEST(SkillBatchRecordCreate, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto in = goodInput();
    in.lot_id                  = "L-2026-99999";
    in.operator_id             = "op-007";
    in.timestamp               = "2026-12-31T23:59:59Z";
    in.parameters_logged_count = 128;

    auto out = skill::batch_record_create::apply(*stock, in);

    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("batch_record_create"));
    EXPECT_EQ(out.signature.pattern["lot_id"].get<std::string>(),
              std::string("L-2026-99999"));
    EXPECT_EQ(out.signature.pattern["operator_id"].get<std::string>(),
              std::string("op-007"));
    EXPECT_EQ(out.signature.pattern["timestamp"].get<std::string>(),
              std::string("2026-12-31T23:59:59Z"));
    EXPECT_EQ(out.signature.pattern["parameters_logged_count"].get<int>(), 128);
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
    EXPECT_EQ(out.signature.tooling.tool_type,
              std::string("ebatch_recorder"));
}

// ─── 4. Recognize via metadata replay ─────────────────────────────────────
TEST(SkillBatchRecordCreate, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    auto out = skill::batch_record_create::apply(*stock, goodInput());

    auto cands = skill::batch_record_create::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["lot_id"].get<std::string>(),
              std::string("L-2026-04891"));
    EXPECT_EQ(cands[0].recovered_params["parameters_logged_count"].get<int>(),
              42);

    // Strip metadata → recognize empty.
    skill::Workpiece raw(out.workpiece->shape());
    auto cands2 = skill::batch_record_create::recognize(raw);
    EXPECT_TRUE(cands2.empty())
        << "batch_record_create cannot be recognized geometrically";
}

// ─── 5. SPECIFIC: zero parameters_logged_count → DFM info (still passes) ─
TEST(SkillBatchRecordCreate, ZeroParametersLoggedIsInfoNotError)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto in = goodInput();
    in.parameters_logged_count = 0;

    auto r = skill::batch_record_create::validate(*stock, in);
    EXPECT_TRUE(r.passed)
        << "zero logged parameters is INFO-level, not blocking";
    EXPECT_TRUE(hasFinding(r, "DFM-EBR-EMPTY"));

    EXPECT_NO_THROW(skill::batch_record_create::apply(*stock, in));
}
