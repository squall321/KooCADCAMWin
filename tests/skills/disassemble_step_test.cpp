// @lat: [[process/test-strategy#skill round-trip]]
//
// disassemble_step skill — 5-case sweep covering:
//   1. identity geometry on apply
//   2. DFM rejects bad input (step_count <= 0)
//   3. signature carries skill kind + step_count + fasteners_removed + time_min
//   4. recognize via metadata replay (and empty when features stripped)
//   5. long-disassembly warning issued but does NOT block

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/disassemble_step.hpp"

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

// ─── 1. Apply: geometry is COMPLETELY unchanged ──────────────────────────
TEST(SkillDisassembleStep, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    skill::disassemble_step::Input in;
    in.step_count        = 5;
    in.fasteners_removed = 12;
    in.time_min          = 8.5;

    auto out = skill::disassemble_step::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("disassemble_step"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. Validate rejects bad input (non-positive step_count) ─────────────
TEST(SkillDisassembleStep, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::disassemble_step::Input bad;
    bad.step_count        = 0;        // invalid
    bad.fasteners_removed = 4;
    bad.time_min          = 5.0;

    auto r = skill::disassemble_step::validate(*stock, bad);
    EXPECT_FALSE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-INPUT"));
    EXPECT_THROW(skill::disassemble_step::apply(*stock, bad), skill::SkillError);

    skill::disassemble_step::Input negFast = bad;
    negFast.step_count        = 3;
    negFast.fasteners_removed = -1;
    auto r2 = skill::disassemble_step::validate(*stock, negFast);
    EXPECT_FALSE(r2.passed);

    skill::disassemble_step::Input zeroTime = bad;
    zeroTime.step_count = 3;
    zeroTime.time_min   = 0.0;
    auto r3 = skill::disassemble_step::validate(*stock, zeroTime);
    EXPECT_FALSE(r3.passed);
}

// ─── 3. Signature records skill kind + key params ────────────────────────
TEST(SkillDisassembleStep, SignatureRecordsKindAndParams)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::disassemble_step::Input in;
    in.step_count        = 7;
    in.fasteners_removed = 23;
    in.time_min          = 18.0;

    auto out = skill::disassemble_step::apply(*stock, in);

    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("disassemble_step"));
    EXPECT_EQ(out.signature.pattern["step_count"].get<int>(),         7);
    EXPECT_EQ(out.signature.pattern["fasteners_removed"].get<int>(),  23);
    EXPECT_NEAR(out.signature.pattern["time_min"].get<double>(),      18.0, 1e-9);
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
    EXPECT_EQ(out.signature.tooling.tool_type, std::string("hand_tools"));
}

// ─── 4. Recognize via metadata replay ────────────────────────────────────
TEST(SkillDisassembleStep, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::disassemble_step::Input in;
    in.step_count        = 4;
    in.fasteners_removed = 9;
    in.time_min          = 6.0;

    auto out = skill::disassemble_step::apply(*stock, in);

    auto cands = skill::disassemble_step::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["step_count"].get<int>(), 4);

    // Strip features → recognize returns empty.
    skill::Workpiece raw(out.workpiece->shape());
    auto empty = skill::disassemble_step::recognize(raw);
    EXPECT_TRUE(empty.empty())
        << "disassemble_step cannot be recognized geometrically";
}

// ─── 5. Specific: long disassembly time → warning, not block ─────────────
TEST(SkillDisassembleStep, LongTimeIssuesWarningButDoesNotBlock)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::disassemble_step::Input in;
    in.step_count        = 30;
    in.fasteners_removed = 80;
    in.time_min          = 180.0;     // 3 hours — past the 120 min threshold

    auto r = skill::disassemble_step::validate(*stock, in);
    EXPECT_TRUE(r.passed)  // warning, not error
        << "long-disassembly is a warning and should NOT block apply";
    EXPECT_TRUE(hasFinding(r, "DFM-DISASM-TIME"));
    EXPECT_NO_THROW(skill::disassemble_step::apply(*stock, in));
}
