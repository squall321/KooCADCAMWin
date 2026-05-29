// @lat: [[process/test-strategy#skill round-trip]]
//
// instrument_assemble skill — 5-case sweep covering identity-geometry
// synthesis, DFM rejection of bad input, metadata-only recognition, and a
// skill-specific assertion that cycle time scales with component count.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/instrument_assemble.hpp"

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

skill::instrument_assemble::Input goodInput()
{
    skill::instrument_assemble::Input in;
    in.instrument_type = "guitar";
    in.component_count = 6;
    return in;
}

}  // namespace

// ─── 1. Apply: geometry CLONED unchanged ──────────────────────────────────
TEST(SkillInstrumentAssemble, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    auto out = skill::instrument_assemble::apply(*stock, goodInput());
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("instrument_assemble"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. Validate REJECTS bad input ───────────────────────────────────────
TEST(SkillInstrumentAssemble, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto in = goodInput();
    in.component_count = 1;     // < 2 required
    auto r = skill::instrument_assemble::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-ASM-COUNT"));
    EXPECT_THROW(skill::instrument_assemble::apply(*stock, in),
                 skill::SkillError);

    auto in2 = goodInput();
    in2.instrument_type = "";    // empty
    auto r2 = skill::instrument_assemble::validate(*stock, in2);
    EXPECT_FALSE(r2.passed);
    EXPECT_TRUE(hasFinding(r2, "DFM-INPUT"));
}

// ─── 3. Signature records kind + key params ──────────────────────────────
TEST(SkillInstrumentAssemble, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto in = goodInput();
    in.instrument_type = "violin";
    in.component_count = 12;
    auto out = skill::instrument_assemble::apply(*stock, in);

    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("instrument_assemble"));
    EXPECT_EQ(out.signature.pattern["instrument_type"].get<std::string>(),
              std::string("violin"));
    EXPECT_EQ(out.signature.pattern["component_count"].get<int>(), 12);
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
}

// ─── 4. Recognize via metadata replay ─────────────────────────────────────
TEST(SkillInstrumentAssemble, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    auto out = skill::instrument_assemble::apply(*stock, goodInput());

    auto cands = skill::instrument_assemble::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["instrument_type"].get<std::string>(),
              std::string("guitar"));
    EXPECT_EQ(cands[0].recovered_params["component_count"].get<int>(), 6);

    // Strip metadata → recognize empty.
    skill::Workpiece raw(out.workpiece->shape());
    auto cands2 = skill::instrument_assemble::recognize(raw);
    EXPECT_TRUE(cands2.empty())
        << "instrument_assemble cannot be recognized geometrically";
}

// ─── 5. SPECIFIC: cycle time scales linearly with component_count ────────
TEST(SkillInstrumentAssemble, CycleTimeScalesWithComponentCount)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto inA = goodInput();
    inA.component_count = 6;
    auto outA = skill::instrument_assemble::apply(*stock, inA);

    auto inB = goodInput();
    inB.component_count = 24;
    auto outB = skill::instrument_assemble::apply(*stock, inB);

    // 24 components should take 4× as long as 6 components.
    EXPECT_NEAR(outB.signature.tooling.est_cycle_time_s,
                4.0 * outA.signature.tooling.est_cycle_time_s, 1e-6);
    EXPECT_GT(outB.signature.tooling.est_cycle_time_s,
              outA.signature.tooling.est_cycle_time_s);
}
