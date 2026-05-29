// @lat: [[process/test-strategy#skill round-trip]]
//
// cycle_optimize skill — metadata-only sweep.  Covers:
//   1. apply clones geometry unchanged
//   2. validate rejects bad input (zero original_cycle_s)
//   3. signature records kind + key params + savings_pct
//   4. recognize via metadata replay
//   5. skill-specific: optimized > original is DFM-CYCLE-REGRESSION error

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/cycle_optimize.hpp"

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

// ─── 1. Apply: geometry COMPLETELY unchanged ──────────────────────────────
TEST(SkillCycleOptimize, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(80.0, 50.0, 15.0, "aluminum_6061");
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    skill::cycle_optimize::Input in;
    auto out = skill::cycle_optimize::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. Validate rejects bad input ────────────────────────────────────────
TEST(SkillCycleOptimize, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(80.0, 50.0, 15.0, "aluminum_6061");

    skill::cycle_optimize::Input in;
    in.original_cycle_s = 0.0;        // invalid

    auto r = skill::cycle_optimize::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_THROW(skill::cycle_optimize::apply(*stock, in), skill::SkillError);
}

// ─── 3. Signature records kind + key params + savings_pct ───────────────
TEST(SkillCycleOptimize, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCuboidStock(80.0, 50.0, 15.0, "aluminum_6061");

    skill::cycle_optimize::Input in;
    in.original_cycle_s  = 40.0;
    in.optimized_cycle_s = 30.0;          // 25% savings
    in.method            = "parallel_grip";

    auto out = skill::cycle_optimize::apply(*stock, in);
    EXPECT_EQ(out.signature.skill_id, std::string("cycle_optimize"));
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("cycle_optimize"));
    EXPECT_NEAR(out.signature.pattern["original_cycle_s"].get<double>(),
                40.0, 1e-9);
    EXPECT_NEAR(out.signature.pattern["optimized_cycle_s"].get<double>(),
                30.0, 1e-9);
    EXPECT_EQ(out.signature.pattern["method"].get<std::string>(),
              std::string("parallel_grip"));
    EXPECT_NEAR(out.signature.pattern["savings_pct"].get<double>(),
                25.0, 1e-6);
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
}

// ─── 4. Recognize via metadata replay ─────────────────────────────────────
TEST(SkillCycleOptimize, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(80.0, 50.0, 15.0, "aluminum_6061");

    skill::cycle_optimize::Input in;
    in.original_cycle_s  = 20.0;
    in.optimized_cycle_s = 18.0;
    in.method            = "skip_air_move";
    auto out = skill::cycle_optimize::apply(*stock, in);

    auto cands = skill::cycle_optimize::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["method"].get<std::string>(),
              std::string("skip_air_move"));

    skill::Workpiece raw(out.workpiece->shape());
    auto cands2 = skill::cycle_optimize::recognize(raw);
    EXPECT_TRUE(cands2.empty());
}

// ─── 5. Skill-specific: optimized > original is DFM-CYCLE-REGRESSION ─────
TEST(SkillCycleOptimize, RegressedCycleIsError)
{
    auto stock = skill::createCuboidStock(80.0, 50.0, 15.0, "aluminum_6061");

    skill::cycle_optimize::Input in;
    in.original_cycle_s  = 20.0;
    in.optimized_cycle_s = 25.0;        // SLOWER than original
    in.method            = "broken_blend";

    auto r = skill::cycle_optimize::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-CYCLE-REGRESSION"));
    EXPECT_THROW(skill::cycle_optimize::apply(*stock, in), skill::SkillError);
}
