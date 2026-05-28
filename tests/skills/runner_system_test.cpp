// @lat: [[process/test-strategy#skill round-trip]]
//
// runner_system — array of cylindrical channels cut through a mold plate.
//
// Cases:
//   1. ApplyRemovesMaterial : two horizontal runners remove ~ π r² × ΣL.
//   2. ValidateRejectsBadInput : dia outside [2.0, 8.0] mm flagged.
//   3. SignatureRecordsKind   : signature.skill_id + dia + segment_count.
//   4. RecognizeMetadataReplay : recognize() replays segments from history.
//   5. EmptySegmentsRejected (specific) : empty segments → DFM-RUNNER-N error.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/runner_system.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using namespace koocadcam;

namespace {

double volumeOf(const TopoDS_Shape& s) {
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}

}  // namespace

// ─── 1. Apply removes material ────────────────────────────────────────────
TEST(SkillRunnerSystem, ApplyRemovesMaterial)
{
    auto stock = skill::createCuboidStock(60.0, 40.0, 20.0);
    const double stockVol = volumeOf(stock->shape());

    skill::runner_system::Input in;
    in.dia_mm = 4.0;
    in.segments.push_back({ 10.0, 10.0, 10.0, 50.0, 10.0, 10.0 });   // L = 40
    in.segments.push_back({ 10.0, 30.0, 10.0, 50.0, 30.0, 10.0 });   // L = 40

    auto out = skill::runner_system::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    const double removed = stockVol - volumeOf(out.workpiece->shape());
    const double r = 2.0;
    const double approx = M_PI * r * r * 80.0;     // 2 × 40 mm
    EXPECT_GT(removed, 0.0);
    EXPECT_NEAR(removed, approx, approx * 0.20);
}

// ─── 2. Validate rejects bad input (dia out of range) ─────────────────────
TEST(SkillRunnerSystem, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(60.0, 40.0, 20.0);

    skill::runner_system::Input in;
    in.dia_mm = 12.0;       // > 8.0 → DFM-RUNNER-DIA
    in.segments.push_back({ 5.0, 10.0, 10.0, 55.0, 10.0, 10.0 });

    auto r = skill::runner_system::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool sawCode = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-RUNNER-DIA") { sawCode = true; break; }
    EXPECT_TRUE(sawCode);

    EXPECT_THROW(skill::runner_system::apply(*stock, in), skill::SkillError);
}

// ─── 3. Signature records kind + key params ───────────────────────────────
TEST(SkillRunnerSystem, SignatureRecordsKind)
{
    auto stock = skill::createCuboidStock(60.0, 40.0, 20.0);

    skill::runner_system::Input in;
    in.dia_mm = 5.0;
    in.segments.push_back({ 10.0, 10.0, 10.0, 50.0, 10.0, 10.0 });
    in.segments.push_back({ 50.0, 10.0, 10.0, 50.0, 30.0, 10.0 });

    auto out = skill::runner_system::apply(*stock, in);
    EXPECT_EQ(out.signature.skill_id, std::string("runner_system"));
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("runner_system"));
    EXPECT_NEAR(out.signature.pattern["dia_mm"].get<double>(), 5.0, 1e-9);
    EXPECT_EQ(out.signature.pattern["segment_count"].get<int>(), 2);
}

// ─── 4. Recognize metadata replay ─────────────────────────────────────────
TEST(SkillRunnerSystem, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(60.0, 40.0, 20.0);

    skill::runner_system::Input in;
    in.dia_mm = 4.5;
    in.segments.push_back({ 8.0, 12.0, 10.0, 52.0, 12.0, 10.0 });
    in.segments.push_back({ 8.0, 28.0, 10.0, 52.0, 28.0, 10.0 });

    auto out = skill::runner_system::apply(*stock, in);
    auto cands = skill::runner_system::recognize(*out.workpiece);
    ASSERT_GE(cands.size(), 1u);

    const auto& c = cands.front();
    EXPECT_GT(c.confidence, 0.7);
    EXPECT_NEAR(c.recovered_params["dia_mm"].get<double>(), 4.5, 1e-9);
    ASSERT_EQ(c.recovered_params["segments"].size(), 2u);
    EXPECT_NEAR(c.recovered_params["segments"][0]["start_x_mm"].get<double>(),
                8.0, 1e-9);
}

// ─── 5. Empty segments rejected (specific) ────────────────────────────────
TEST(SkillRunnerSystem, EmptySegmentsRejected)
{
    auto stock = skill::createCuboidStock(60.0, 40.0, 20.0);

    skill::runner_system::Input in;
    in.dia_mm = 4.0;
    // segments left empty

    auto r = skill::runner_system::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool sawCode = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-RUNNER-N") { sawCode = true; break; }
    EXPECT_TRUE(sawCode);
}
