// @lat: [[process/test-strategy#skill round-trip]]
//
// draw_bead skill — closed-loop drawbead groove for material restraint.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/draw_bead.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

#include <cmath>

using namespace koocadcam;

namespace {
double volumeOf(const TopoDS_Shape& s) {
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}
}  // namespace

// ── 1. Apply: closed-loop drawbead removes material ─────────────────────
TEST(SkillDrawBead, ApplyRemovesOrAddsMaterial)
{
    auto stock = skill::createCuboidStock(100.0, 80.0, 1.5);

    skill::draw_bead::Input in;
    // Closed rectangular loop in XY (4 corners + repeat first).
    in.waypoints = {
        {20.0, 20.0},
        {80.0, 20.0},
        {80.0, 60.0},
        {20.0, 60.0},
        {20.0, 20.0}
    };
    in.width_mm = 5.0;     // ≥ 3 × 1.5 = 4.5
    in.depth_mm = 1.0;     // ≥ 0.75 and ≤ 2.25

    auto out = skill::draw_bead::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    // Removed volume ≈ path_length × width × depth.
    const double pathLen = 2.0 * (60.0 + 40.0);   // perimeter = 200
    const double expected = pathLen * 5.0 * 1.0;
    const double removed  = volumeOf(stock->shape()) - volumeOf(out.workpiece->shape());
    // 0.40 tolerance because stadium-chain double-counts width at endpoints
    // and the closed loop overlaps the segment-cylinder unions.
    EXPECT_NEAR(removed, expected, expected * 0.40);

    EXPECT_EQ(out.signature.skill_id, std::string("draw_bead"));
    EXPECT_TRUE(out.signature.pattern["is_closed"].get<bool>());
}

// ── 2. ValidateRejectsBadInput ──────────────────────────────────────────
TEST(SkillDrawBead, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(100.0, 80.0, 1.5);

    // Too few waypoints
    skill::draw_bead::Input few;
    few.waypoints = {{20.0, 20.0}, {80.0, 20.0}};  // < 3 → DFM-INPUT
    few.width_mm  = 5.0;
    few.depth_mm  = 1.0;
    auto r1 = skill::draw_bead::validate(*stock, few);
    EXPECT_FALSE(r1.passed);
    EXPECT_THROW(skill::draw_bead::apply(*stock, few), skill::SkillError);

    // Width too narrow
    skill::draw_bead::Input narrow;
    narrow.waypoints = {
        {20.0, 20.0}, {80.0, 20.0}, {80.0, 60.0}, {20.0, 20.0}
    };
    narrow.width_mm = 2.0;      // < 3 × 1.5 = 4.5 → DFM-DB-MIN-W
    narrow.depth_mm = 1.0;
    auto r2 = skill::draw_bead::validate(*stock, narrow);
    EXPECT_FALSE(r2.passed);
    bool foundW = false;
    for (const auto& f : r2.findings)
        if (f.code == "DFM-DB-MIN-W") { foundW = true; break; }
    EXPECT_TRUE(foundW);
}

// ── 3. SignatureRecordsKind ─────────────────────────────────────────────
TEST(SkillDrawBead, SignatureRecordsKind)
{
    auto stock = skill::createCuboidStock(100.0, 80.0, 1.5);

    skill::draw_bead::Input in;
    in.waypoints = {
        {20.0, 20.0}, {80.0, 20.0}, {80.0, 60.0}, {20.0, 60.0}, {20.0, 20.0}
    };
    in.width_mm = 5.0;
    in.depth_mm = 1.0;

    auto out = skill::draw_bead::apply(*stock, in);
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(), "draw_bead");
    EXPECT_TRUE(out.signature.pattern["is_closed"].get<bool>());
    EXPECT_NEAR(out.signature.pattern["width_mm"].get<double>(), 5.0, 1e-6);
    EXPECT_NEAR(out.signature.pattern["depth_mm"].get<double>(), 1.0, 1e-6);
    EXPECT_EQ(out.signature.tooling.tool_type, std::string("drawbead_die"));
}

// ── 4. RecognizeMetadataReplay ──────────────────────────────────────────
TEST(SkillDrawBead, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(100.0, 80.0, 1.5);

    skill::draw_bead::Input in;
    in.waypoints = {
        {20.0, 20.0}, {80.0, 20.0}, {80.0, 60.0}, {20.0, 60.0}, {20.0, 20.0}
    };
    in.width_mm = 5.0;
    in.depth_mm = 1.0;

    auto out = skill::draw_bead::apply(*stock, in);
    auto cands = skill::draw_bead::recognize(*out.workpiece);

    bool foundSkill = false;
    for (const auto& c : cands) {
        if (c.skill_id == "draw_bead") { foundSkill = true; break; }
    }
    EXPECT_TRUE(foundSkill);
}

// ── 5. Specific assertion: auto-closes an open loop input ──────────────
TEST(SkillDrawBead, AutoClosesOpenLoop)
{
    auto stock = skill::createCuboidStock(100.0, 80.0, 1.5);

    skill::draw_bead::Input openLoop;
    // First != last (no auto-close given).
    openLoop.waypoints = {
        {20.0, 20.0}, {80.0, 20.0}, {80.0, 60.0}, {20.0, 60.0}
    };
    openLoop.width_mm = 5.0;
    openLoop.depth_mm = 1.0;

    auto r = skill::draw_bead::validate(*stock, openLoop);
    // Open loop should produce an info-level DFM-DB-CLOSED but still pass.
    EXPECT_TRUE(r.passed);
    bool foundClosed = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-DB-CLOSED" && f.severity == "info") {
            foundClosed = true; break;
        }
    EXPECT_TRUE(foundClosed);

    // And apply() should succeed by auto-closing.
    auto out = skill::draw_bead::apply(*stock, openLoop);
    EXPECT_TRUE(out.signature.pattern["is_closed"].get<bool>());
}
