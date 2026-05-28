// @lat: [[process/test-strategy#skill round-trip]]
//
// coining skill — shallow stamp impression.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/coining.hpp"

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

// ── 1. Apply: circular coining removes a shallow disc of material ───────
TEST(SkillCoining, ApplyRemovesOrAddsMaterial)
{
    auto stock = skill::createCuboidStock(100.0, 80.0, 1.5);

    skill::coining::Input in;
    in.position_x_mm = 50.0;
    in.position_y_mm = 40.0;
    in.shape         = skill::coining::Shape::Circle;
    in.diameter_mm   = 6.0;
    in.depth_mm      = 0.3;          // 20% of 1.5 → OK

    auto out = skill::coining::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    const double expected = M_PI * 3.0 * 3.0 * 0.3;
    const double removed  = volumeOf(stock->shape()) - volumeOf(out.workpiece->shape());
    EXPECT_NEAR(removed, expected, expected * 0.10);

    EXPECT_EQ(out.signature.skill_id, std::string("coining"));
    EXPECT_EQ(out.signature.tooling.tool_type, std::string("coining_die"));
}

// ── 2. ValidateRejectsBadInput ──────────────────────────────────────────
TEST(SkillCoining, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(100.0, 80.0, 1.5);

    // Depth > 30% of thickness
    skill::coining::Input deep;
    deep.position_x_mm = 50.0;
    deep.position_y_mm = 40.0;
    deep.shape         = skill::coining::Shape::Circle;
    deep.diameter_mm   = 6.0;
    deep.depth_mm      = 0.8;        // > 0.3 × 1.5 = 0.45 → DFM-C-MAX-DEPTH

    auto r = skill::coining::validate(*stock, deep);
    EXPECT_FALSE(r.passed);
    bool foundMax = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-C-MAX-DEPTH") { foundMax = true; break; }
    EXPECT_TRUE(foundMax);
    EXPECT_THROW(skill::coining::apply(*stock, deep), skill::SkillError);

    // Depth too small (< 5% of thickness)
    skill::coining::Input shallow = deep;
    shallow.depth_mm = 0.01;         // < 0.05 × 1.5 = 0.075 → DFM-C-MIN-DEPTH
    auto r2 = skill::coining::validate(*stock, shallow);
    EXPECT_FALSE(r2.passed);
    bool foundMin = false;
    for (const auto& f : r2.findings)
        if (f.code == "DFM-C-MIN-DEPTH") { foundMin = true; break; }
    EXPECT_TRUE(foundMin);
}

// ── 3. SignatureRecordsKind ─────────────────────────────────────────────
TEST(SkillCoining, SignatureRecordsKind)
{
    auto stock = skill::createCuboidStock(100.0, 80.0, 1.5);

    skill::coining::Input in;
    in.position_x_mm = 60.0;
    in.position_y_mm = 30.0;
    in.shape         = skill::coining::Shape::Rectangle;
    in.length_mm     = 12.0;
    in.width_mm      = 6.0;
    in.depth_mm      = 0.3;
    in.corner_r_mm   = 1.0;

    auto out = skill::coining::apply(*stock, in);
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(), "coining");
    EXPECT_EQ(out.signature.pattern["shape"].get<std::string>(), "rectangle");
    EXPECT_TRUE(out.signature.pattern["shallow_pocket_present"].get<bool>());
    EXPECT_LE(out.signature.pattern["depth_ratio"].get<double>(), 0.3);
}

// ── 4. RecognizeMetadataReplay ──────────────────────────────────────────
TEST(SkillCoining, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(100.0, 80.0, 1.5);

    skill::coining::Input in;
    in.position_x_mm = 60.0;
    in.position_y_mm = 30.0;
    in.shape         = skill::coining::Shape::Circle;
    in.diameter_mm   = 8.0;
    in.depth_mm      = 0.3;

    auto out = skill::coining::apply(*stock, in);
    auto cands = skill::coining::recognize(*out.workpiece);

    ASSERT_FALSE(cands.empty());
    const auto& c = cands.front();
    EXPECT_EQ(c.skill_id, std::string("coining"));
    EXPECT_NEAR(c.recovered_params["position_x_mm"].get<double>(), 60.0, 0.5);
    EXPECT_NEAR(c.recovered_params["position_y_mm"].get<double>(), 30.0, 0.5);
    EXPECT_NEAR(c.recovered_params["depth_mm"].get<double>(), 0.3, 0.05);
}

// ── 5. Specific assertion: depth is strictly shallow ───────────────────
TEST(SkillCoining, DepthIsShallow)
{
    auto stock = skill::createCuboidStock(100.0, 80.0, 1.5);

    skill::coining::Input in;
    in.position_x_mm = 50.0;
    in.position_y_mm = 40.0;
    in.shape         = skill::coining::Shape::Circle;
    in.diameter_mm   = 6.0;
    in.depth_mm      = 0.4;       // 26.6% of 1.5 → OK (≤ 30%)

    auto out = skill::coining::apply(*stock, in);
    // The depth_ratio = depth_mm / sheet_thickness should be ≤ 0.3.
    EXPECT_LE(out.signature.pattern["depth_ratio"].get<double>(), 0.3);
    // Distinguishes coining from emboss: emboss can be up to 3× thickness.
    EXPECT_LT(out.signature.pattern["depth_mm"].get<double>(),
              out.signature.pattern["sheet_thickness_mm"].get<double>());
}
