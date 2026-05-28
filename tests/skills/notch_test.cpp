// @lat: [[process/test-strategy#skill round-trip]]
//
// notch skill — rectangular or triangular cut at sheet edge.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/notch.hpp"

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

// ── 1. Apply: rectangular notch removes expected material ───────────────
TEST(SkillNotch, ApplyRemovesOrAddsMaterial)
{
    auto stock = skill::createCuboidStock(100.0, 80.0, 1.5);

    skill::notch::Input in;
    in.edge_selector = skill::flange::EdgeSide::XMax;
    in.shape         = skill::notch::Shape::Rectangle;
    in.position_mm   = 40.0;     // mid-edge along y
    in.width_mm      = 8.0;      // ≥ 1.5 × 1.5 = 2.25
    in.depth_mm      = 5.0;      // ≥ 1.5

    auto out = skill::notch::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    // Removed volume ≈ width × depth × thickness = 8 × 5 × 1.5 = 60
    const double expected = 8.0 * 5.0 * 1.5;
    const double removed  = volumeOf(stock->shape()) - volumeOf(out.workpiece->shape());
    EXPECT_NEAR(removed, expected, expected * 0.10);

    EXPECT_EQ(out.signature.skill_id, std::string("notch"));
    EXPECT_EQ(out.signature.tooling.tool_type, std::string("punch_press"));
    EXPECT_EQ(out.signature.pattern["shape"].get<std::string>(), "rectangle");
}

// ── 2. ValidateRejectsBadInput ──────────────────────────────────────────
TEST(SkillNotch, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(100.0, 80.0, 1.5);

    skill::notch::Input bad;
    bad.edge_selector = skill::flange::EdgeSide::XMax;
    bad.shape         = skill::notch::Shape::Rectangle;
    bad.position_mm   = 40.0;
    bad.width_mm      = 1.0;     // < 1.5 × 1.5 = 2.25 → DFM-N-MIN-W
    bad.depth_mm      = 0.5;     // < thickness → DFM-N-MIN-D

    auto r = skill::notch::validate(*stock, bad);
    EXPECT_FALSE(r.passed);
    bool foundW = false, foundD = false;
    for (const auto& f : r.findings) {
        if (f.code == "DFM-N-MIN-W") foundW = true;
        if (f.code == "DFM-N-MIN-D") foundD = true;
    }
    EXPECT_TRUE(foundW);
    EXPECT_TRUE(foundD);
    EXPECT_THROW(skill::notch::apply(*stock, bad), skill::SkillError);
}

// ── 3. SignatureRecordsKind ─────────────────────────────────────────────
TEST(SkillNotch, SignatureRecordsKind)
{
    auto stock = skill::createCuboidStock(100.0, 80.0, 1.5);

    skill::notch::Input in;
    in.edge_selector = skill::flange::EdgeSide::YMax;
    in.shape         = skill::notch::Shape::Triangle;
    in.position_mm   = 50.0;
    in.width_mm      = 6.0;
    in.depth_mm      = 4.0;

    auto out = skill::notch::apply(*stock, in);
    EXPECT_EQ(out.signature.skill_id, std::string("notch"));
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(), "notch");
    EXPECT_EQ(out.signature.pattern["shape"].get<std::string>(), "triangle");
    EXPECT_EQ(out.signature.pattern["edge_selector"].get<std::string>(), "y_max");
    EXPECT_TRUE(out.signature.pattern["is_through_cut"].get<bool>());
}

// ── 4. RecognizeMetadataReplay ──────────────────────────────────────────
TEST(SkillNotch, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(100.0, 80.0, 1.5);

    skill::notch::Input in;
    in.edge_selector = skill::flange::EdgeSide::XMax;
    in.shape         = skill::notch::Shape::Rectangle;
    in.position_mm   = 40.0;
    in.width_mm      = 8.0;
    in.depth_mm      = 5.0;

    auto out = skill::notch::apply(*stock, in);
    auto cands = skill::notch::recognize(*out.workpiece);

    ASSERT_FALSE(cands.empty());
    bool foundSkill = false;
    for (const auto& c : cands) {
        if (c.skill_id == "notch") { foundSkill = true; break; }
    }
    EXPECT_TRUE(foundSkill);
}

// ── 5. Specific assertion: triangular notch removes ~ half the volume ──
TEST(SkillNotch, TriangleRemovesHalfVolumeOfRectangle)
{
    auto stock = skill::createCuboidStock(100.0, 80.0, 1.5);

    skill::notch::Input rect;
    rect.edge_selector = skill::flange::EdgeSide::XMin;
    rect.shape         = skill::notch::Shape::Rectangle;
    rect.position_mm   = 40.0;
    rect.width_mm      = 6.0;
    rect.depth_mm      = 4.0;

    auto outR = skill::notch::apply(*stock, rect);
    const double remR = volumeOf(stock->shape()) - volumeOf(outR.workpiece->shape());

    auto stock2 = skill::createCuboidStock(100.0, 80.0, 1.5);
    skill::notch::Input tri = rect;
    tri.shape = skill::notch::Shape::Triangle;
    auto outT = skill::notch::apply(*stock2, tri);
    const double remT = volumeOf(stock2->shape()) - volumeOf(outT.workpiece->shape());

    // Triangle ~ half the area of rectangle ⇒ removed volume ~ half.
    EXPECT_NEAR(remT, remR * 0.5, remR * 0.15);
}
