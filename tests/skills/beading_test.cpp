// @lat: [[process/test-strategy#skill round-trip]]
//
// beading skill — long thin ridge or groove along a polyline.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/beading.hpp"

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

// ── 1. Apply: ridge mode ADDS material; groove mode REMOVES ─────────────
TEST(SkillBeading, ApplyRemovesOrAddsMaterial)
{
    auto stock = skill::createCuboidStock(100.0, 80.0, 1.5);

    skill::beading::Input ridge;
    ridge.waypoints = {{20.0, 40.0}, {80.0, 40.0}};
    ridge.width_mm  = 5.0;       // ≥ 2 × 1.5 = 3.0
    ridge.depth_mm  = 1.5;       // ≥ 0.75 and ≤ 3.0
    ridge.mode      = skill::beading::Mode::Ridge;

    auto outR = skill::beading::apply(*stock, ridge);
    const double dR = volumeOf(outR.workpiece->shape()) - volumeOf(stock->shape());
    EXPECT_GT(dR, 0.0);

    auto stock2 = skill::createCuboidStock(100.0, 80.0, 1.5);
    skill::beading::Input groove = ridge;
    groove.mode = skill::beading::Mode::Groove;
    auto outG = skill::beading::apply(*stock2, groove);
    const double dG = volumeOf(outG.workpiece->shape()) - volumeOf(stock2->shape());
    EXPECT_LT(dG, 0.0);

    EXPECT_EQ(outR.signature.skill_id, std::string("beading"));
    EXPECT_EQ(outG.signature.skill_id, std::string("beading"));
}

// ── 2. ValidateRejectsBadInput ──────────────────────────────────────────
TEST(SkillBeading, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(100.0, 80.0, 1.5);

    // Width too small
    skill::beading::Input badW;
    badW.waypoints = {{20.0, 40.0}, {80.0, 40.0}};
    badW.width_mm  = 1.0;        // < 2 × 1.5 = 3.0 → DFM-BD-MIN-W
    badW.depth_mm  = 1.0;
    badW.mode      = skill::beading::Mode::Ridge;

    auto r = skill::beading::validate(*stock, badW);
    EXPECT_FALSE(r.passed);
    bool foundW = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-BD-MIN-W") { foundW = true; break; }
    EXPECT_TRUE(foundW);
    EXPECT_THROW(skill::beading::apply(*stock, badW), skill::SkillError);
}

// ── 3. SignatureRecordsKind ─────────────────────────────────────────────
TEST(SkillBeading, SignatureRecordsKind)
{
    auto stock = skill::createCuboidStock(100.0, 80.0, 1.5);

    skill::beading::Input in;
    in.waypoints = {{30.0, 30.0}, {30.0, 50.0}, {60.0, 50.0}};
    in.width_mm  = 4.0;
    in.depth_mm  = 1.0;
    in.mode      = skill::beading::Mode::Ridge;

    auto out = skill::beading::apply(*stock, in);
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(), "beading");
    EXPECT_EQ(out.signature.pattern["mode"].get<std::string>(), "ridge");
    EXPECT_TRUE(out.signature.pattern["is_additive"].get<bool>());
    EXPECT_GT(out.signature.pattern["path_length_mm"].get<double>(), 40.0);
}

// ── 4. RecognizeMetadataReplay ──────────────────────────────────────────
TEST(SkillBeading, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(100.0, 80.0, 1.5);

    skill::beading::Input in;
    in.waypoints = {{20.0, 40.0}, {80.0, 40.0}};
    in.width_mm  = 4.0;
    in.depth_mm  = 1.0;
    in.mode      = skill::beading::Mode::Ridge;

    auto out = skill::beading::apply(*stock, in);
    auto cands = skill::beading::recognize(*out.workpiece);

    // The ridge top face should be detectable (above sheet top by depth).
    bool foundBeading = false;
    for (const auto& c : cands) {
        if (c.skill_id == "beading") { foundBeading = true; break; }
    }
    EXPECT_TRUE(foundBeading);
}

// ── 5. Specific assertion: path_length matches polyline arc length ─────
TEST(SkillBeading, PathLengthMatchesPolyline)
{
    auto stock = skill::createCuboidStock(100.0, 80.0, 1.5);

    skill::beading::Input in;
    in.waypoints = {{10.0, 10.0}, {10.0, 50.0}, {70.0, 50.0}};
    in.width_mm  = 4.0;
    in.depth_mm  = 1.0;
    in.mode      = skill::beading::Mode::Ridge;

    auto out = skill::beading::apply(*stock, in);
    // Expected: 40 + 60 = 100 mm
    EXPECT_NEAR(out.signature.pattern["path_length_mm"].get<double>(),
                100.0, 0.5);
}
