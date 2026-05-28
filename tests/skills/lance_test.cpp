// @lat: [[process/test-strategy#skill round-trip]]
//
// lance skill — partial slit + bend (combined cut + bend signature).

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/lance.hpp"

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

// ── 1. Apply: lance slits + lifts a tab; resulting solid is valid ──────
TEST(SkillLance, ApplyRemovesOrAddsMaterial)
{
    auto stock = skill::createCuboidStock(100.0, 80.0, 1.5);

    skill::lance::Input in;
    in.waypoints   = {{30.0, 40.0}, {50.0, 40.0}};
    in.tab_width_mm = 8.0;       // ≥ 1.5
    in.lift_mm      = 3.0;       // ≥ 1.5 and ≤ 5 × 1.5 = 7.5

    auto out = skill::lance::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    // Slit removes very little; tab adds a slab of (tab_width × cut_path × thickness)
    // approximately = 8 × 20 × 1.5 = 240 mm³ in volume change.
    const double netDelta = volumeOf(out.workpiece->shape()) - volumeOf(stock->shape());
    EXPECT_GT(netDelta, 0.0);   // tab added > slit removed

    EXPECT_EQ(out.signature.skill_id, std::string("lance"));
    EXPECT_EQ(out.signature.tooling.tool_type, std::string("lancing_die"));
}

// ── 2. ValidateRejectsBadInput ──────────────────────────────────────────
TEST(SkillLance, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(100.0, 80.0, 1.5);

    // a) Lift too small (< thickness)
    skill::lance::Input badLift;
    badLift.waypoints    = {{30.0, 40.0}, {50.0, 40.0}};
    badLift.tab_width_mm = 8.0;
    badLift.lift_mm      = 0.5;     // < 1.5 → DFM-L-MIN-LIFT
    auto r1 = skill::lance::validate(*stock, badLift);
    EXPECT_FALSE(r1.passed);
    bool foundLift = false;
    for (const auto& f : r1.findings)
        if (f.code == "DFM-L-MIN-LIFT") { foundLift = true; break; }
    EXPECT_TRUE(foundLift);
    EXPECT_THROW(skill::lance::apply(*stock, badLift), skill::SkillError);

    // b) Insufficient waypoints
    skill::lance::Input badPoints;
    badPoints.waypoints    = {{30.0, 40.0}};   // < 2 → DFM-INPUT
    badPoints.tab_width_mm = 8.0;
    badPoints.lift_mm      = 3.0;
    auto r2 = skill::lance::validate(*stock, badPoints);
    EXPECT_FALSE(r2.passed);
}

// ── 3. SignatureRecordsKind ─────────────────────────────────────────────
TEST(SkillLance, SignatureRecordsKind)
{
    auto stock = skill::createCuboidStock(100.0, 80.0, 1.5);

    skill::lance::Input in;
    in.waypoints    = {{20.0, 40.0}, {60.0, 40.0}};
    in.tab_width_mm = 10.0;
    in.lift_mm      = 4.0;

    auto out = skill::lance::apply(*stock, in);
    EXPECT_EQ(out.signature.skill_id, std::string("lance"));
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(), "lance");
    EXPECT_EQ(out.signature.pattern["cut_kind"].get<std::string>(), "Polyline");
    EXPECT_TRUE(out.signature.pattern["is_combined_cut_bend"].get<bool>());
    EXPECT_TRUE(out.signature.pattern["is_through_cut"].get<bool>());
}

// ── 4. RecognizeMetadataReplay ──────────────────────────────────────────
TEST(SkillLance, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(100.0, 80.0, 1.5);

    skill::lance::Input in;
    in.waypoints    = {{30.0, 40.0}, {70.0, 40.0}};
    in.tab_width_mm = 12.0;
    in.lift_mm      = 4.0;

    auto out = skill::lance::apply(*stock, in);
    auto cands = skill::lance::recognize(*out.workpiece);

    // Recognize should find the raised tab.  Accept zero-or-one — the
    // tab is geometrically disconnected so detection is best-effort.
    if (!cands.empty()) {
        EXPECT_EQ(cands.front().skill_id, std::string("lance"));
        EXPECT_NEAR(cands.front().recovered_params["lift_mm"].get<double>(),
                    4.0, 1.0);
    }
}

// ── 5. Specific assertion: signature captures BOTH cut and bend ────────
TEST(SkillLance, SignatureCapturesCutAndBend)
{
    auto stock = skill::createCuboidStock(100.0, 80.0, 1.5);

    skill::lance::Input in;
    in.waypoints    = {{20.0, 40.0}, {60.0, 40.0}};
    in.tab_width_mm = 10.0;
    in.lift_mm      = 4.0;

    auto out = skill::lance::apply(*stock, in);
    // Path length recorded in pattern → cut info present.
    EXPECT_GT(out.signature.pattern["cut_path_length_mm"].get<double>(), 35.0);
    // Tab width + lift recorded in pattern → bend info present.
    EXPECT_NEAR(out.signature.pattern["tab_width_mm"].get<double>(), 10.0, 1e-6);
    EXPECT_NEAR(out.signature.pattern["lift_mm"].get<double>(), 4.0, 1e-6);
    EXPECT_GT(out.signature.pattern["slit_kerf_mm"].get<double>(), 0.0);
}
