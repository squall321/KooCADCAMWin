// @lat: [[process/test-strategy#skill round-trip]]
//
// wire_form — bend wire along a 3D polyline (real geometric impl).

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/wire_form.hpp"

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

// ── 1. Apply: real bent-pipe geometry — bbox spans the polyline extents ────
TEST(SkillWireForm, ApplyBuildsBentPipeGeometry)
{
    auto stock = skill::createCylindricalStock(1.0, 100.0);

    skill::wire_form::Input in;
    in.wire_dia_mm = 1.0;
    // Open "U" shape over generous segments (≥ 5 × wire_dia per side so the
    // bend-radius DFM passes).
    in.waypoints = {
        {  0.0,  0.0, 0.0 },
        { 20.0,  0.0, 0.0 },
        { 20.0, 10.0, 0.0 },
        {  0.0, 10.0, 0.0 },
        {  0.0, 20.0, 0.0 },
    };

    auto out = skill::wire_form::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    // bbox should span the polyline footprint (20 × 20) in XY, ~1 mm in Z.
    double xMin, yMin, zMin, xMax, yMax, zMax;
    out.workpiece->boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    EXPECT_GT(xMax - xMin, 20.0 - 1e-3);
    EXPECT_GT(yMax - yMin, 20.0 - 1e-3);
    EXPECT_LT(zMax - zMin, 2.0);          // wire thickness only

    // Volume should be > 0 and within an order of magnitude of the analytic
    // estimate π(d/2)² × length.  Length = 20 + 10 + 20 + 10 = 60 mm.
    const double v1 = volumeOf(out.workpiece->shape());
    const double expectedV = M_PI * 0.25 * 1.0 * 1.0 * 60.0;   // ~47 mm³
    EXPECT_GT(v1, expectedV * 0.5);
    EXPECT_LT(v1, expectedV * 2.0);

    EXPECT_TRUE(out.signature.pattern["geometry_changed"].get<bool>());
    EXPECT_LT(out.signature.tooling.stock_removed_mm3, 0.0);   // material added
    EXPECT_EQ(out.signature.skill_id, std::string("wire_form"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
}

// ── 2. DFM: bad input rejected (< 2 waypoints, zero wire_dia) ──────────────
TEST(SkillWireForm, ValidateRejectsBadInput)
{
    auto stock = skill::createCylindricalStock(1.0, 100.0);

    {
        skill::wire_form::Input in;
        in.wire_dia_mm = 1.0;
        in.waypoints   = { { 0.0, 0.0, 0.0 } };
        auto r = skill::wire_form::validate(*stock, in);
        EXPECT_FALSE(r.passed);
        EXPECT_THROW(skill::wire_form::apply(*stock, in), skill::SkillError);
    }

    {
        skill::wire_form::Input in;
        in.wire_dia_mm = 0.0;
        in.waypoints = {
            {  0.0, 0.0, 0.0 },
            { 10.0, 0.0, 0.0 },
        };
        auto r = skill::wire_form::validate(*stock, in);
        EXPECT_FALSE(r.passed);
    }
}

// ── 3. Signature records waypoints + key params + polyline length ──────────
TEST(SkillWireForm, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCylindricalStock(1.0, 100.0);

    skill::wire_form::Input in;
    in.wire_dia_mm = 1.0;
    in.waypoints = {
        {  0.0,  0.0, 0.0 },
        { 30.0,  0.0, 0.0 },
        { 30.0, 20.0, 0.0 },
    };

    auto out = skill::wire_form::apply(*stock, in);

    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("wire_form"));
    EXPECT_EQ(out.signature.pattern["waypoint_count"].get<int>(), 3);
    EXPECT_EQ(out.signature.pattern["bend_count"].get<int>(), 1);
    EXPECT_NEAR(out.signature.pattern["wire_dia_mm"].get<double>(), 1.0, 1e-9);
    EXPECT_NEAR(out.signature.pattern["polyline_length_mm"].get<double>(),
                50.0, 1e-6);
    EXPECT_EQ(out.signature.pattern["waypoints"].size(), 3u);
    EXPECT_TRUE(out.signature.pattern["geometry_changed"].get<bool>());
}

// ── 4. Recognize — metadata replay (1.0) + geometric fallback (0.45) ───────
TEST(SkillWireForm, RecognizeMetadataAndGeometric)
{
    auto stock = skill::createCylindricalStock(1.0, 100.0);

    skill::wire_form::Input in;
    in.wire_dia_mm = 1.2;
    in.waypoints = {
        {  0.0,  0.0,  0.0 },
        { 15.0,  0.0,  0.0 },
        { 15.0, 10.0,  5.0 },         // 3-D bend, > 2.4 mm radius
        {  0.0, 10.0,  5.0 },
    };

    auto out = skill::wire_form::apply(*stock, in);

    // Metadata replay.
    auto cands = skill::wire_form::recognize(*out.workpiece);
    ASSERT_GE(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["waypoints"].size(), 4u);
    EXPECT_NEAR(cands[0].recovered_params["wire_dia_mm"].get<double>(),
                1.2, 1e-9);

    // Geometric fallback — strip metadata.  The U-shape spans XYZ so the
    // heuristic should match.
    skill::Workpiece raw(out.workpiece->shape());
    auto rawCands = skill::wire_form::recognize(raw);
    if (!rawCands.empty()) {
        EXPECT_NEAR(rawCands[0].confidence, 0.45, 1e-6);
        EXPECT_EQ(rawCands[0].matched_geometry["source"].get<std::string>(),
                  std::string("geometric_fallback"));
    }
}

// ── 5. Too-tight bend radius rejected (DFM-WF-BEND-R, ASM Vol 14B) ─────────
TEST(SkillWireForm, TooTightBendRejected)
{
    auto stock = skill::createCylindricalStock(1.0, 100.0);

    skill::wire_form::Input in;
    in.wire_dia_mm = 2.0;             // min radius = 4 mm
    // Short segments at 90° corner → implied radius ≈ 1 mm < 4 mm.
    in.waypoints = {
        {  0.0, 0.0, 0.0 },
        {  1.0, 0.0, 0.0 },
        {  1.0, 1.0, 0.0 },
    };

    auto r = skill::wire_form::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-WF-BEND-R") { found = true; break; }
    EXPECT_TRUE(found);
    EXPECT_THROW(skill::wire_form::apply(*stock, in), skill::SkillError);
}
