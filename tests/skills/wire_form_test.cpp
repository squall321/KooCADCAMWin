// @lat: [[process/test-strategy#skill round-trip]]
//
// wire_form — bend wire along a 3D polyline (metadata-only).

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/wire_form.hpp"

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

// ── 1. ApplyHandlesInput — paperclip-like 5-point polyline accepted ────────
TEST(SkillWireForm, ApplyHandlesInput)
{
    auto stock = skill::createCylindricalStock(1.0, 100.0);  // Ø1 × 100 mm wire
    const double v0 = volumeOf(stock->shape());

    skill::wire_form::Input in;
    in.wire_dia_mm = 1.0;
    // Open "U" shape — comfortably above 2 × wire_dia min bend.
    in.waypoints = {
        {  0.0,  0.0, 0.0 },
        { 20.0,  0.0, 0.0 },
        { 20.0, 10.0, 0.0 },
        {  0.0, 10.0, 0.0 },
        {  0.0, 20.0, 0.0 },
    };

    auto out = skill::wire_form::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    EXPECT_NEAR(volumeOf(out.workpiece->shape()), v0, 1e-9);
    EXPECT_EQ(out.signature.skill_id, std::string("wire_form"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
}

// ── 2. ValidateRejectsBadInput — < 2 waypoints & zero wire_dia ─────────────
TEST(SkillWireForm, ValidateRejectsBadInput)
{
    auto stock = skill::createCylindricalStock(1.0, 100.0);

    // Case A: 1 waypoint.
    {
        skill::wire_form::Input in;
        in.wire_dia_mm = 1.0;
        in.waypoints   = { { 0.0, 0.0, 0.0 } };
        auto r = skill::wire_form::validate(*stock, in);
        EXPECT_FALSE(r.passed);
        EXPECT_THROW(skill::wire_form::apply(*stock, in), skill::SkillError);
    }

    // Case B: wire_dia_mm = 0.
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

// ── 3. SignatureRecordsKind + polyline waypoints + key params ──────────────
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
    // Polyline length = 30 + 20 = 50.
    EXPECT_NEAR(out.signature.pattern["polyline_length_mm"].get<double>(),
                50.0, 1e-6);
    EXPECT_EQ(out.signature.pattern["waypoints"].size(), 3u);
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
}

// ── 4. RecognizeMetadataReplay — exact polyline recovery ───────────────────
TEST(SkillWireForm, RecognizeMetadataReplay)
{
    auto stock = skill::createCylindricalStock(1.0, 100.0);

    skill::wire_form::Input in;
    in.wire_dia_mm = 1.2;
    in.waypoints = {
        {  0.0,  0.0,  0.0 },
        { 15.0,  0.0,  0.0 },
        { 15.0, 10.0,  5.0 },         // 3-D bend, comfortably > 2.4 mm radius
        {  0.0, 10.0,  5.0 },
    };

    auto out = skill::wire_form::apply(*stock, in);
    auto cands = skill::wire_form::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["waypoints"].size(), 4u);
    EXPECT_NEAR(cands[0].recovered_params["wire_dia_mm"].get<double>(),
                1.2, 1e-9);
    EXPECT_NEAR(
        cands[0].recovered_params["waypoints"][2]["z_mm"].get<double>(),
        5.0, 1e-9);

    // Workpiece stripped of metadata — recognize is empty.
    skill::Workpiece raw(out.workpiece->shape());
    EXPECT_TRUE(skill::wire_form::recognize(raw).empty());
}

// ── 5. SPECIFIC: too-tight bend radius rejected (DFM-WF-BEND-R) ────────────
TEST(SkillWireForm, TooTightBendRejected)
{
    auto stock = skill::createCylindricalStock(1.0, 100.0);

    skill::wire_form::Input in;
    in.wire_dia_mm = 2.0;             // → min radius = 4 mm
    // Very short segments at a 90° corner → implied radius ≈ 1 mm < 4 mm.
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
