// @lat: [[process/test-strategy#skill round-trip]]

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/wire_edm.hpp"

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

// ── 1. Apply: triangular hole through the part ───────────────────────────
TEST(SkillWireEdm, ApplyCutsTriangularThroughHole)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::wire_edm::Input in;
    in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.contour_waypoints = {
        { 20.0, 15.0 },
        { 35.0, 15.0 },
        { 27.5, 32.0 },
    };

    auto out = skill::wire_edm::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    // Triangle area = 1/2 × 15 × 17 = 127.5 mm² → through 10 mm thickness.
    const double expected = 127.5 * 10.0;
    const double removed = volumeOf(stock->shape()) - volumeOf(out.workpiece->shape());
    EXPECT_NEAR(removed, expected, expected * 0.05);

    EXPECT_EQ(out.signature.skill_id, std::string("wire_edm"));
    EXPECT_TRUE(out.signature.pattern["is_through_cut"].get<bool>());
    EXPECT_EQ(out.signature.pattern["polygon_edge_count"].get<size_t>(), 3u);
}

// ── 2. Apply: pentagonal through-hole ────────────────────────────────────
TEST(SkillWireEdm, ApplyPentagonalThroughHole)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 8.0);

    skill::wire_edm::Input in;
    in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    const double cx = 30.0, cy = 30.0, R = 12.0;
    for (int i = 0; i < 5; ++i) {
        const double a = i * 2.0 * M_PI / 5.0 + M_PI / 2.0;
        in.contour_waypoints.push_back({ cx + R * std::cos(a),
                                         cy + R * std::sin(a) });
    }

    auto out = skill::wire_edm::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    // Regular pentagon area = (5/2) R² sin(2π/5).
    const double polyArea = 2.5 * R * R * std::sin(2.0 * M_PI / 5.0);
    const double expected = polyArea * 8.0;
    const double removed = volumeOf(stock->shape()) - volumeOf(out.workpiece->shape());
    EXPECT_NEAR(removed, expected, expected * 0.05);

    EXPECT_EQ(out.signature.pattern["polygon_edge_count"].get<size_t>(), 5u);
}

// ── 3. DFM-INPUT: < 3 waypoints rejected ─────────────────────────────────
TEST(SkillWireEdm, ValidateRejectsTooFewPoints)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);
    skill::wire_edm::Input in;
    in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.contour_waypoints = { { 10.0, 10.0 }, { 40.0, 10.0 } };  // only 2

    auto r = skill::wire_edm::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_THROW(skill::wire_edm::apply(*stock, in), skill::SkillError);
}

// ── 4. DFM-WIRE-KERF info always emitted ─────────────────────────────────
TEST(SkillWireEdm, ValidateEmitsKerfInfo)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);
    skill::wire_edm::Input in;
    in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.contour_waypoints = { { 10.0, 10.0 }, { 40.0, 10.0 }, { 25.0, 40.0 } };

    auto r = skill::wire_edm::validate(*stock, in);
    EXPECT_TRUE(r.passed);  // info findings do not fail validation
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-WIRE-KERF" && f.severity == "info") {
            found = true; break;
        }
    EXPECT_TRUE(found);
}

// ── 5. Recognize finds the polygonal through-walls ───────────────────────
// Wire-EDM recognize now clusters vertical walls into closed loops via
// shared vertical edges, classifies outer (stock outline) vs inner (cut)
// loops by XY bbox match, and reconstructs the polygon waypoints from the
// loop's vertical-edge top-Z vertices.
TEST(SkillWireEdm, RecognizeFindsPolygonalCut)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);
    skill::wire_edm::Input in;
    in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.contour_waypoints = {
        { 15.0, 15.0 },
        { 35.0, 15.0 },
        { 35.0, 35.0 },
        { 15.0, 35.0 },
    };

    auto out = skill::wire_edm::apply(*stock, in);
    auto cands = skill::wire_edm::recognize(*out.workpiece);
    ASSERT_FALSE(cands.empty());
    EXPECT_GE(cands[0].confidence, 0.5);
    EXPECT_LE(cands[0].confidence, 0.85);
    EXPECT_EQ(cands[0].matched_geometry["polygon_size"].get<size_t>(), 4u);
}
