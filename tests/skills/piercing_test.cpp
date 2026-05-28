// @lat: [[process/test-strategy#skill round-trip]]
//
// piercing skill — through hole (circle or rect) with explicit clearance.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/piercing.hpp"

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

// ── 1. Apply: circular piercing removes π r² × t ──────────────────────
TEST(SkillPiercing, ApplyChangesGeometryOrPreserves)
{
    auto stock = skill::createCuboidStock(100.0, 80.0, 1.5);

    skill::piercing::Input in;
    in.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.shape        = skill::piercing::Shape::Circle;
    in.position_x_mm = 30.0;
    in.position_y_mm = 40.0;
    in.diameter_mm  = 4.0;    // 4.0 > 1.5 (thickness) ⇒ pass DFM-PRC-MIN-DIM
    in.clearance_mm = 0.1;    // 6.7% of 1.5 — within 3-15% range

    auto out = skill::piercing::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    const double expected = M_PI * 2.0 * 2.0 * 1.5;
    const double removed  = volumeOf(stock->shape()) - volumeOf(out.workpiece->shape());
    EXPECT_NEAR(removed, expected, expected * 0.05);
}

// ── 2. Validate rejects bad input: too small hole + bad clearance ────
TEST(SkillPiercing, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(100.0, 80.0, 1.5);

    // Hole smaller than thickness.
    skill::piercing::Input in;
    in.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.shape        = skill::piercing::Shape::Circle;
    in.position_x_mm = 30.0;
    in.position_y_mm = 40.0;
    in.diameter_mm  = 1.0;      // < 1.5 mm thickness

    auto r = skill::piercing::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-PRC-MIN-DIM") { found = true; break; }
    EXPECT_TRUE(found);
    EXPECT_THROW(skill::piercing::apply(*stock, in), skill::SkillError);

    // Bad clearance.
    skill::piercing::Input in2 = in;
    in2.diameter_mm  = 4.0;
    in2.clearance_mm = 0.5;     // 33% of 1.5 — outside 3-15%
    auto r2 = skill::piercing::validate(*stock, in2);
    EXPECT_FALSE(r2.passed);
    bool clearanceFound = false;
    for (const auto& f : r2.findings)
        if (f.code == "DFM-PRC-CLEARANCE") { clearanceFound = true; break; }
    EXPECT_TRUE(clearanceFound);
}

// ── 3. Signature records kind, clearance, and is_through_cut ─────────
TEST(SkillPiercing, SignatureRecordsKind)
{
    auto stock = skill::createCuboidStock(100.0, 80.0, 1.5);

    skill::piercing::Input in;
    in.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.shape        = skill::piercing::Shape::Circle;
    in.position_x_mm = 50.0;
    in.position_y_mm = 40.0;
    in.diameter_mm  = 4.0;
    in.clearance_mm = 0.12;     // 8% of 1.5

    auto out = skill::piercing::apply(*stock, in);
    EXPECT_EQ(out.signature.skill_id, std::string("piercing"));
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(), std::string("piercing"));
    EXPECT_TRUE(out.signature.pattern["is_through_cut"].get<bool>());
    EXPECT_NEAR(out.signature.pattern["punch_die_clearance_mm"].get<double>(),
                0.12, 1e-6);
    EXPECT_EQ(out.signature.tooling.tool_type, std::string("pierce_punch_die"));
}

// ── 4. Recognize: geometric for circles, metadata replay for rectangles
TEST(SkillPiercing, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(100.0, 80.0, 1.5);

    skill::piercing::Input in;
    in.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.shape        = skill::piercing::Shape::Circle;
    in.position_x_mm = 60.0;
    in.position_y_mm = 30.0;
    in.diameter_mm  = 4.0;
    in.clearance_mm = 0.1;

    auto out   = skill::piercing::apply(*stock, in);
    auto cands = skill::piercing::recognize(*out.workpiece);

    ASSERT_FALSE(cands.empty());
    const auto& c = cands.front();
    EXPECT_EQ(c.skill_id, std::string("piercing"));
    EXPECT_NEAR(c.recovered_params["position_x_mm"].get<double>(), 60.0, 1e-3);
    EXPECT_NEAR(c.recovered_params["dims"]["diameter_mm"].get<double>(), 4.0, 0.05);
}

// ── 5. Specific: default clearance = 8% of thickness ─────────────────
TEST(SkillPiercing, DefaultClearanceIsEightPct)
{
    auto stock = skill::createCuboidStock(100.0, 80.0, 1.5);

    skill::piercing::Input in;
    in.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.shape        = skill::piercing::Shape::Circle;
    in.position_x_mm = 50.0;
    in.position_y_mm = 40.0;
    in.diameter_mm  = 4.0;
    in.clearance_mm = 0.0;       // request default

    auto out = skill::piercing::apply(*stock, in);
    const double expected = 0.08 * 1.5;
    EXPECT_NEAR(out.signature.pattern["punch_die_clearance_mm"].get<double>(),
                expected, 1e-6);
}
