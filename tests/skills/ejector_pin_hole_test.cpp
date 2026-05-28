// @lat: [[process/test-strategy#skill round-trip]]
//
// ejector_pin_hole — single straight cylindrical through-hole.
//
// Cases:
//   1. ApplyRemovesMaterial : a 5 mm pin removes ~ π r² × depth.
//   2. ValidateRejectsBadInput : dia outside [1.0, 25.0] mm flagged.
//   3. SignatureRecordsKind + dia/depth/through.
//   4. RecognizeMetadataReplay : recognize() replays parameters from history.
//   5. ShortDepthWarn (specific) : depth < 2 × dia → DFM-EJ-DEPTH warning.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/ejector_pin_hole.hpp"

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
TEST(SkillEjectorPinHole, ApplyRemovesMaterial)
{
    auto stock = skill::createCuboidStock(40.0, 30.0, 20.0);
    const double stockVol = volumeOf(stock->shape());

    skill::ejector_pin_hole::Input in;
    in.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1), 5.0, "largest" };
    in.position_x_mm = 20.0;
    in.position_y_mm = 15.0;
    in.dia_mm       = 5.0;
    in.depth_mm     = 20.0;

    auto out = skill::ejector_pin_hole::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    const double removed = stockVol - volumeOf(out.workpiece->shape());
    const double r = 2.5;
    const double approx = M_PI * r * r * 20.0;
    EXPECT_GT(removed, 0.0);
    EXPECT_NEAR(removed, approx, approx * 0.15);
}

// ─── 2. Validate rejects bad input (oversized dia) ────────────────────────
TEST(SkillEjectorPinHole, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(40.0, 30.0, 20.0);

    skill::ejector_pin_hole::Input in;
    in.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 10.0;
    in.position_y_mm = 10.0;
    in.dia_mm       = 30.0;      // > 25
    in.depth_mm     = 20.0;

    auto r = skill::ejector_pin_hole::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool sawCode = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-EJ-DIA") { sawCode = true; break; }
    EXPECT_TRUE(sawCode);

    EXPECT_THROW(skill::ejector_pin_hole::apply(*stock, in), skill::SkillError);
}

// ─── 3. Signature records kind + key params ───────────────────────────────
TEST(SkillEjectorPinHole, SignatureRecordsKind)
{
    auto stock = skill::createCuboidStock(40.0, 30.0, 20.0);

    skill::ejector_pin_hole::Input in;
    in.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 20.0;
    in.position_y_mm = 15.0;
    in.dia_mm       = 4.0;
    in.depth_mm     = 20.0;

    auto out = skill::ejector_pin_hole::apply(*stock, in);
    EXPECT_EQ(out.signature.skill_id, std::string("ejector_pin_hole"));
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("ejector_pin_hole"));
    EXPECT_NEAR(out.signature.pattern["dia_mm"].get<double>(),   4.0, 1e-9);
    EXPECT_NEAR(out.signature.pattern["depth_mm"].get<double>(), 20.0, 1e-9);
    EXPECT_TRUE(out.signature.pattern["through"].get<bool>());
}

// ─── 4. Recognize metadata replay ─────────────────────────────────────────
TEST(SkillEjectorPinHole, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(40.0, 30.0, 20.0);

    skill::ejector_pin_hole::Input in;
    in.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 18.0;
    in.position_y_mm = 12.0;
    in.dia_mm       = 6.0;
    in.depth_mm     = 20.0;

    auto out = skill::ejector_pin_hole::apply(*stock, in);
    auto cands = skill::ejector_pin_hole::recognize(*out.workpiece);
    ASSERT_GE(cands.size(), 1u);

    const auto& c = cands.front();
    EXPECT_GT(c.confidence, 0.7);
    EXPECT_NEAR(c.recovered_params["dia_mm"].get<double>(),        6.0, 0.05);
    EXPECT_NEAR(c.recovered_params["position_x_mm"].get<double>(), 18.0, 0.05);
    EXPECT_NEAR(c.recovered_params["depth_mm"].get<double>(),      20.0, 0.05);
}

// ─── 5. Short depth produces warning (specific) ───────────────────────────
TEST(SkillEjectorPinHole, ShortDepthWarn)
{
    auto stock = skill::createCuboidStock(40.0, 30.0, 20.0);

    skill::ejector_pin_hole::Input in;
    in.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 20.0;
    in.position_y_mm = 15.0;
    in.dia_mm       = 5.0;
    in.depth_mm     = 5.0;     // 5 < 2 × 5 → warning

    auto r = skill::ejector_pin_hole::validate(*stock, in);
    EXPECT_TRUE(r.passed);     // warning, not error
    bool sawWarn = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-EJ-DEPTH" && f.severity == "warning") {
            sawWarn = true; break;
        }
    EXPECT_TRUE(sawWarn);
}
