// @lat: [[process/test-strategy#compound pin_and_diamond_locating_set]]
//
// Verifies REAL 3-2-1 fixture receptacle geometry: round hole + capsule
// slot (2 end caps fused with a bar) + 3 chamfer cones.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/pin_and_diamond_locating_set.hpp"

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

// ─── 1. apply: real volume delta (round + capsule + chamfers) ────────────
TEST(SkillPinAndDiamondLocatingSet, ApplyRealCompoundGeometry)
{
    auto stock = skill::createCuboidStock(80.0, 50.0, 15.0);
    const int    stockFaces = stock->faceCount();
    const double stockVol   = volumeOf(stock->shape());

    skill::pin_and_diamond_locating_set::Input in;
    in.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.axis_dir     = gp_Dir(0, 0, -1);
    in.round_x_mm   = 20.0; in.round_y_mm   = 25.0;
    in.diamond_x_mm = 60.0; in.diamond_y_mm = 25.0;
    in.pin_dia_mm   = 6.0;
    in.hole_depth_mm = 10.0;
    in.diamond_slot_extension_mm = 0.6;
    in.chamfer_mm   = 0.3;

    auto out = skill::pin_and_diamond_locating_set::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    // Volumes (depth 10):
    //   Round Ø6 cyl   = π·3²·10 = 282.7
    //   Capsule area   = π·3² + 2 · 0.6 · 6  = 28.27 + 7.2 = 35.47
    //   Capsule vol    = 35.47·10 = 354.7
    //   Chamfer bands tiny (we accept up to 25% tolerance)
    const double rPin = 3.0;
    const double roundVol = M_PI * rPin * rPin * 10.0;
    const double capsuleArea = M_PI * rPin * rPin + 2.0 * 0.6 * 6.0;
    const double diamondVol  = capsuleArea * 10.0;
    const double approx      = roundVol + diamondVol;
    const double removed     = stockVol - volumeOf(out.workpiece->shape());
    EXPECT_NEAR(removed, approx, approx * 0.20);
    EXPECT_GT(out.workpiece->faceCount(), stockFaces);
}

// ─── 2. validate rejects too-small pin or too-close connection ───────────
TEST(SkillPinAndDiamondLocatingSet, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(80.0, 50.0, 15.0);

    // (a) pin_dia 3 < 4 mm (Carr Lane)
    {
        skill::pin_and_diamond_locating_set::Input in;
        in.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
        in.round_x_mm   = 20.0; in.round_y_mm = 25.0;
        in.diamond_x_mm = 60.0; in.diamond_y_mm = 25.0;
        in.pin_dia_mm   = 3.0;        // < 4 mm
        in.hole_depth_mm = 10.0;
        in.diamond_slot_extension_mm = 0.6;
        in.chamfer_mm   = 0.3;
        auto r = skill::pin_and_diamond_locating_set::validate(*stock, in);
        EXPECT_FALSE(r.passed);
        bool found = false;
        for (const auto& f : r.findings)
            if (f.code == "DFM-LOCATE-DIA") { found = true; break; }
        EXPECT_TRUE(found);
        EXPECT_THROW(skill::pin_and_diamond_locating_set::apply(*stock, in),
                     skill::SkillError);
    }
    // (b) connection too short (L < 3·D)
    {
        skill::pin_and_diamond_locating_set::Input in;
        in.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
        in.round_x_mm   = 25.0; in.round_y_mm = 25.0;
        in.diamond_x_mm = 30.0; in.diamond_y_mm = 25.0;   // L = 5 < 3·6 = 18
        in.pin_dia_mm   = 6.0;
        in.hole_depth_mm = 10.0;
        in.diamond_slot_extension_mm = 0.6;
        in.chamfer_mm   = 0.3;
        auto r = skill::pin_and_diamond_locating_set::validate(*stock, in);
        EXPECT_FALSE(r.passed);
        bool found = false;
        for (const auto& f : r.findings)
            if (f.code == "DFM-LOCATE-SPACING") { found = true; break; }
        EXPECT_TRUE(found);
    }
    // (c) slot extension > pin Ø (too sloppy)
    {
        skill::pin_and_diamond_locating_set::Input in;
        in.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
        in.round_x_mm   = 20.0; in.round_y_mm = 25.0;
        in.diamond_x_mm = 60.0; in.diamond_y_mm = 25.0;
        in.pin_dia_mm   = 6.0;
        in.hole_depth_mm = 10.0;
        in.diamond_slot_extension_mm = 7.0;      // > 6
        in.chamfer_mm   = 0.3;
        auto r = skill::pin_and_diamond_locating_set::validate(*stock, in);
        EXPECT_FALSE(r.passed);
        bool found = false;
        for (const auto& f : r.findings)
            if (f.code == "DFM-LOCATE-SLOT") { found = true; break; }
        EXPECT_TRUE(found);
    }
}

// ─── 3. signature compound + params round-trip ───────────────────────────
TEST(SkillPinAndDiamondLocatingSet, SignatureCompoundRoundTrip)
{
    auto stock = skill::createCuboidStock(80.0, 50.0, 15.0);

    skill::pin_and_diamond_locating_set::Input in;
    in.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.axis_dir     = gp_Dir(0, 0, -1);
    in.round_x_mm   = 20.0; in.round_y_mm = 25.0;
    in.diamond_x_mm = 60.0; in.diamond_y_mm = 25.0;
    in.pin_dia_mm   = 6.0;
    in.hole_depth_mm = 10.0;
    in.diamond_slot_extension_mm = 0.6;
    in.chamfer_mm   = 0.3;

    auto out = skill::pin_and_diamond_locating_set::apply(*stock, in);
    EXPECT_TRUE(out.signature.pattern.at("is_compound").get<bool>());
    EXPECT_GE(out.signature.pattern.at("subfeature_count").get<int>(), 2);
    EXPECT_EQ(out.signature.pattern.at("subfeature_count").get<int>(), 4);
    EXPECT_NEAR(out.signature.params.at("pin_dia_mm").get<double>(), 6.0, 1e-9);
    EXPECT_NEAR(out.signature.params.at("diamond_slot_extension_mm").get<double>(),
                0.6, 1e-9);
}

// ─── 4. recognize: metadata replay or geometric, confidence > 0.5 ───────
TEST(SkillPinAndDiamondLocatingSet, RecognizeReturnsCandidate)
{
    auto stock = skill::createCuboidStock(80.0, 50.0, 15.0);

    skill::pin_and_diamond_locating_set::Input in;
    in.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.axis_dir     = gp_Dir(0, 0, -1);
    in.round_x_mm   = 20.0; in.round_y_mm = 25.0;
    in.diamond_x_mm = 60.0; in.diamond_y_mm = 25.0;
    in.pin_dia_mm   = 6.0;
    in.hole_depth_mm = 10.0;
    in.diamond_slot_extension_mm = 0.6;
    in.chamfer_mm   = 0.3;

    auto out = skill::pin_and_diamond_locating_set::apply(*stock, in);
    auto cands = skill::pin_and_diamond_locating_set::recognize(*out.workpiece);
    ASSERT_FALSE(cands.empty());
    EXPECT_GT(cands.front().confidence, 0.5);
    EXPECT_EQ(cands.front().skill_id,
              std::string("pin_and_diamond_locating_set"));
}

// ─── 5. feature-specific: connection_length stamped is |B − A| ──────────
TEST(SkillPinAndDiamondLocatingSet, ConnectionLengthEqualsEuclideanDistance)
{
    auto stock = skill::createCuboidStock(80.0, 50.0, 15.0);

    skill::pin_and_diamond_locating_set::Input in;
    in.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.axis_dir     = gp_Dir(0, 0, -1);
    in.round_x_mm   = 15.0; in.round_y_mm   = 10.0;
    in.diamond_x_mm = 55.0; in.diamond_y_mm = 40.0;
    in.pin_dia_mm   = 6.0;
    in.hole_depth_mm = 10.0;
    in.diamond_slot_extension_mm = 0.6;
    in.chamfer_mm   = 0.3;

    auto out = skill::pin_and_diamond_locating_set::apply(*stock, in);
    const double dx = in.diamond_x_mm - in.round_x_mm;
    const double dy = in.diamond_y_mm - in.round_y_mm;
    const double Lexpect = std::sqrt(dx * dx + dy * dy);
    EXPECT_NEAR(out.signature.pattern.at("connection_length_mm").get<double>(),
                Lexpect, 1e-6);
}
