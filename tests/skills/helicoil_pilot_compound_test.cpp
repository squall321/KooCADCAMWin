// @lat: [[process/test-strategy#compound fastener-seat]]
//
// helicoil_pilot_compound — COMPOUND macro: drill_pilot + thread_pilot
// (slice-1 same cylinder) + 90° countersink at entry.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/helicoil_pilot_compound.hpp"

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

// ─── 1. apply produces valid shape with N sub-features ───────────────────
TEST(SkillHelicoilPilotCompound, ApplyProducesValidShapeWithMultipleSubFeatures)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 20.0);
    const int stockFaceCount = stock->faceCount();
    const double stockVol = volumeOf(stock->shape());

    skill::helicoil_pilot_compound::Input in;
    in.entry_face         = skill::FaceByNormal{ gp_Dir(0, 0, 1), 5.0, "largest" };
    in.position_x_mm      = 25.0;
    in.position_y_mm      = 25.0;
    in.target_thread_size = "M4";
    in.insert_length_mm   = 4.0;        // 1×D insert

    auto out = skill::helicoil_pilot_compound::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    // M4 STI tap drill = 4.10 mm.  Total depth = 4.0 × 1.5 + 0.5 = 6.5 mm.
    // Plus the cone frustum atop (90°, 4.8 → 4.1, dep ≈ 0.35 mm).
    const double pilotDia = 4.10;
    const double totalDep = 4.0 * 1.5 + 0.5;
    const double pilotR   = pilotDia / 2.0;
    const double pilotVol = M_PI * pilotR * pilotR * totalDep;
    const double removed  = stockVol - volumeOf(out.workpiece->shape());
    // Cone adds a small extra; allow 15% tolerance.
    EXPECT_GE(removed, pilotVol * 0.95);
    EXPECT_LE(removed, pilotVol * 1.30);

    EXPECT_GT(out.workpiece->faceCount(), stockFaceCount);
    EXPECT_TRUE(out.signature.pattern.at("is_compound").get<bool>());
    EXPECT_EQ(out.signature.pattern.at("subfeature_count").get<int>(), 3);
}

// ─── 2. validate rejects bad input ───────────────────────────────────────
TEST(SkillHelicoilPilotCompound, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 20.0);

    // (a) Unknown target size
    {
        skill::helicoil_pilot_compound::Input in;
        in.entry_face         = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
        in.target_thread_size = "M99";
        in.insert_length_mm   = 4.0;
        auto r = skill::helicoil_pilot_compound::validate(*stock, in);
        EXPECT_FALSE(r.passed);
        bool found = false;
        for (const auto& f : r.findings)
            if (f.code == "DFM-HELI-SIZE") { found = true; break; }
        EXPECT_TRUE(found);
        EXPECT_THROW(skill::helicoil_pilot_compound::apply(*stock, in), skill::SkillError);
    }
    // (b) Zero insert length
    {
        skill::helicoil_pilot_compound::Input in;
        in.entry_face         = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
        in.target_thread_size = "M4";
        in.insert_length_mm   = 0.0;
        auto r = skill::helicoil_pilot_compound::validate(*stock, in);
        EXPECT_FALSE(r.passed);
        EXPECT_THROW(skill::helicoil_pilot_compound::apply(*stock, in), skill::SkillError);
    }
    // (c) Too thin stock
    {
        auto thin = skill::createCuboidStock(50.0, 50.0, 2.0);
        skill::helicoil_pilot_compound::Input in;
        in.entry_face         = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
        in.position_x_mm = 25.0; in.position_y_mm = 25.0;
        in.target_thread_size = "M6";
        in.insert_length_mm   = 6.0;
        auto r = skill::helicoil_pilot_compound::validate(*thin, in);
        EXPECT_FALSE(r.passed);
    }
}

// ─── 3. Signature records compound kind ──────────────────────────────────
TEST(SkillHelicoilPilotCompound, SignatureRecordsCompoundKind)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 30.0);

    skill::helicoil_pilot_compound::Input in;
    in.entry_face         = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm      = 25.0;
    in.position_y_mm      = 25.0;
    in.target_thread_size = "M5";
    in.insert_length_mm   = 5.0;

    auto out = skill::helicoil_pilot_compound::apply(*stock, in);
    EXPECT_EQ(out.signature.pattern.at("kind").get<std::string>(),
              std::string("helicoil_pilot_compound"));
    EXPECT_TRUE(out.signature.pattern.at("is_compound").get<bool>());
    EXPECT_GE(out.signature.pattern.at("subfeature_count").get<int>(), 2);
}

// ─── 4. recognize replays metadata at confidence 1.0 ─────────────────────
TEST(SkillHelicoilPilotCompound, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 20.0);

    skill::helicoil_pilot_compound::Input in;
    in.entry_face         = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm      = 25.0;
    in.position_y_mm      = 25.0;
    in.target_thread_size = "M4";
    in.insert_length_mm   = 4.0;

    auto out = skill::helicoil_pilot_compound::apply(*stock, in);
    auto cands = skill::helicoil_pilot_compound::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_DOUBLE_EQ(cands[0].confidence, 1.0);
    EXPECT_EQ(cands[0].recovered_params.at("target_thread_size").get<std::string>(),
              std::string("M4"));
}

// ─── 5. Specific: STI tap-drill diameter matches Heli-Coil chart ─────────
TEST(SkillHelicoilPilotCompound, StiTapDrillMatchesChart)
{
    // Heli-Coil STI Tap-Drill chart values (metric coarse, 1×D inserts).
    EXPECT_NEAR(skill::helicoil_pilot_compound::stiTapDrillFor("M3"),  3.10, 1e-3);
    EXPECT_NEAR(skill::helicoil_pilot_compound::stiTapDrillFor("M4"),  4.10, 1e-3);
    EXPECT_NEAR(skill::helicoil_pilot_compound::stiTapDrillFor("M5"),  5.20, 1e-3);
    EXPECT_NEAR(skill::helicoil_pilot_compound::stiTapDrillFor("M8"),  8.30, 1e-3);
    EXPECT_EQ  (skill::helicoil_pilot_compound::stiTapDrillFor("M99"), 0.0);

    // STI tap-drill must be LARGER than the nominal thread D
    // (because the insert thickens the wall).  Sanity-check 2 cases:
    EXPECT_GT(skill::helicoil_pilot_compound::stiTapDrillFor("M4"), 4.0);
    EXPECT_GT(skill::helicoil_pilot_compound::stiTapDrillFor("M6"), 6.0);

    auto stock = skill::createCuboidStock(50.0, 50.0, 20.0);
    skill::helicoil_pilot_compound::Input in;
    in.entry_face         = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 25.0; in.position_y_mm = 25.0;
    in.target_thread_size = "M4";
    in.insert_length_mm   = 4.0;
    auto out = skill::helicoil_pilot_compound::apply(*stock, in);
    EXPECT_NEAR(out.signature.pattern.at("sti_tap_drill_mm").get<double>(), 4.10, 1e-6);
}
