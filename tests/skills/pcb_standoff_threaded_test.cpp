// @lat: [[process/test-strategy#compound feature]]
//
// pcb_standoff_threaded — 3-op compound: boss protrusion + PCB clearance
// recess + tapped blind hole.
//
// Test cases:
//   1. ApplyProducesValidShapeWithMultipleSubFeatures (boss ADDS material;
//      tap + recess REMOVE — verify net shape volume changes correctly)
//   2. ValidateRejectsBadInput — unknown thread size
//   3. SignatureRecordsCompoundKind
//   4. RecognizeMetadataReplay
//   5. CaptiveNutHexFlatMatchesMSizeTable — M3 → 5.5 mm hex flats

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/pcb_standoff_threaded.hpp"

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

// ─── 1. Apply produces valid shape ────────────────────────────────────────
TEST(SkillPcbStandoffThreaded, ApplyProducesValidShapeWithMultipleSubFeatures)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 6.0);

    skill::pcb_standoff_threaded::Input in;
    in.face            = skill::FaceByNormal{ gp_Dir(0, 0, 1), 5.0, "largest" };
    in.position_x_mm   = 25.0;
    in.position_y_mm   = 25.0;
    in.axis_dir        = gp_Dir(0, 0, 1);
    in.thread_M        = "M3";
    in.boss_height_mm  = 6.0;

    const int facesBefore = stock->faceCount();
    const double stockVol = volumeOf(stock->shape());
    auto out = skill::pcb_standoff_threaded::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    // boss adds material; tap (pilot Ø2.5) + recess remove some.
    // Boss default = pilot + 4 = 6.5 mm Ø, height 6 → adds ~199 mm³
    // Tap: π × 1.25² × 5.5 = ~27 mm³ removed
    // Recess: ring (7.5² − 2.5²) × 0.5 × π / 4 → ~19.6 mm³ removed
    // Net add ≈ 152 mm³
    const double resultVol = volumeOf(out.workpiece->shape());
    EXPECT_GT(resultVol, stockVol);   // NET addition (boss > removals)

    // Face count must grow — boss + recess + tap = many new faces
    EXPECT_GT(out.workpiece->faceCount(), facesBefore);

    EXPECT_EQ(out.signature.skill_id, std::string("pcb_standoff_threaded"));
}

// ─── 2. Validate rejects unknown thread ───────────────────────────────────
TEST(SkillPcbStandoffThreaded, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 6.0);
    skill::pcb_standoff_threaded::Input in;
    in.face            = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm   = 25.0;
    in.position_y_mm   = 25.0;
    in.thread_M        = "M99";   // not in table
    in.boss_height_mm  = 6.0;

    auto r = skill::pcb_standoff_threaded::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-TAP-SIZE") { found = true; break; }
    EXPECT_TRUE(found);

    EXPECT_THROW(skill::pcb_standoff_threaded::apply(*stock, in), skill::SkillError);
}

// ─── 3. Signature records compound kind ───────────────────────────────────
TEST(SkillPcbStandoffThreaded, SignatureRecordsCompoundKind)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 6.0);
    skill::pcb_standoff_threaded::Input in;
    in.face            = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm   = 25.0;
    in.position_y_mm   = 25.0;
    in.thread_M        = "M3";
    in.boss_height_mm  = 6.0;
    auto out = skill::pcb_standoff_threaded::apply(*stock, in);

    const auto& pat = out.signature.pattern;
    EXPECT_TRUE(pat.at("is_compound").get<bool>());
    EXPECT_GE(pat.at("subfeature_count").get<int>(), 2);
    EXPECT_EQ(pat.at("kind").get<std::string>(),
              std::string("pcb_standoff_threaded"));
    EXPECT_EQ(pat.at("thread_M").get<std::string>(), std::string("M3"));
    EXPECT_NEAR(pat.at("pilot_dia_mm").get<double>(), 2.5, 1e-6);
}

// ─── 4. Recognize: metadata replay → confidence 1.0 ───────────────────────
TEST(SkillPcbStandoffThreaded, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 6.0);
    skill::pcb_standoff_threaded::Input in;
    in.face            = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm   = 25.0;
    in.position_y_mm   = 25.0;
    in.thread_M        = "M4";
    in.boss_height_mm  = 5.0;
    auto out = skill::pcb_standoff_threaded::apply(*stock, in);

    auto cands = skill::pcb_standoff_threaded::recognize(*out.workpiece);
    ASSERT_GE(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-9);
    EXPECT_EQ(cands[0].recovered_params["thread_M"].get<std::string>(),
              std::string("M4"));
}

// ─── 5. Captive-nut hex flat-to-flat matches the M-size table ─────────────
TEST(SkillPcbStandoffThreaded, CaptiveNutHexFlatMatchesMSizeTable)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 6.0);
    skill::pcb_standoff_threaded::Input in;
    in.face            = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm   = 25.0;
    in.position_y_mm   = 25.0;
    in.thread_M        = "M3";
    in.boss_height_mm  = 6.0;
    auto out = skill::pcb_standoff_threaded::apply(*stock, in);

    const auto& pat = out.signature.pattern;
    // M3 hex nut: flat-to-flat = 5.5 mm (ISO 4032)
    EXPECT_NEAR(pat.at("captive_nut_hex_mm").get<double>(), 5.5, 0.1);
    // Pilot table lookup directly
    EXPECT_NEAR(skill::pcb_standoff_threaded::pilotDiameterFor("M3"), 2.5, 1e-6);
    EXPECT_NEAR(skill::pcb_standoff_threaded::pilotDiameterFor("M5"), 4.2, 1e-6);
    EXPECT_EQ(skill::pcb_standoff_threaded::pilotDiameterFor("M99"), 0.0);
}
