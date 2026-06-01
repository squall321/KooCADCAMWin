// @lat: [[process/test-strategy#compound feature]]
//
// vibration_isolator_seat — 4-subfeature compound mechanical seat.
//
// Test cases:
//   1. apply produces a valid shape; volume removed AND face count increases.
//   2. validate rejects unknown stud_M.
//   3. signature carries kind=skill_id, is_compound=true, subfeature_count=4.
//   4. recognize via metadata replay returns confidence 1.0.
//   5. specific assertion: total seat depth = bushing_seat + relief + cb_depth
//      and stays below plate thickness.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/vibration_isolator_seat.hpp"

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

// ─── 1. apply produces valid shape with multiple sub-features ─────────────
TEST(SkillVibrationIsolatorSeat, ApplyProducesValidShapeWithMultipleSubFeatures)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 15.0);

    skill::vibration_isolator_seat::Input in;
    in.entry_face     = skill::FaceByNormal{ gp_Dir(0, 0, 1), 5.0, "largest" };
    in.position_x_mm  = 25.0;
    in.position_y_mm  = 25.0;
    in.axis_dir       = gp_Dir(0, 0, -1);
    in.stud_M         = "M6";
    in.bushing_od_mm  = 18.0;
    in.plate_thick_mm = 15.0;

    auto out = skill::vibration_isolator_seat::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    // Volume removed > 0 (at least the through pilot hole volume).
    const double removed = volumeOf(stock->shape()) - volumeOf(out.workpiece->shape());
    EXPECT_GT(removed, 0.0);

    // Pilot hole alone for M6 (pilot 6.6) through 15 mm plate ≈
    // π × 3.3² × 15 = 513 mm³.  Compound feature adds more, so removed
    // should be at least the pilot volume.
    const double pilotApprox = M_PI * 3.3 * 3.3 * 15.0;
    EXPECT_GT(removed, pilotApprox * 0.8);

    // Compound feature should DRAMATICALLY change face count.  Starting
    // cuboid has 6 faces.  Adding 4 concentric cylindrical/annular
    // sub-features adds at least 4 new cylindrical faces and several
    // planar steps.
    EXPECT_GT(out.workpiece->faceCount(), stock->faceCount() + 3);

    EXPECT_EQ(out.signature.skill_id, std::string("vibration_isolator_seat"));
}

// ─── 2. validate rejects bad input ────────────────────────────────────────
TEST(SkillVibrationIsolatorSeat, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 15.0);

    skill::vibration_isolator_seat::Input in;
    in.entry_face     = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm  = 25.0;
    in.position_y_mm  = 25.0;
    in.stud_M         = "M99";   // unknown
    in.bushing_od_mm  = 18.0;
    in.plate_thick_mm = 15.0;

    auto r = skill::vibration_isolator_seat::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool foundStud = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-STUD") { foundStud = true; break; }
    EXPECT_TRUE(foundStud);

    EXPECT_THROW(skill::vibration_isolator_seat::apply(*stock, in),
                 skill::SkillError);
}

// ─── 3. signature records compound kind ───────────────────────────────────
TEST(SkillVibrationIsolatorSeat, SignatureRecordsCompoundKind)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 15.0);

    skill::vibration_isolator_seat::Input in;
    in.entry_face     = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm  = 25.0;
    in.position_y_mm  = 25.0;
    in.stud_M         = "M6";
    in.bushing_od_mm  = 18.0;
    in.plate_thick_mm = 15.0;

    auto out = skill::vibration_isolator_seat::apply(*stock, in);
    const auto& pat = out.signature.pattern;

    EXPECT_EQ(pat.at("kind").get<std::string>(),
              std::string("vibration_isolator_seat"));
    EXPECT_TRUE(pat.at("is_compound").get<bool>());
    EXPECT_GE(pat.at("subfeature_count").get<int>(), 2);
    EXPECT_EQ(pat.at("subfeature_count").get<int>(), 4);
    EXPECT_EQ(pat.at("stud_M").get<std::string>(), std::string("M6"));
    EXPECT_NEAR(pat.at("bushing_od_mm").get<double>(), 18.0, 1e-6);
}

// ─── 4. recognize metadata replay ─────────────────────────────────────────
TEST(SkillVibrationIsolatorSeat, RecognizeMetadataReplay)
{
    // M8/Ø22 → total_seat_depth = 8.8 + 0.6 + 7.5 = 16.9 mm; DFM rule
    // requires plate ≥ total + 1.0, so plate = 20.0 mm leaves a 2.1 mm
    // safety margin (well above the test-5 bad-case 0.8 mm deficit).
    auto stock = skill::createCuboidStock(50.0, 50.0, 20.0);
    skill::vibration_isolator_seat::Input in;
    in.entry_face     = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm  = 25.0;
    in.position_y_mm  = 25.0;
    in.stud_M         = "M8";
    in.bushing_od_mm  = 22.0;
    in.plate_thick_mm = 20.0;
    auto out = skill::vibration_isolator_seat::apply(*stock, in);

    auto cands = skill::vibration_isolator_seat::recognize(*out.workpiece);
    ASSERT_GE(cands.size(), 1u);
    EXPECT_EQ(cands[0].skill_id,
              std::string("vibration_isolator_seat"));
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params.at("stud_M").get<std::string>(),
              std::string("M8"));
}

// ─── 5. specific assertion: seat depth < plate thickness ──────────────────
TEST(SkillVibrationIsolatorSeat, TotalSeatDepthBelowPlateThickness)
{
    // Good case M6/Ø18: total = 7.2 + 0.6 + 6.0 = 13.8.  Plate 15 mm leaves
    // a 1.2 mm safety margin > the 1.0 mm DFM clearance → DFM pass.
    auto stock = skill::createCuboidStock(50.0, 50.0, 15.0);
    skill::vibration_isolator_seat::Input in;
    in.entry_face     = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm  = 25.0;
    in.position_y_mm  = 25.0;
    in.stud_M         = "M6";
    in.bushing_od_mm  = 18.0;
    in.plate_thick_mm = 15.0;
    auto out = skill::vibration_isolator_seat::apply(*stock, in);

    const auto& pat = out.signature.pattern;
    const double total = pat.at("bushing_seat_depth_mm").get<double>()
                         + pat.at("relief_depth_mm").get<double>()
                         + pat.at("cb_depth_mm").get<double>();
    EXPECT_LT(total, in.plate_thick_mm);
    // Source-of-truth: assert the pattern matches the skill's own helpers
    // (no hard-coded 0.4 / 0.6 / +1.0 constants in the test).
    const double expected =
        skill::vibration_isolator_seat::totalSeatDepth(in.stud_M, in.bushing_od_mm);
    EXPECT_NEAR(total, expected, 1e-6);

    // Bad case M6/Ø18 with plate = 10 mm: deficit 13.8 + 1 − 10 = 4.8 mm,
    // well above the 1 mm DFM clearance — DFM must reject.
    skill::vibration_isolator_seat::Input bad = in;
    bad.plate_thick_mm = 10.0;
    auto r = skill::vibration_isolator_seat::validate(*stock, bad);
    EXPECT_FALSE(r.passed);
    bool foundPlate = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-PLATE-T") { foundPlate = true; break; }
    EXPECT_TRUE(foundPlate);
}
