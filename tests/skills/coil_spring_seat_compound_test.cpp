// @lat: [[process/test-strategy#compound feature]]
//
// coil_spring_seat_compound — 3 sub-feature chain:
//   spring pocket + pilot pin + annular shoulder.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/coil_spring_seat_compound.hpp"

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

// ─── 1. apply produces a valid shape with multiple sub-features ───────────
TEST(SkillCoilSpringSeat, ApplyProducesValidShapeWithMultipleSubFeatures)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 40.0);

    skill::coil_spring_seat_compound::Input in;
    in.face_id        = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm  = 30.0;
    in.position_y_mm  = 30.0;
    in.axis_dir       = gp_Dir(0, 0, -1);
    in.spring_od      = 12.0;
    in.free_height    = 20.0;
    in.pilot_dia      = 4.0;
    in.slip_fit_mm    = 0.2;

    const double v0 = volumeOf(stock->shape());
    const int    n0 = stock->faceCount();
    auto out = skill::coil_spring_seat_compound::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    // Volume removed = pocket_vol + (pilot_depth - pocket_depth) × pilot_area.
    const double pocketR  = (12.0 + 2.0 * 0.2) / 2.0;
    const double pocketD  = 20.0 + 1.0;       // free_height + 1 mm
    const double pilotD   = 20.0 + 4.0;       // free_height + 4 mm
    const double pilotR   = 4.0 / 2.0;
    const double approx   = M_PI * pocketR * pocketR * pocketD
                          + M_PI * pilotR  * pilotR  * (pilotD - pocketD);
    const double v1 = volumeOf(out.workpiece->shape());
    EXPECT_NEAR(v0 - v1, approx, approx * 0.05);

    // Face count must increase (multiple sub-features = multiple new walls).
    EXPECT_GT(out.workpiece->faceCount(), n0);
}

// ─── 2. validate rejects bad input (pilot dia too large) ─────────────────
TEST(SkillCoilSpringSeat, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 40.0);

    skill::coil_spring_seat_compound::Input in;
    in.face_id        = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm  = 30.0;
    in.position_y_mm  = 30.0;
    in.spring_od      = 10.0;
    in.free_height    = 15.0;
    in.pilot_dia      = 9.0;        // 0.9 × OD > 0.5 × OD — should reject
    in.slip_fit_mm    = 0.2;

    auto r = skill::coil_spring_seat_compound::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool foundPilot = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-SPRING-PILOT") { foundPilot = true; break; }
    EXPECT_TRUE(foundPilot);

    EXPECT_THROW(skill::coil_spring_seat_compound::apply(*stock, in),
                 skill::SkillError);
}

// ─── 3. signature records is_compound + subfeature_count >= 2 ────────────
TEST(SkillCoilSpringSeat, SignatureRecordsCompoundKind)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 40.0);

    skill::coil_spring_seat_compound::Input in;
    in.face_id        = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm  = 30.0;
    in.position_y_mm  = 30.0;
    in.spring_od      = 14.0;
    in.free_height    = 22.0;
    in.pilot_dia      = 5.0;
    in.slip_fit_mm    = 0.2;

    auto out = skill::coil_spring_seat_compound::apply(*stock, in);

    EXPECT_EQ(out.signature.skill_id, std::string("coil_spring_seat_compound"));
    EXPECT_EQ(out.signature.pattern.at("kind"),
              std::string("coil_spring_seat_compound"));
    EXPECT_TRUE(out.signature.pattern.at("is_compound").get<bool>());
    EXPECT_GE(out.signature.pattern.at("subfeature_count").get<int>(), 2);
    EXPECT_EQ(out.signature.pattern.at("subfeature_count").get<int>(), 3);
    EXPECT_NEAR(out.signature.params.at("spring_od").get<double>(), 14.0, 1e-9);
    EXPECT_NEAR(out.signature.params.at("pilot_dia").get<double>(), 5.0, 1e-9);
}

// ─── 4. recognize() replays metadata at confidence 1.0 ───────────────────
TEST(SkillCoilSpringSeat, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 40.0);

    skill::coil_spring_seat_compound::Input in;
    in.face_id        = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm  = 30.0;
    in.position_y_mm  = 30.0;
    in.spring_od      = 10.0;
    in.free_height    = 18.0;
    in.pilot_dia      = 3.5;
    in.slip_fit_mm    = 0.2;

    auto out = skill::coil_spring_seat_compound::apply(*stock, in);
    auto cands = skill::coil_spring_seat_compound::recognize(*out.workpiece);

    ASSERT_EQ(cands.size(), 1u);
    EXPECT_EQ(cands[0].skill_id, std::string("coil_spring_seat_compound"));
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-9);
    EXPECT_NEAR(cands[0].recovered_params.at("spring_od").get<double>(),
                10.0, 1e-9);
    EXPECT_NEAR(cands[0].recovered_params.at("pilot_dia").get<double>(),
                3.5, 1e-9);
}

// ─── 5. specific: pilot_depth > pocket_depth (annular shoulder exists) ────
TEST(SkillCoilSpringSeat, PilotIsDeeperThanPocketLeavingShoulder)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 40.0);

    skill::coil_spring_seat_compound::Input in;
    in.face_id        = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm  = 30.0;
    in.position_y_mm  = 30.0;
    in.spring_od      = 12.0;
    in.free_height    = 20.0;
    in.pilot_dia      = 4.0;
    in.slip_fit_mm    = 0.2;

    auto out = skill::coil_spring_seat_compound::apply(*stock, in);

    const double pocketD = out.signature.pattern.at("pocket_depth_mm").get<double>();
    const double pilotD  = out.signature.pattern.at("pilot_depth_mm").get<double>();
    EXPECT_GT(pilotD, pocketD);                    // pilot is deeper
    const double shoulder = out.signature.pattern.at("shoulder_height_mm").get<double>();
    EXPECT_NEAR(shoulder, pilotD - pocketD, 1e-9);  // shoulder = pilot - pocket
    EXPECT_GT(shoulder, 0.0);
    EXPECT_TRUE(out.signature.pattern.at("annular_shoulder_present").get<bool>());
}
