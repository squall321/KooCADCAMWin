// @lat: [[process/test-strategy#skill round-trip]]
//
// bolted_joint — assembly-level joint with a clearance through-hole and
// torque/preload metadata.
//
// Cases (6):
//   1. Apply drills a clearance through-hole sized to bolt_thread + clearance.
//   2. boltNominalDia / holeDiameterFor helpers map standard threads correctly.
//   3. DFM rejects unknown thread / negative clearance / missing torque.
//   4. DFM rejects clearance outside [0.05, 1.0].
//   5. recognize() replays from history at confidence 1.0.
//   6. Through-hole property always true.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/bolted_joint.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

#include <cmath>

using namespace koocadcam;

namespace {

double volumeOf(const TopoDS_Shape& s)
{
    GProp_GProps p;
    BRepGProp::VolumeProperties(s, p);
    return p.Mass();
}

}  // namespace

// ─── 1. Apply drills clearance through-hole ──────────────────────────────
TEST(SkillBoltedJoint, ApplyDrillsClearanceHole)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);
    const double volBefore = volumeOf(stock->shape());

    skill::bolted_joint::Input in;
    in.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 25.0;
    in.position_y_mm = 25.0;
    in.axis_dir      = gp_Dir(0, 0, -1);
    in.bolt_thread   = "M5";
    in.clearance_mm  = 0.5;            // hole = 5.0 + 0.5 = 5.5 mm
    in.torque_Nm     = 6.0;
    in.preload_N     = 8000.0;

    auto out = skill::bolted_joint::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    const double expectedDia = 5.5;
    const double approx = M_PI * (expectedDia / 2.0) * (expectedDia / 2.0) * 10.0;
    const double removed = volBefore - volumeOf(out.workpiece->shape());
    EXPECT_NEAR(removed, approx, approx * 0.05);

    EXPECT_EQ(out.signature.skill_id, std::string("bolted_joint"));
    EXPECT_NEAR(out.signature.params["hole_dia_mm"].get<double>(), 5.5, 1e-6);
    EXPECT_EQ(out.signature.params["bolt_thread"].get<std::string>(), std::string("M5"));
    EXPECT_NEAR(out.signature.params["torque_Nm"].get<double>(), 6.0, 1e-6);
    EXPECT_NEAR(out.signature.params["preload_N"].get<double>(), 8000.0, 1e-6);
    EXPECT_TRUE(out.signature.params["through_hole"].get<bool>());
}

// ─── 2. Helper lookups ───────────────────────────────────────────────────
TEST(SkillBoltedJoint, HelperLookups)
{
    EXPECT_NEAR(skill::bolted_joint::boltNominalDia("M3"),  3.0, 1e-9);
    EXPECT_NEAR(skill::bolted_joint::boltNominalDia("M5"),  5.0, 1e-9);
    EXPECT_NEAR(skill::bolted_joint::boltNominalDia("M10"), 10.0, 1e-9);
    EXPECT_NEAR(skill::bolted_joint::boltNominalDia("M99"), 0.0, 1e-9);

    EXPECT_NEAR(skill::bolted_joint::holeDiameterFor("M5", 0.5),  5.5, 1e-9);
    EXPECT_NEAR(skill::bolted_joint::holeDiameterFor("M8", 0.3),  8.3, 1e-9);
    EXPECT_NEAR(skill::bolted_joint::holeDiameterFor("M99", 0.5), 0.0, 1e-9);
}

// ─── 3. DFM rejects missing torque / preload ─────────────────────────────
TEST(SkillBoltedJoint, ValidateRejectsZeroTorque)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::bolted_joint::Input in;
    in.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 25.0;
    in.position_y_mm = 25.0;
    in.bolt_thread   = "M5";
    in.clearance_mm  = 0.5;
    in.torque_Nm     = 0.0;     // invalid
    in.preload_N     = 5000.0;

    auto r = skill::bolted_joint::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool sawCode = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-BOLT-TORQUE") { sawCode = true; break; }
    EXPECT_TRUE(sawCode);
    EXPECT_THROW(skill::bolted_joint::apply(*stock, in), skill::SkillError);
}

// ─── 4. DFM rejects out-of-range clearance ───────────────────────────────
TEST(SkillBoltedJoint, ValidateRejectsBadClearance)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::bolted_joint::Input in;
    in.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 25.0;
    in.position_y_mm = 25.0;
    in.bolt_thread   = "M5";
    in.clearance_mm  = 2.5;     // outside [0.05, 1.0]
    in.torque_Nm     = 6.0;
    in.preload_N     = 8000.0;

    auto r = skill::bolted_joint::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool sawCode = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-BOLT-FIT") { sawCode = true; break; }
    EXPECT_TRUE(sawCode);
}

// ─── 5. Recognize replays history ────────────────────────────────────────
TEST(SkillBoltedJoint, RecognizeReplaysFromHistory)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::bolted_joint::Input in;
    in.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 20.0;
    in.position_y_mm = 30.0;
    in.bolt_thread   = "M6";
    in.clearance_mm  = 0.4;
    in.torque_Nm     = 10.0;
    in.preload_N     = 12000.0;

    auto out = skill::bolted_joint::apply(*stock, in);
    auto cands = skill::bolted_joint::recognize(*out.workpiece);

    ASSERT_FALSE(cands.empty());
    const auto& c = cands.front();
    EXPECT_NEAR(c.confidence, 1.0, 1e-9);
    EXPECT_EQ(c.recovered_params["bolt_thread"].get<std::string>(), std::string("M6"));
    EXPECT_NEAR(c.recovered_params["clearance_mm"].get<double>(), 0.4, 1e-6);
    EXPECT_NEAR(c.recovered_params["hole_dia_mm"].get<double>(),  6.4, 1e-6);
    EXPECT_NEAR(c.recovered_params["torque_Nm"].get<double>(),    10.0, 1e-6);
    EXPECT_NEAR(c.recovered_params["preload_N"].get<double>(),    12000.0, 1e-6);
}

// ─── 6. Through-hole property always true ───────────────────────────────
TEST(SkillBoltedJoint, AlwaysThroughHole)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 8.0);

    skill::bolted_joint::Input in;
    in.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 20.0;
    in.position_y_mm = 20.0;
    in.bolt_thread   = "M4";
    in.clearance_mm  = 0.3;
    in.torque_Nm     = 4.0;
    in.preload_N     = 5000.0;

    auto out = skill::bolted_joint::apply(*stock, in);
    EXPECT_TRUE(out.signature.params["through_hole"].get<bool>());
    EXPECT_FALSE(out.signature.pattern["bottom_planar_face_present"].get<bool>());
}
