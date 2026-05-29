// @lat: [[process/test-strategy#skill round-trip]]
//
// erection_lift — single dock-side block hoist.  Covers:
//   1. apply preserves geometry verbatim
//   2. validate rejects overloaded crane (mass > capacity)
//   3. signature records lift_id + mass + height + capacity + utilization
//   4. recognize replays metadata
//   5. utilization > 0.85 triggers DFM-LIFT-UTIL info (not blocking)

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/erection_lift.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

using namespace koocadcam;

namespace {

double volumeOf(const TopoDS_Shape& s)
{
    GProp_GProps p;
    BRepGProp::VolumeProperties(s, p);
    return p.Mass();
}

bool hasFinding(const skill::DFMReport& r, const std::string& code)
{
    for (const auto& f : r.findings) if (f.code == code) return true;
    return false;
}

}  // namespace

// ─── 1. Apply clones geometry unchanged ──────────────────────────────────
TEST(SkillErectionLift, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0, "marine_steel");
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    skill::erection_lift::Input in;
    in.lift_id          = "LFT-42";
    in.mass_t           = 400.0;
    in.lift_height_m    = 30.0;
    in.crane_capacity_t = 800.0;

    auto out = skill::erection_lift::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.workpiece->features().size(), 1u);
}

// ─── 2. validate rejects overloaded crane ────────────────────────────────
TEST(SkillErectionLift, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0, "marine_steel");

    skill::erection_lift::Input in;
    in.lift_id          = "LFT-X";
    in.mass_t           = 1200.0;      // exceeds capacity
    in.lift_height_m    = 25.0;
    in.crane_capacity_t = 800.0;

    auto r = skill::erection_lift::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-LIFT-CAPACITY"));
    EXPECT_THROW(skill::erection_lift::apply(*stock, in), skill::SkillError);

    // Non-positive height also blocks.
    in.mass_t        = 100.0;
    in.lift_height_m = 0.0;
    auto r2 = skill::erection_lift::validate(*stock, in);
    EXPECT_FALSE(r2.passed);
}

// ─── 3. signature records kind + key params ──────────────────────────────
TEST(SkillErectionLift, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0, "marine_steel");

    skill::erection_lift::Input in;
    in.lift_id          = "LFT-A1";
    in.mass_t           = 500.0;
    in.lift_height_m    = 40.0;
    in.crane_capacity_t = 1000.0;

    auto out = skill::erection_lift::apply(*stock, in);
    EXPECT_EQ(out.signature.skill_id, std::string("erection_lift"));
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("erection_lift"));
    EXPECT_EQ(out.signature.pattern["lift_id"].get<std::string>(),
              std::string("LFT-A1"));
    EXPECT_NEAR(out.signature.pattern["mass_t"].get<double>(), 500.0, 1e-9);
    EXPECT_NEAR(out.signature.pattern["lift_height_m"].get<double>(), 40.0, 1e-9);
    EXPECT_NEAR(out.signature.pattern["crane_capacity_t"].get<double>(),
                1000.0, 1e-9);
    EXPECT_NEAR(out.signature.pattern["utilization"].get<double>(), 0.5, 1e-6);
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
}

// ─── 4. recognize replays metadata ───────────────────────────────────────
TEST(SkillErectionLift, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0, "marine_steel");

    skill::erection_lift::Input in;
    in.lift_id          = "LFT-Z";
    in.mass_t           = 200.0;
    in.lift_height_m    = 15.0;
    in.crane_capacity_t = 800.0;
    auto out = skill::erection_lift::apply(*stock, in);

    auto cands = skill::erection_lift::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["lift_id"].get<std::string>(),
              std::string("LFT-Z"));

    skill::Workpiece raw(out.workpiece->shape());
    EXPECT_TRUE(skill::erection_lift::recognize(raw).empty());
}

// ─── 5. High utilization (>0.85) is info, not blocking ───────────────────
TEST(SkillErectionLift, HighUtilizationEmitsInfoNotError)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0, "marine_steel");

    skill::erection_lift::Input in;
    in.lift_id          = "LFT-near-max";
    in.mass_t           = 720.0;       // utilization 0.9 -> info
    in.lift_height_m    = 20.0;
    in.crane_capacity_t = 800.0;

    auto r = skill::erection_lift::validate(*stock, in);
    EXPECT_TRUE(r.passed)
        << "utilization at 0.9 should emit info, not block";
    EXPECT_TRUE(hasFinding(r, "DFM-LIFT-UTIL"));
    EXPECT_NO_THROW(skill::erection_lift::apply(*stock, in));
}
