// @lat: [[process/test-strategy#skill round-trip]]
//
// rocket_engine_test skill — hot-fire qualification no-op sweep.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/rocket_engine_test.hpp"

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

// ─── 1. Apply: geometry CLONED unchanged ──────────────────────────────────
TEST(SkillRocketEngineTest, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(200.0, 200.0, 500.0, "inconel_718");
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    skill::rocket_engine_test::Input in;
    in.engine_id   = "ENG-RAPTOR-007";
    in.thrust_kn   = 2300.0;
    in.burn_time_s = 180.0;

    auto out = skill::rocket_engine_test::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("rocket_engine_test"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. ValidateRejectsBadInput: zero thrust → error ─────────────────────
TEST(SkillRocketEngineTest, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(200.0, 200.0, 500.0);

    skill::rocket_engine_test::Input in;
    in.engine_id   = "ENG-BAD";
    in.thrust_kn   = 0.0;        // invalid
    in.burn_time_s = 60.0;

    auto r = skill::rocket_engine_test::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-SPACE-THRUST"));
    EXPECT_THROW(skill::rocket_engine_test::apply(*stock, in), skill::SkillError);

    // empty engine_id independently
    skill::rocket_engine_test::Input in2;
    in2.engine_id   = "";
    in2.thrust_kn   = 100.0;
    in2.burn_time_s = 60.0;
    auto r2 = skill::rocket_engine_test::validate(*stock, in2);
    EXPECT_FALSE(r2.passed);
    EXPECT_TRUE(hasFinding(r2, "DFM-INPUT"));
}

// ─── 3. Signature records kind + engine_id + thrust + burn_time ──────────
TEST(SkillRocketEngineTest, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCuboidStock(200.0, 200.0, 500.0);

    skill::rocket_engine_test::Input in;
    in.engine_id   = "ENG-MERLIN-042";
    in.thrust_kn   = 845.0;
    in.burn_time_s = 162.0;

    auto out = skill::rocket_engine_test::apply(*stock, in);
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("rocket_engine_test"));
    EXPECT_EQ(out.signature.pattern["engine_id"].get<std::string>(),
              std::string("ENG-MERLIN-042"));
    EXPECT_NEAR(out.signature.pattern["thrust_kn"].get<double>(),   845.0, 1e-9);
    EXPECT_NEAR(out.signature.pattern["burn_time_s"].get<double>(), 162.0, 1e-9);
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
}

// ─── 4. Recognize via metadata replay ─────────────────────────────────────
TEST(SkillRocketEngineTest, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(200.0, 200.0, 500.0);

    skill::rocket_engine_test::Input in;
    in.engine_id   = "ENG-VULCAIN-009";
    in.thrust_kn   = 1370.0;
    in.burn_time_s = 540.0;

    auto out  = skill::rocket_engine_test::apply(*stock, in);
    auto cands = skill::rocket_engine_test::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["engine_id"].get<std::string>(),
              std::string("ENG-VULCAIN-009"));

    skill::Workpiece raw(out.workpiece->shape());
    EXPECT_TRUE(skill::rocket_engine_test::recognize(raw).empty())
        << "rocket_engine_test cannot be recognized geometrically";
}

// ─── 5. SPECIFIC: total_impulse derived from thrust × burn_time ──────────
TEST(SkillRocketEngineTest, TotalImpulseDerivedFromThrustBurn)
{
    auto stock = skill::createCuboidStock(200.0, 200.0, 500.0);

    skill::rocket_engine_test::Input in;
    in.engine_id   = "ENG-IMP";
    in.thrust_kn   = 500.0;
    in.burn_time_s = 200.0;

    auto out = skill::rocket_engine_test::apply(*stock, in);
    EXPECT_NEAR(out.signature.pattern["total_impulse_kns"].get<double>(),
                500.0 * 200.0, 1e-6);

    // Tooling cycle time includes burn + safe period.
    EXPECT_GT(out.signature.tooling.est_cycle_time_s, in.burn_time_s);
}
