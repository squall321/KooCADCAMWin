// @lat: [[process/test-strategy#skill round-trip]]
//
// interior_install — 5-case sweep covering identity-geometry synthesis,
// component_count / harness_length DFM gates, signature, metadata replay,
// and the cycle-time formula.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/interior_install.hpp"

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

}  // namespace

// ── 1. Apply: geometry COMPLETELY unchanged ──────────────────────────────
TEST(SkillInteriorInstall, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    skill::interior_install::Input in;
    in.component_count  = 30;
    in.harness_length_m = 70.0;

    auto out = skill::interior_install::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("interior_install"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ── 2. DFM: non-positive counts rejected ─────────────────────────────────
TEST(SkillInteriorInstall, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::interior_install::Input bad;
    bad.component_count  = 0;                  // invalid
    bad.harness_length_m = 10.0;
    EXPECT_FALSE(skill::interior_install::validate(*stock, bad).passed);
    EXPECT_THROW(skill::interior_install::apply(*stock, bad),
                 skill::SkillError);

    bad.component_count  = -3;                 // invalid
    EXPECT_FALSE(skill::interior_install::validate(*stock, bad).passed);

    bad.component_count  = 10;
    bad.harness_length_m = 0.0;                // invalid
    EXPECT_FALSE(skill::interior_install::validate(*stock, bad).passed);

    bad.harness_length_m = -5.0;               // invalid
    EXPECT_FALSE(skill::interior_install::validate(*stock, bad).passed);
}

// ── 3. Signature records kind + key params ───────────────────────────────
TEST(SkillInteriorInstall, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::interior_install::Input in;
    in.component_count  = 42;
    in.harness_length_m = 88.25;

    auto out = skill::interior_install::apply(*stock, in);
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("interior_install"));
    EXPECT_EQ(out.signature.pattern["component_count"].get<int>(), 42);
    EXPECT_NEAR(out.signature.pattern["harness_length_m"].get<double>(),
                88.25, 1e-9);
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
    EXPECT_EQ(out.signature.tooling.tool_type,
              std::string("interior_install_station"));
}

// ── 4. Recognize via metadata replay ─────────────────────────────────────
TEST(SkillInteriorInstall, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::interior_install::Input in;
    in.component_count  = 28;
    in.harness_length_m = 65.0;

    auto out = skill::interior_install::apply(*stock, in);
    auto cands = skill::interior_install::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["component_count"].get<int>(), 28);
    EXPECT_NEAR(cands[0].recovered_params["harness_length_m"].get<double>(),
                65.0, 1e-9);

    // Same shape without metadata → empty.
    skill::Workpiece raw(out.workpiece->shape());
    EXPECT_TRUE(skill::interior_install::recognize(raw).empty());
}

// ── 5. Cycle-time formula: 30 + 30*components + 8*harness ────────────────
TEST(SkillInteriorInstall, CycleTimeMatchesFormula)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::interior_install::Input in;
    in.component_count  = 25;
    in.harness_length_m = 60.0;

    auto out = skill::interior_install::apply(*stock, in);
    // 30 + 30*25 + 8*60 = 30 + 750 + 480 = 1260.
    EXPECT_NEAR(out.signature.tooling.est_cycle_time_s, 1260.0, 1e-9);
}
