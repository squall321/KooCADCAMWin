// @lat: [[process/test-strategy#skill round-trip]]
//
// open_die_forge — flat-die hammer forging that reduces thickness on one
// bbox axis (similar to rolling, but without a shape constraint).

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/open_die_forge.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

#include <algorithm>
#include <cmath>

using namespace koocadcam;

namespace {
double volumeOf(const TopoDS_Shape& s) {
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}
double smallestExtent(const skill::Workpiece& wp) {
    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    return std::min({ xMax - xMin, yMax - yMin, zMax - zMin });
}
}  // namespace

// ── 1. ApplyHandlesInput — reduces stock thickness ──────────────────────
TEST(SkillOpenDieForge, ApplyHandlesInput)
{
    auto stock = skill::createCuboidStock(100.0, 80.0, 30.0);
    const double t0 = smallestExtent(*stock);   // = 30 mm (Z is smallest)

    skill::open_die_forge::Input in;
    in.reduction_pct = 25.0;
    in.temp_c        = 1100.0;

    auto out = skill::open_die_forge::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    const double t1 = smallestExtent(*out.workpiece);
    EXPECT_NEAR(t1, t0 * 0.75, 1e-6);
    EXPECT_EQ(out.signature.skill_id, std::string("open_die_forge"));
}

// ── 2. ValidateRejectsBadInput — > 80% reduction & negative temp ────────
TEST(SkillOpenDieForge, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(100.0, 80.0, 30.0);

    skill::open_die_forge::Input over;
    over.reduction_pct = 90.0;        // > 80% triggers DFM-FORGE-OVER
    over.temp_c        = 1100.0;
    auto r1 = skill::open_die_forge::validate(*stock, over);
    EXPECT_FALSE(r1.passed);
    EXPECT_THROW(skill::open_die_forge::apply(*stock, over), skill::SkillError);

    skill::open_die_forge::Input badTemp;
    badTemp.reduction_pct = 20.0;
    badTemp.temp_c        = -1.0;
    auto r2 = skill::open_die_forge::validate(*stock, badTemp);
    EXPECT_FALSE(r2.passed);
    EXPECT_THROW(skill::open_die_forge::apply(*stock, badTemp), skill::SkillError);
}

// ── 3. SignatureRecordsKind + params ────────────────────────────────────
TEST(SkillOpenDieForge, SignatureRecordsKindAndParams)
{
    auto stock = skill::createCuboidStock(100.0, 80.0, 30.0);

    skill::open_die_forge::Input in;
    in.reduction_pct = 30.0;
    in.temp_c        = 1050.0;

    auto out = skill::open_die_forge::apply(*stock, in);
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("open_die_forge"));
    EXPECT_NEAR(out.signature.params["reduction_pct"].get<double>(), 30.0, 1e-9);
    EXPECT_NEAR(out.signature.params["temp_c"].get<double>(), 1050.0, 1e-9);
    EXPECT_NEAR(out.signature.pattern["effective_reduction_pct"].get<double>(),
                30.0, 1e-3);
    EXPECT_TRUE(out.signature.pattern["geometry_changed"].get<bool>());
    EXPECT_FALSE(out.signature.pattern["shape_constrained"].get<bool>());
}

// ── 4. RecognizeMetadataReplay — exact recovery from feature history ────
TEST(SkillOpenDieForge, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(100.0, 80.0, 30.0);

    skill::open_die_forge::Input in;
    in.reduction_pct = 20.0;
    in.temp_c        = 1200.0;

    auto out = skill::open_die_forge::apply(*stock, in);
    auto cands = skill::open_die_forge::recognize(*out.workpiece);
    ASSERT_GE(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_NEAR(cands[0].recovered_params["reduction_pct"].get<double>(),
                20.0, 1e-9);
    EXPECT_NEAR(cands[0].recovered_params["temp_c"].get<double>(),
                1200.0, 1e-9);
}

// ── 5. SPECIFIC: cold-work temperature emits info-only finding ──────────
TEST(SkillOpenDieForge, ColdTemperatureIsInfoOnly)
{
    auto stock = skill::createCuboidStock(100.0, 80.0, 30.0);

    skill::open_die_forge::Input in;
    in.reduction_pct = 15.0;
    in.temp_c        = 400.0;          // below 700 °C hot-work window

    auto r = skill::open_die_forge::validate(*stock, in);
    EXPECT_TRUE(r.passed)
        << "cold forging temperature should NOT block (info only)";
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-FORGE-COLD") { found = true; break; }
    EXPECT_TRUE(found);
    EXPECT_NO_THROW(skill::open_die_forge::apply(*stock, in));
}
