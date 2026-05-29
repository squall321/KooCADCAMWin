// @lat: [[process/test-strategy#skill round-trip]]
//
// propulsion_install skill — 5-case sweep covering identity-geometry
// synthesis, DFM rejection of out-of-range shaft diameter, signature-
// records-kind+params, metadata-only recognition, and the skill-specific
// assertion that est_cycle_time_s grows with installed power.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/propulsion_install.hpp"

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

skill::propulsion_install::Input goodInput()
{
    skill::propulsion_install::Input in;
    in.propulsion_type = "diesel";
    in.power_kw        = 5000.0;
    in.shaft_dia_mm    = 250.0;
    return in;
}

}  // namespace

// ─── 1. Apply: geometry CLONED unchanged ──────────────────────────────────
TEST(SkillPropulsionInstall, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    auto out = skill::propulsion_install::apply(*stock, goodInput());
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("propulsion_install"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. Validate REJECTS bad input ───────────────────────────────────────
TEST(SkillPropulsionInstall, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto in = goodInput();
    in.shaft_dia_mm = 30.0;                    // < 50 mm minimum
    auto r = skill::propulsion_install::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-PROP-SHAFT"));
    EXPECT_THROW(skill::propulsion_install::apply(*stock, in),
                 skill::SkillError);

    auto in2 = goodInput();
    in2.propulsion_type = "";                  // empty
    auto r2 = skill::propulsion_install::validate(*stock, in2);
    EXPECT_FALSE(r2.passed);
    EXPECT_TRUE(hasFinding(r2, "DFM-INPUT"));

    auto in3 = goodInput();
    in3.power_kw = 0.0;                        // ≤ 0
    auto r3 = skill::propulsion_install::validate(*stock, in3);
    EXPECT_FALSE(r3.passed);
    EXPECT_TRUE(hasFinding(r3, "DFM-INPUT"));
}

// ─── 3. Signature records kind + key params ──────────────────────────────
TEST(SkillPropulsionInstall, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto in = goodInput();
    in.propulsion_type = "azimuth";
    in.power_kw        = 12000.0;
    in.shaft_dia_mm    = 380.0;
    auto out = skill::propulsion_install::apply(*stock, in);

    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("propulsion_install"));
    EXPECT_EQ(out.signature.pattern["propulsion_type"].get<std::string>(),
              std::string("azimuth"));
    EXPECT_NEAR(out.signature.pattern["power_kw"].get<double>(),
                12000.0, 1e-9);
    EXPECT_NEAR(out.signature.pattern["shaft_dia_mm"].get<double>(),
                380.0, 1e-9);
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
}

// ─── 4. Recognize via metadata replay ─────────────────────────────────────
TEST(SkillPropulsionInstall, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    auto out = skill::propulsion_install::apply(*stock, goodInput());

    auto cands = skill::propulsion_install::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["propulsion_type"].get<std::string>(),
              std::string("diesel"));

    skill::Workpiece raw(out.workpiece->shape());
    auto cands2 = skill::propulsion_install::recognize(raw);
    EXPECT_TRUE(cands2.empty())
        << "propulsion_install cannot be recognized geometrically";
}

// ─── 5. SPECIFIC: higher power ⇒ longer estimated cycle time ──────────────
TEST(SkillPropulsionInstall, HigherPowerExtendsEstimatedCycleTime)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto small = goodInput();
    small.power_kw = 1000.0;
    auto smallOut = skill::propulsion_install::apply(*stock, small);

    auto big = goodInput();
    big.power_kw = 40000.0;
    auto bigOut = skill::propulsion_install::apply(*stock, big);

    EXPECT_GT(bigOut.signature.tooling.est_cycle_time_s,
              smallOut.signature.tooling.est_cycle_time_s);
    EXPECT_EQ(bigOut.signature.tooling.tool_type,
              std::string("engine_room_overhead_crane"));
}
