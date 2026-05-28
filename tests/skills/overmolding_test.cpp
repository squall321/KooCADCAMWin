// @lat: [[process/test-strategy#skill round-trip]]
//
// overmolding — soft TPE/TPU layer on a rigid substrate (additive shell).
//
// Cases (5):
//   1. apply handles input — adds shell volume.
//   2. validate rejects bad input (out-of-range durometer).
//   3. signature records kind + substrate_material + overmold_material +
//      durometer.
//   4. recognize() replays metadata.
//   5. specific assertion: shell volume matches expanded bbox - substrate bbox.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/overmolding.hpp"

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

// ─── 1. Apply handles input — adds shell volume ────────────────────────────
TEST(SkillOvermolding, ApplyHandlesInput)
{
    auto stock = skill::createCuboidStock(30.0, 20.0, 10.0, "abs");
    const double subVol = volumeOf(stock->shape());

    skill::overmolding::Input in;
    in.substrate_material  = "abs";
    in.overmold_material   = "tpe";
    in.durometer_shore_a   = 60.0;
    in.shell_thickness_mm  = 1.5;

    auto out = skill::overmolding::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    const double newVol = volumeOf(out.workpiece->shape());
    EXPECT_GT(newVol, subVol);
    // Outer bbox = (30+3) × (20+3) × (10+3) = 33 × 23 × 13 = 9867.
    const double expectedOuter = 33.0 * 23.0 * 13.0;
    EXPECT_NEAR(newVol, expectedOuter, expectedOuter * 0.01);
}

// ─── 2. Validate rejects bad input (durometer out of range) ────────────────
TEST(SkillOvermolding, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(30.0, 20.0, 10.0);

    // Durometer too high.
    skill::overmolding::Input badDurom;
    badDurom.substrate_material  = "abs";
    badDurom.overmold_material   = "tpe";
    badDurom.durometer_shore_a   = 110.0;       // > 95
    badDurom.shell_thickness_mm  = 1.5;

    auto r = skill::overmolding::validate(*stock, badDurom);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-OVMOLD-DUROM" && f.severity == "error") {
            found = true;
            break;
        }
    EXPECT_TRUE(found);
    EXPECT_THROW(skill::overmolding::apply(*stock, badDurom),
                 skill::SkillError);

    // Shell too thick.
    skill::overmolding::Input badThick = badDurom;
    badThick.durometer_shore_a   = 60.0;
    badThick.shell_thickness_mm  = 8.0;         // > 6
    auto r2 = skill::overmolding::validate(*stock, badThick);
    EXPECT_FALSE(r2.passed);
    EXPECT_THROW(skill::overmolding::apply(*stock, badThick),
                 skill::SkillError);
}

// ─── 3. Signature records kind + materials + durometer ─────────────────────
TEST(SkillOvermolding, SignatureRecordsKindAndMaterials)
{
    auto stock = skill::createCuboidStock(30.0, 20.0, 10.0, "polycarbonate");

    skill::overmolding::Input in;
    in.substrate_material  = "polycarbonate";
    in.overmold_material   = "tpu";
    in.durometer_shore_a   = 75.0;
    in.shell_thickness_mm  = 2.0;

    auto out = skill::overmolding::apply(*stock, in);
    EXPECT_EQ(out.signature.skill_id, std::string("overmolding"));
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("overmolding"));
    EXPECT_EQ(out.signature.pattern["substrate_material"].get<std::string>(),
              std::string("polycarbonate"));
    EXPECT_EQ(out.signature.pattern["overmold_material"].get<std::string>(),
              std::string("tpu"));
    EXPECT_NEAR(out.signature.pattern["durometer_shore_a"].get<double>(),
                75.0, 1e-9);
    EXPECT_NEAR(out.signature.pattern["shell_thickness_mm"].get<double>(),
                2.0, 1e-9);
    EXPECT_TRUE(out.signature.pattern["additive"].get<bool>());
    EXPECT_TRUE(out.signature.pattern["soft_outer_layer"].get<bool>());
    EXPECT_EQ(out.signature.tooling.tool_type, std::string("overmold"));
}

// ─── 4. Recognize via metadata replay ──────────────────────────────────────
TEST(SkillOvermolding, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(30.0, 20.0, 10.0);

    skill::overmolding::Input in;
    in.substrate_material  = "nylon";
    in.overmold_material   = "tpe";
    in.durometer_shore_a   = 55.0;
    in.shell_thickness_mm  = 1.0;

    auto out   = skill::overmolding::apply(*stock, in);
    auto cands = skill::overmolding::recognize(*out.workpiece);

    ASSERT_EQ(cands.size(), 1u);
    EXPECT_GT(cands[0].confidence, 0.99);
    EXPECT_EQ(cands[0].recovered_params["substrate_material"].get<std::string>(),
              std::string("nylon"));
    EXPECT_EQ(cands[0].recovered_params["overmold_material"].get<std::string>(),
              std::string("tpe"));
    EXPECT_NEAR(cands[0].recovered_params["durometer_shore_a"].get<double>(),
                55.0, 1e-9);

    // Stripped metadata → no recognition.
    skill::Workpiece raw(out.workpiece->shape());
    EXPECT_TRUE(skill::overmolding::recognize(raw).empty());
}

// ─── 5. Specific assertion: shell volume matches bbox expansion math ──────
TEST(SkillOvermolding, ShellVolumeMatchesBboxMath)
{
    auto stock = skill::createCuboidStock(20.0, 20.0, 10.0);
    const double subVol = volumeOf(stock->shape());

    skill::overmolding::Input in;
    in.substrate_material  = "abs";
    in.overmold_material   = "tpe";
    in.durometer_shore_a   = 60.0;
    in.shell_thickness_mm  = 2.5;

    auto out = skill::overmolding::apply(*stock, in);
    const double newVol = volumeOf(out.workpiece->shape());
    const double addedVol = newVol - subVol;

    // Expected added = (25 × 25 × 15) - (20 × 20 × 10) = 9375 - 4000 = 5375
    const double expected = 25.0 * 25.0 * 15.0 - 20.0 * 20.0 * 10.0;
    EXPECT_NEAR(addedVol, expected, expected * 0.02);

    // Signature records the same volume (sign-flipped, per weld_buildup
    // convention: stock_removed < 0 means stock ADDED).
    EXPECT_LT(out.signature.tooling.stock_removed_mm3, 0.0);
    EXPECT_NEAR(std::abs(out.signature.tooling.stock_removed_mm3),
                expected, expected * 0.02);
}
