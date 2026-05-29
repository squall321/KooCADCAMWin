// @lat: [[process/test-strategy#skill round-trip]]
//
// battery_pack_assemble skill — 5-case sweep.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/battery_pack_assemble.hpp"

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

skill::battery_pack_assemble::Input goodInput()
{
    skill::battery_pack_assemble::Input in;
    in.module_count = 8;
    in.total_kwh    = 60.0;
    in.cooling_type = "liquid";
    return in;
}

}  // namespace

// ─── 1. Apply: geometry CLONED unchanged ──────────────────────────────────
TEST(SkillBatteryPackAssemble, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(2000.0, 1500.0, 150.0);
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    auto out = skill::battery_pack_assemble::apply(*stock, goodInput());
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("battery_pack_assemble"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. Validate REJECTS bad input ────────────────────────────────────────
TEST(SkillBatteryPackAssemble, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(2000.0, 1500.0, 150.0);

    auto in = goodInput();
    in.cooling_type = "freon";            // unknown
    auto r = skill::battery_pack_assemble::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-PACK-COOL"));
    EXPECT_THROW(skill::battery_pack_assemble::apply(*stock, in),
                 skill::SkillError);

    auto in2 = goodInput();
    in2.module_count = 0;
    auto r2 = skill::battery_pack_assemble::validate(*stock, in2);
    EXPECT_FALSE(r2.passed);
    EXPECT_TRUE(hasFinding(r2, "DFM-INPUT"));

    auto in3 = goodInput();
    in3.total_kwh = -5.0;
    auto r3 = skill::battery_pack_assemble::validate(*stock, in3);
    EXPECT_FALSE(r3.passed);
    EXPECT_TRUE(hasFinding(r3, "DFM-INPUT"));
}

// ─── 3. Signature records kind + key params ───────────────────────────────
TEST(SkillBatteryPackAssemble, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCuboidStock(2000.0, 1500.0, 150.0);

    auto in = goodInput();
    in.module_count = 12;
    in.total_kwh    = 95.0;
    in.cooling_type = "air";

    auto out = skill::battery_pack_assemble::apply(*stock, in);
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("battery_pack_assemble"));
    EXPECT_EQ(out.signature.pattern["module_count"].get<int>(), 12);
    EXPECT_NEAR(out.signature.pattern["total_kwh"].get<double>(), 95.0, 1e-9);
    EXPECT_EQ(out.signature.pattern["cooling_type"].get<std::string>(),
              std::string("air"));
    EXPECT_NEAR(out.signature.pattern["kwh_per_module"].get<double>(),
                95.0 / 12.0, 1e-9);
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
}

// ─── 4. Recognize via metadata replay ─────────────────────────────────────
TEST(SkillBatteryPackAssemble, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(2000.0, 1500.0, 150.0);
    auto out = skill::battery_pack_assemble::apply(*stock, goodInput());

    auto cands = skill::battery_pack_assemble::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["cooling_type"].get<std::string>(),
              std::string("liquid"));

    skill::Workpiece raw(out.workpiece->shape());
    auto cands2 = skill::battery_pack_assemble::recognize(raw);
    EXPECT_TRUE(cands2.empty());
}

// ─── 5. SPECIFIC: liquid cooling tool_type + adds coolant connect time ────
TEST(SkillBatteryPackAssemble, LiquidCoolingSelectsLiquidStationAndAddsTime)
{
    auto stock = skill::createCuboidStock(2000.0, 1500.0, 150.0);

    auto inAir = goodInput();
    inAir.cooling_type = "air";
    auto outAir = skill::battery_pack_assemble::apply(*stock, inAir);
    EXPECT_EQ(outAir.signature.tooling.tool_type,
              std::string("air_cooled_pack_assembly_station"));

    auto inLiq = goodInput();
    inLiq.cooling_type = "liquid";
    auto outLiq = skill::battery_pack_assemble::apply(*stock, inLiq);
    EXPECT_EQ(outLiq.signature.tooling.tool_type,
              std::string("liquid_cooled_pack_assembly_station"));

    // Liquid path adds coolant-connect time, so cycle is longer.
    EXPECT_GT(outLiq.signature.tooling.est_cycle_time_s,
              outAir.signature.tooling.est_cycle_time_s);
}
