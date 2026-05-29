// @lat: [[process/test-strategy#skill round-trip]]
//
// thermal_vacuum_test skill — TVAC qualification campaign sweep.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/thermal_vacuum_test.hpp"

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
TEST(SkillThermalVacuumTest, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(300.0, 300.0, 400.0,
                                          "aluminum_6061");
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    skill::thermal_vacuum_test::Input in;
    in.chamber_pressure_torr = 1.0e-5;
    in.temp_range_c          = 140.0;     // -70 .. +70
    in.cycles                = 4;

    auto out = skill::thermal_vacuum_test::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("thermal_vacuum_test"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. ValidateRejectsBadInput: chamber pressure too high → error ───────
TEST(SkillThermalVacuumTest, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(300.0, 300.0, 400.0);

    skill::thermal_vacuum_test::Input in;
    in.chamber_pressure_torr = 1.0;        // 1 torr — far too high
    in.temp_range_c          = 100.0;
    in.cycles                = 3;

    auto r = skill::thermal_vacuum_test::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-SPACE-VAC"));
    EXPECT_THROW(skill::thermal_vacuum_test::apply(*stock, in),
                 skill::SkillError);

    skill::thermal_vacuum_test::Input in2;
    in2.chamber_pressure_torr = 1.0e-5;
    in2.temp_range_c          = 500.0;     // > 400 °C envelope
    in2.cycles                = 3;
    auto r2 = skill::thermal_vacuum_test::validate(*stock, in2);
    EXPECT_FALSE(r2.passed);
    EXPECT_TRUE(hasFinding(r2, "DFM-SPACE-TEMP"));
}

// ─── 3. Signature records kind + pressure + temp_range + cycles ──────────
TEST(SkillThermalVacuumTest, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCuboidStock(300.0, 300.0, 400.0);

    skill::thermal_vacuum_test::Input in;
    in.chamber_pressure_torr = 5.0e-6;
    in.temp_range_c          = 165.0;
    in.cycles                = 8;

    auto out = skill::thermal_vacuum_test::apply(*stock, in);
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("thermal_vacuum_test"));
    EXPECT_NEAR(out.signature.pattern["chamber_pressure_torr"].get<double>(),
                5.0e-6, 1e-12);
    EXPECT_NEAR(out.signature.pattern["temp_range_c"].get<double>(),
                165.0, 1e-9);
    EXPECT_EQ(out.signature.pattern["cycles"].get<int>(), 8);
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
}

// ─── 4. Recognize via metadata replay ─────────────────────────────────────
TEST(SkillThermalVacuumTest, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(300.0, 300.0, 400.0);

    skill::thermal_vacuum_test::Input in;
    in.chamber_pressure_torr = 1.0e-5;
    in.temp_range_c          = 120.0;
    in.cycles                = 6;

    auto out  = skill::thermal_vacuum_test::apply(*stock, in);
    auto cands = skill::thermal_vacuum_test::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_NEAR(cands[0].recovered_params["temp_range_c"].get<double>(),
                120.0, 1e-9);

    skill::Workpiece raw(out.workpiece->shape());
    EXPECT_TRUE(skill::thermal_vacuum_test::recognize(raw).empty())
        << "thermal_vacuum_test cannot be recognized geometrically";
}

// ─── 5. SPECIFIC: qualification level distinguishes 8+ vs <8 cycles ──────
TEST(SkillThermalVacuumTest, QualificationLevelByCycles)
{
    auto stock = skill::createCuboidStock(300.0, 300.0, 400.0);

    skill::thermal_vacuum_test::Input accept;
    accept.chamber_pressure_torr = 1.0e-5;
    accept.temp_range_c          = 100.0;
    accept.cycles                = 4;

    skill::thermal_vacuum_test::Input qual;
    qual.chamber_pressure_torr = 1.0e-5;
    qual.temp_range_c          = 100.0;
    qual.cycles                = 8;

    auto outA = skill::thermal_vacuum_test::apply(*stock, accept);
    auto outQ = skill::thermal_vacuum_test::apply(*stock, qual);

    EXPECT_EQ(outA.signature.pattern["qualification_level"].get<std::string>(),
              std::string("acceptance"));
    EXPECT_EQ(outQ.signature.pattern["qualification_level"].get<std::string>(),
              std::string("qualification"));
    EXPECT_EQ(outA.signature.tooling.tool_type, std::string("tvac_chamber"));
}
