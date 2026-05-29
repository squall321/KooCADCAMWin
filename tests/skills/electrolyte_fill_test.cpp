// @lat: [[process/test-strategy#skill round-trip]]
//
// electrolyte_fill skill — battery-manufacturing metadata-only sweep.
// Covers:
//   1. apply clones the workpiece without touching geometry
//   2. validate rejects volume / vacuum bad input
//   3. signature records kind + key process params
//   4. recognize replays metadata only
//   5. high vacuum pressure emits info-only warning

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/electrolyte_fill.hpp"

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

// ─── 1. Apply: geometry is COMPLETELY unchanged ──────────────────────────
TEST(SkillElectrolyteFill, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(70.0, 100.0, 6.0, "Al_laminate");
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    skill::electrolyte_fill::Input in;
    in.volume_ml             = 4.5;
    in.electrolyte_type      = "LiPF6_EC_DMC";
    in.vacuum_pressure_mbar  = 30.0;
    in.fill_temperature_c    = 22.0;
    in.dwell_time_min        = 60.0;

    auto out = skill::electrolyte_fill::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("electrolyte_fill"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. DFM: rejects bad inputs ──────────────────────────────────────────
TEST(SkillElectrolyteFill, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(70.0, 100.0, 6.0);

    // 2a. negative volume
    skill::electrolyte_fill::Input badVol;
    badVol.volume_ml            = -1.0;
    badVol.vacuum_pressure_mbar = 50.0;
    auto r1 = skill::electrolyte_fill::validate(*stock, badVol);
    EXPECT_FALSE(r1.passed);
    EXPECT_TRUE(hasFinding(r1, "DFM-INPUT"));
    EXPECT_THROW(skill::electrolyte_fill::apply(*stock, badVol),
                 skill::SkillError);

    // 2b. volume too small for any pouch cell
    skill::electrolyte_fill::Input tiny;
    tiny.volume_ml            = 0.1;
    tiny.vacuum_pressure_mbar = 50.0;
    auto r2 = skill::electrolyte_fill::validate(*stock, tiny);
    EXPECT_FALSE(r2.passed);
    EXPECT_TRUE(hasFinding(r2, "DFM-FILL-VOL-SMALL"));

    // 2c. zero vacuum (no vacuum chamber)
    skill::electrolyte_fill::Input noVac;
    noVac.volume_ml            = 4.0;
    noVac.vacuum_pressure_mbar = 0.0;
    auto r3 = skill::electrolyte_fill::validate(*stock, noVac);
    EXPECT_FALSE(r3.passed);
    EXPECT_TRUE(hasFinding(r3, "DFM-FILL-VAC-LOW"));
}

// ─── 3. Signature records kind + key parameters ──────────────────────────
TEST(SkillElectrolyteFill, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCuboidStock(70.0, 100.0, 6.0);

    skill::electrolyte_fill::Input in;
    in.volume_ml             = 6.2;
    in.electrolyte_type      = "LiFSI_EMC";
    in.vacuum_pressure_mbar  = 25.0;
    in.fill_temperature_c    = 28.0;
    in.dwell_time_min        = 90.0;

    auto out = skill::electrolyte_fill::apply(*stock, in);

    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("electrolyte_fill"));
    EXPECT_NEAR(out.signature.pattern["volume_ml"].get<double>(),
                6.2, 1e-6);
    EXPECT_EQ(out.signature.pattern["electrolyte_type"].get<std::string>(),
              std::string("LiFSI_EMC"));
    EXPECT_NEAR(out.signature.pattern["vacuum_pressure_mbar"].get<double>(),
                25.0, 1e-6);
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
    EXPECT_EQ(out.signature.tooling.tool_type,
              std::string("vacuum_fill_nozzle"));
}

// ─── 4. Recognize replays metadata only ──────────────────────────────────
TEST(SkillElectrolyteFill, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(70.0, 100.0, 6.0);

    skill::electrolyte_fill::Input in;
    in.volume_ml             = 8.0;
    in.electrolyte_type      = "LiPF6_EC_EMC";
    in.vacuum_pressure_mbar  = 40.0;

    auto out = skill::electrolyte_fill::apply(*stock, in);

    auto cands = skill::electrolyte_fill::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_NEAR(cands[0].recovered_params["volume_ml"].get<double>(),
                8.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["electrolyte_type"].get<std::string>(),
              std::string("LiPF6_EC_EMC"));

    skill::Workpiece raw(out.workpiece->shape());
    auto empty = skill::electrolyte_fill::recognize(raw);
    EXPECT_TRUE(empty.empty())
        << "electrolyte_fill cannot be recognized geometrically";
}

// ─── 5. Specific: poor vacuum (>200 mbar) emits info-only warning ───────
TEST(SkillElectrolyteFill, HighVacuumPressureEmitsInfoAndProceeds)
{
    auto stock = skill::createCuboidStock(70.0, 100.0, 6.0);

    skill::electrolyte_fill::Input poorVac;
    poorVac.volume_ml             = 4.0;
    poorVac.electrolyte_type      = "LiPF6_EC_DMC";
    poorVac.vacuum_pressure_mbar  = 500.0;       // far above 200 mbar threshold

    auto r = skill::electrolyte_fill::validate(*stock, poorVac);
    EXPECT_TRUE(r.passed)
        << "poor vacuum should not block the operation (info only)";
    EXPECT_TRUE(hasFinding(r, "DFM-FILL-VAC-HIGH"));
    EXPECT_NO_THROW(skill::electrolyte_fill::apply(*stock, poorVac));
}
