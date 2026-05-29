// @lat: [[process/test-strategy#skill round-trip]]
//
// cell_formation skill — battery-manufacturing metadata-only sweep.
// Covers:
//   1. apply clones the workpiece without touching geometry
//   2. validate rejects rate / cycles / capacity bad input
//   3. signature records kind + key formation params
//   4. recognize replays metadata only
//   5. fast C-rate produces info-only finding (SEI quality risk)

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/cell_formation.hpp"

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
TEST(SkillCellFormation, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCylindricalStock(21.0, 70.0, "Al_can");
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    skill::cell_formation::Input in;
    in.charge_rate_c        = 0.1;
    in.cycle_count          = 3;
    in.target_capacity_mah  = 5000.0;
    in.upper_voltage_v      = 4.2;
    in.aging_days           = 14.0;

    auto out = skill::cell_formation::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("cell_formation"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. DFM: rejects bad inputs ──────────────────────────────────────────
TEST(SkillCellFormation, ValidateRejectsBadInput)
{
    auto stock = skill::createCylindricalStock(21.0, 70.0);

    // 2a. zero charge rate
    skill::cell_formation::Input badRate;
    badRate.charge_rate_c       = 0.0;
    badRate.cycle_count         = 3;
    badRate.target_capacity_mah = 3000.0;
    auto r1 = skill::cell_formation::validate(*stock, badRate);
    EXPECT_FALSE(r1.passed);
    EXPECT_TRUE(hasFinding(r1, "DFM-INPUT"));
    EXPECT_THROW(skill::cell_formation::apply(*stock, badRate),
                 skill::SkillError);

    // 2b. zero cycles
    skill::cell_formation::Input noCycles;
    noCycles.charge_rate_c       = 0.1;
    noCycles.cycle_count         = 0;
    noCycles.target_capacity_mah = 3000.0;
    auto r2 = skill::cell_formation::validate(*stock, noCycles);
    EXPECT_FALSE(r2.passed);
    EXPECT_TRUE(hasFinding(r2, "DFM-FORM-CYCLES"));

    // 2c. negative capacity
    skill::cell_formation::Input negCap;
    negCap.charge_rate_c       = 0.1;
    negCap.cycle_count         = 3;
    negCap.target_capacity_mah = -500.0;
    auto r3 = skill::cell_formation::validate(*stock, negCap);
    EXPECT_FALSE(r3.passed);
    EXPECT_THROW(skill::cell_formation::apply(*stock, negCap),
                 skill::SkillError);
}

// ─── 3. Signature records kind + key parameters ──────────────────────────
TEST(SkillCellFormation, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCylindricalStock(21.0, 70.0);

    skill::cell_formation::Input in;
    in.charge_rate_c        = 0.05;
    in.cycle_count          = 5;
    in.target_capacity_mah  = 4800.0;
    in.upper_voltage_v      = 3.65;       // LFP
    in.aging_days           = 21.0;

    auto out = skill::cell_formation::apply(*stock, in);

    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("cell_formation"));
    EXPECT_NEAR(out.signature.pattern["charge_rate_c"].get<double>(),
                0.05, 1e-6);
    EXPECT_EQ(out.signature.pattern["cycle_count"].get<int>(), 5);
    EXPECT_NEAR(out.signature.pattern["target_capacity_mah"].get<double>(),
                4800.0, 1e-6);
    EXPECT_NEAR(out.signature.pattern["upper_voltage_v"].get<double>(),
                3.65, 1e-6);
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
    EXPECT_EQ(out.signature.tooling.tool_type,
              std::string("formation_cycler"));
}

// ─── 4. Recognize replays metadata only ──────────────────────────────────
TEST(SkillCellFormation, RecognizeMetadataReplay)
{
    auto stock = skill::createCylindricalStock(21.0, 70.0);

    skill::cell_formation::Input in;
    in.charge_rate_c        = 0.1;
    in.cycle_count          = 4;
    in.target_capacity_mah  = 3500.0;
    in.upper_voltage_v      = 4.2;
    in.aging_days           = 7.0;

    auto out = skill::cell_formation::apply(*stock, in);

    auto cands = skill::cell_formation::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["cycle_count"].get<int>(), 4);
    EXPECT_NEAR(cands[0].recovered_params["target_capacity_mah"].get<double>(),
                3500.0, 1e-6);

    skill::Workpiece raw(out.workpiece->shape());
    auto empty = skill::cell_formation::recognize(raw);
    EXPECT_TRUE(empty.empty())
        << "cell_formation cannot be recognized geometrically";
}

// ─── 5. Specific: fast C-rate emits SEI quality info, does not block ────
TEST(SkillCellFormation, FastChargeRateEmitsInfoAndProceeds)
{
    auto stock = skill::createCylindricalStock(21.0, 70.0);

    skill::cell_formation::Input fast;
    fast.charge_rate_c        = 1.0;          // 1 C: well above 0.5 C threshold
    fast.cycle_count          = 3;
    fast.target_capacity_mah  = 3000.0;
    fast.upper_voltage_v      = 4.2;

    auto r = skill::cell_formation::validate(*stock, fast);
    EXPECT_TRUE(r.passed) << "fast rate is info-only, must not block";
    EXPECT_TRUE(hasFinding(r, "DFM-FORM-RATE-FAST"));
    EXPECT_NO_THROW(skill::cell_formation::apply(*stock, fast));
}
