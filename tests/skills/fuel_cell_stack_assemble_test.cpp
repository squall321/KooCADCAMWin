// @lat: [[process/test-strategy#skill round-trip]]
//
// fuel_cell_stack_assemble — 5-case sweep: identity-geometry synthesis,
// DFM gates, metadata signature replay, expected voltage check.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/fuel_cell_stack_assemble.hpp"

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

skill::fuel_cell_stack_assemble::Input goodInput()
{
    skill::fuel_cell_stack_assemble::Input in;
    in.cell_count      = 100;
    in.mea_type        = "PEM";
    in.stack_voltage_v = 100.0;    // 100 × 1.0 V
    return in;
}

}  // namespace

// ── 1. Apply: geometry CLONED unchanged ──────────────────────────────────
TEST(SkillFuelCellStackAssemble, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(80.0, 80.0, 20.0);
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    auto out = skill::fuel_cell_stack_assemble::apply(*stock, goodInput());
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("fuel_cell_stack_assemble"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ── 2. Validate REJECTS bad input ────────────────────────────────────────
TEST(SkillFuelCellStackAssemble, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(80.0, 80.0, 20.0);

    auto bad = goodInput();
    bad.cell_count = 0;
    EXPECT_FALSE(skill::fuel_cell_stack_assemble::validate(*stock, bad).passed);
    EXPECT_THROW(skill::fuel_cell_stack_assemble::apply(*stock, bad),
                 skill::SkillError);

    bad = goodInput();
    bad.cell_count = -5;
    EXPECT_FALSE(skill::fuel_cell_stack_assemble::validate(*stock, bad).passed);

    bad = goodInput();
    bad.stack_voltage_v = 0.0;
    EXPECT_FALSE(skill::fuel_cell_stack_assemble::validate(*stock, bad).passed);
    EXPECT_THROW(skill::fuel_cell_stack_assemble::apply(*stock, bad),
                 skill::SkillError);
}

// ── 3. Signature records kind + key params ───────────────────────────────
TEST(SkillFuelCellStackAssemble, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCuboidStock(80.0, 80.0, 20.0);

    skill::fuel_cell_stack_assemble::Input in;
    in.cell_count      = 60;
    in.mea_type        = "SOFC";
    in.stack_voltage_v = 54.0;   // 60 × 0.9 V

    auto out = skill::fuel_cell_stack_assemble::apply(*stock, in);

    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("fuel_cell_stack_assemble"));
    EXPECT_EQ(out.signature.pattern["cell_count"].get<int>(), 60);
    EXPECT_EQ(out.signature.pattern["mea_type"].get<std::string>(),
              std::string("SOFC"));
    EXPECT_NEAR(out.signature.pattern["stack_voltage_v"].get<double>(),
                54.0, 1e-9);
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
    EXPECT_EQ(out.signature.tooling.tool_type, std::string("stack_press"));
}

// ── 4. Recognize via metadata replay ─────────────────────────────────────
TEST(SkillFuelCellStackAssemble, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(80.0, 80.0, 20.0);
    auto out = skill::fuel_cell_stack_assemble::apply(*stock, goodInput());

    auto cands = skill::fuel_cell_stack_assemble::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["mea_type"].get<std::string>(),
              std::string("PEM"));
    EXPECT_EQ(cands[0].recovered_params["cell_count"].get<int>(), 100);

    // Strip metadata → empty.
    skill::Workpiece raw(out.workpiece->shape());
    EXPECT_TRUE(skill::fuel_cell_stack_assemble::recognize(raw).empty());
}

// ── 5. SPECIFIC: voltage mismatch ≥ 20 % → DFM-FC-VOLT info ──────────────
TEST(SkillFuelCellStackAssemble, VoltageMismatchEmitsInfo)
{
    auto stock = skill::createCuboidStock(80.0, 80.0, 20.0);

    skill::fuel_cell_stack_assemble::Input in;
    in.cell_count      = 100;          // expected ≈ 100 V at PEM
    in.mea_type        = "PEM";
    in.stack_voltage_v = 50.0;         // 50 % low → info finding

    auto r = skill::fuel_cell_stack_assemble::validate(*stock, in);
    EXPECT_TRUE(r.passed);             // info, not error
    EXPECT_TRUE(hasFinding(r, "DFM-FC-VOLT"));

    // expected_stack_voltage_v carried in pattern
    auto out = skill::fuel_cell_stack_assemble::apply(*stock, in);
    EXPECT_NEAR(out.signature.pattern["expected_stack_voltage_v"].get<double>(),
                100.0, 1e-9);

    // est_cycle_time_s scales with cell_count (60 + 15 × N).
    EXPECT_NEAR(out.signature.tooling.est_cycle_time_s,
                60.0 + 15.0 * 100, 1e-9);
}
