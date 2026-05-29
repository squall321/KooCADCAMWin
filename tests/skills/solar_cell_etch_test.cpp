// @lat: [[process/test-strategy#skill round-trip]]
//
// solar_cell_etch — 5-case sweep: identity-geometry synthesis, DFM gates on
// cell_size / etchant / time_min, metadata signature replay, est cycle time
// scales with bath time.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/solar_cell_etch.hpp"

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

skill::solar_cell_etch::Input goodInput()
{
    skill::solar_cell_etch::Input in;
    in.cell_size_mm = 166.0;   // M6 wafer
    in.etchant      = "KOH";
    in.time_min     = 12.0;
    return in;
}

}  // namespace

// ── 1. Apply: geometry CLONED unchanged ──────────────────────────────────
TEST(SkillSolarCellEtch, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    auto out = skill::solar_cell_etch::apply(*stock, goodInput());
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("solar_cell_etch"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ── 2. Validate REJECTS bad input ────────────────────────────────────────
TEST(SkillSolarCellEtch, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto bad = goodInput();
    bad.cell_size_mm = 0.0;
    EXPECT_FALSE(skill::solar_cell_etch::validate(*stock, bad).passed);
    EXPECT_THROW(skill::solar_cell_etch::apply(*stock, bad), skill::SkillError);

    bad = goodInput();
    bad.time_min = 0.0;
    EXPECT_FALSE(skill::solar_cell_etch::validate(*stock, bad).passed);
    EXPECT_THROW(skill::solar_cell_etch::apply(*stock, bad), skill::SkillError);

    bad = goodInput();
    bad.time_min = -5.0;
    EXPECT_FALSE(skill::solar_cell_etch::validate(*stock, bad).passed);
}

// ── 3. Signature records kind + key params ───────────────────────────────
TEST(SkillSolarCellEtch, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::solar_cell_etch::Input in;
    in.cell_size_mm = 210.0;     // M12 wafer
    in.etchant      = "HF_HNO3";
    in.time_min     = 8.0;

    auto out = skill::solar_cell_etch::apply(*stock, in);

    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("solar_cell_etch"));
    EXPECT_NEAR(out.signature.pattern["cell_size_mm"].get<double>(),
                210.0, 1e-9);
    EXPECT_EQ(out.signature.pattern["etchant"].get<std::string>(),
              std::string("HF_HNO3"));
    EXPECT_NEAR(out.signature.pattern["time_min"].get<double>(), 8.0, 1e-9);
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
    EXPECT_EQ(out.signature.tooling.tool_type, std::string("etch_bath"));
}

// ── 4. Recognize via metadata replay ─────────────────────────────────────
TEST(SkillSolarCellEtch, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    auto out = skill::solar_cell_etch::apply(*stock, goodInput());

    auto cands = skill::solar_cell_etch::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["etchant"].get<std::string>(),
              std::string("KOH"));
    EXPECT_NEAR(cands[0].recovered_params["cell_size_mm"].get<double>(),
                166.0, 1e-9);

    // Strip metadata → empty.
    skill::Workpiece raw(out.workpiece->shape());
    EXPECT_TRUE(skill::solar_cell_etch::recognize(raw).empty());
}

// ── 5. SPECIFIC: est_cycle_time_s == time_min × 60 ───────────────────────
TEST(SkillSolarCellEtch, CycleTimeMatchesBathDuration)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::solar_cell_etch::Input shortIn = goodInput();
    shortIn.time_min = 5.0;
    auto outShort = skill::solar_cell_etch::apply(*stock, shortIn);
    EXPECT_NEAR(outShort.signature.tooling.est_cycle_time_s, 300.0, 1e-9);

    skill::solar_cell_etch::Input longIn = goodInput();
    longIn.time_min = 30.0;
    auto outLong = skill::solar_cell_etch::apply(*stock, longIn);
    EXPECT_NEAR(outLong.signature.tooling.est_cycle_time_s, 1800.0, 1e-9);

    EXPECT_GT(outLong.signature.tooling.est_cycle_time_s,
              outShort.signature.tooling.est_cycle_time_s);

    // Out-of-range wafer + odd etchant should be INFO, not block.
    skill::solar_cell_etch::Input edge = goodInput();
    edge.cell_size_mm = 300.0;       // beyond M12
    edge.etchant      = "novel_acid";
    auto r = skill::solar_cell_etch::validate(*stock, edge);
    EXPECT_TRUE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-PV-CELL-SIZE"));
    EXPECT_TRUE(hasFinding(r, "DFM-PV-ETCHANT"));
}
