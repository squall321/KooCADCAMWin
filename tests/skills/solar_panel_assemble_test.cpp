// @lat: [[process/test-strategy#skill round-trip]]
//
// solar_panel_assemble skill — 5-case sweep.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/solar_panel_assemble.hpp"

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

skill::solar_panel_assemble::Input goodInput()
{
    skill::solar_panel_assemble::Input in;
    in.cell_count   = 60;
    in.cell_size_mm = 166.0;
    in.encapsulant  = "EVA";
    return in;
}

}  // namespace

// ─── 1. Apply: geometry CLONED unchanged ──────────────────────────────────
TEST(SkillSolarPanelAssemble, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    auto out = skill::solar_panel_assemble::apply(*stock, goodInput());
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("solar_panel_assemble"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. Validate REJECTS bad input ────────────────────────────────────────
TEST(SkillSolarPanelAssemble, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto in = goodInput();
    in.cell_count = -10;
    auto r = skill::solar_panel_assemble::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-INPUT"));
    EXPECT_THROW(skill::solar_panel_assemble::apply(*stock, in), skill::SkillError);

    auto in2 = goodInput();
    in2.cell_size_mm = 50.0;     // < 125 mm
    auto r2 = skill::solar_panel_assemble::validate(*stock, in2);
    EXPECT_FALSE(r2.passed);
    EXPECT_TRUE(hasFinding(r2, "DFM-PV-CELL-SIZE"));
}

// ─── 3. Signature records kind + cell_count + cell_size + encapsulant ────
TEST(SkillSolarPanelAssemble, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto in = goodInput();
    in.cell_count   = 72;
    in.cell_size_mm = 182.0;
    in.encapsulant  = "POE";

    auto out = skill::solar_panel_assemble::apply(*stock, in);

    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("solar_panel_assemble"));
    EXPECT_EQ(out.signature.pattern["cell_count"].get<int>(), 72);
    EXPECT_NEAR(out.signature.pattern["cell_size_mm"].get<double>(),
                182.0, 1e-9);
    EXPECT_EQ(out.signature.pattern["encapsulant"].get<std::string>(),
              std::string("POE"));
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
}

// ─── 4. Recognize via metadata replay ─────────────────────────────────────
TEST(SkillSolarPanelAssemble, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    auto out = skill::solar_panel_assemble::apply(*stock, goodInput());

    auto cands = skill::solar_panel_assemble::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["cell_count"].get<int>(), 60);

    skill::Workpiece raw(out.workpiece->shape());
    auto cands2 = skill::solar_panel_assemble::recognize(raw);
    EXPECT_TRUE(cands2.empty());
}

// ─── 5. SPECIFIC: total_active_area_mm2 == N × cell_size² ────────────────
TEST(SkillSolarPanelAssemble, TotalActiveAreaMatchesCellsTimesArea)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto in = goodInput();
    in.cell_count   = 60;
    in.cell_size_mm = 158.75;

    auto out = skill::solar_panel_assemble::apply(*stock, in);

    const double expectedTotal =
        60.0 * 158.75 * 158.75;
    EXPECT_NEAR(out.signature.pattern["total_active_area_mm2"].get<double>(),
                expectedTotal, 1e-3);
    EXPECT_EQ(out.signature.tooling.tool_type,
              std::string("stringer_laminator"));
}
