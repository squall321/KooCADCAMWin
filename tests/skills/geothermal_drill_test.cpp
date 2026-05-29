// @lat: [[process/test-strategy#skill round-trip]]
//
// geothermal_drill — 5-case sweep: identity-geometry synthesis, DFM gates
// on depth / BHT / completion, metadata replay, implied gradient computed.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/geothermal_drill.hpp"

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

skill::geothermal_drill::Input goodInput()
{
    skill::geothermal_drill::Input in;
    in.well_depth_m       = 2500.0;
    in.bottom_hole_temp_c = 200.0;
    in.completion_method  = "slotted_liner";
    return in;
}

}  // namespace

// ── 1. Apply: geometry CLONED unchanged ──────────────────────────────────
TEST(SkillGeothermalDrill, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(100.0, 100.0, 50.0);
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    auto out = skill::geothermal_drill::apply(*stock, goodInput());
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("geothermal_drill"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ── 2. Validate REJECTS bad input ────────────────────────────────────────
TEST(SkillGeothermalDrill, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(100.0, 100.0, 50.0);

    auto bad = goodInput();
    bad.well_depth_m = 0.0;
    EXPECT_FALSE(skill::geothermal_drill::validate(*stock, bad).passed);
    EXPECT_THROW(skill::geothermal_drill::apply(*stock, bad),
                 skill::SkillError);

    bad = goodInput();
    bad.bottom_hole_temp_c = 0.0;
    EXPECT_FALSE(skill::geothermal_drill::validate(*stock, bad).passed);
    EXPECT_THROW(skill::geothermal_drill::apply(*stock, bad),
                 skill::SkillError);

    bad = goodInput();
    bad.well_depth_m = -100.0;
    EXPECT_FALSE(skill::geothermal_drill::validate(*stock, bad).passed);
}

// ── 3. Signature records kind + key params ───────────────────────────────
TEST(SkillGeothermalDrill, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCuboidStock(100.0, 100.0, 50.0);

    skill::geothermal_drill::Input in;
    in.well_depth_m       = 4000.0;
    in.bottom_hole_temp_c = 280.0;
    in.completion_method  = "perforated_casing";

    auto out = skill::geothermal_drill::apply(*stock, in);

    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("geothermal_drill"));
    EXPECT_NEAR(out.signature.pattern["well_depth_m"].get<double>(),
                4000.0, 1e-9);
    EXPECT_NEAR(out.signature.pattern["bottom_hole_temp_c"].get<double>(),
                280.0, 1e-9);
    EXPECT_EQ(out.signature.pattern["completion_method"].get<std::string>(),
              std::string("perforated_casing"));
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
    EXPECT_EQ(out.signature.tooling.tool_type,
              std::string("rotary_drilling_rig"));
}

// ── 4. Recognize via metadata replay ─────────────────────────────────────
TEST(SkillGeothermalDrill, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(100.0, 100.0, 50.0);
    auto out = skill::geothermal_drill::apply(*stock, goodInput());

    auto cands = skill::geothermal_drill::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["completion_method"].get<std::string>(),
              std::string("slotted_liner"));
    EXPECT_NEAR(cands[0].recovered_params["well_depth_m"].get<double>(),
                2500.0, 1e-9);

    // Strip metadata → empty.
    skill::Workpiece raw(out.workpiece->shape());
    EXPECT_TRUE(skill::geothermal_drill::recognize(raw).empty());
}

// ── 5. SPECIFIC: implied gradient °C/km computed in pattern ──────────────
TEST(SkillGeothermalDrill, ImpliedGradientCarriedInPattern)
{
    auto stock = skill::createCuboidStock(100.0, 100.0, 50.0);

    // depth = 3000 m, BHT = 200 °C → gradient = (200 - 20) / 3 = 60 °C/km
    skill::geothermal_drill::Input in = goodInput();
    in.well_depth_m       = 3000.0;
    in.bottom_hole_temp_c = 200.0;
    auto out = skill::geothermal_drill::apply(*stock, in);
    EXPECT_NEAR(
        out.signature.pattern["implied_gradient_c_per_km"].get<double>(),
        60.0, 1e-6);

    // Low-temp well: BHT below 120 °C → DFM-GEO-TEMP info, no block.
    skill::geothermal_drill::Input lowT = goodInput();
    lowT.bottom_hole_temp_c = 90.0;
    auto r = skill::geothermal_drill::validate(*stock, lowT);
    EXPECT_TRUE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-GEO-TEMP"));
    EXPECT_NO_THROW(skill::geothermal_drill::apply(*stock, lowT));

    // Unknown completion method → DFM-GEO-COMPL info, no block.
    skill::geothermal_drill::Input weirdCompletion = goodInput();
    weirdCompletion.completion_method = "frac_pack_screen";
    auto r2 = skill::geothermal_drill::validate(*stock, weirdCompletion);
    EXPECT_TRUE(r2.passed);
    EXPECT_TRUE(hasFinding(r2, "DFM-GEO-COMPL"));
}
