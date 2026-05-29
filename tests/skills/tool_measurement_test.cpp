// @lat: [[process/test-strategy#skill round-trip]]
//
// tool_measurement skill — 5-case sweep covering identity-geometry
// synthesis, DFM error on empty IDs / non-positive dimensions, signature
// metadata round-trip, recognition by metadata replay, and the specific
// ToolingMeta carry of measured dia + length to the CAM tool library.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/tool_measurement.hpp"

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

skill::tool_measurement::Input goodInput()
{
    skill::tool_measurement::Input in;
    in.tool_id         = "EM-6MM-4F-001";
    in.meas_machine_id = "ZOLLER-P410-01";
    in.dia_meas_mm     = 6.000;
    in.length_meas_mm  = 75.000;
    return in;
}

}  // namespace

// ─── 1. Apply: geometry CLONED unchanged ─────────────────────────────────
TEST(SkillToolMeasurement, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCylindricalStock(80.0, 20.0, "carbide");
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    auto out = skill::tool_measurement::apply(*stock, goodInput());
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("tool_measurement"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. Validate REJECTS bad input (empty machine_id, zero dimensions) ───
TEST(SkillToolMeasurement, ValidateRejectsBadInput)
{
    auto stock = skill::createCylindricalStock(80.0, 20.0, "carbide");

    auto in = goodInput();
    in.meas_machine_id = "";                          // empty
    auto r = skill::tool_measurement::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-INPUT"));
    EXPECT_THROW(skill::tool_measurement::apply(*stock, in),
                 skill::SkillError);

    auto in2 = goodInput();
    in2.dia_meas_mm = 0.0;                            // not > 0
    auto r2 = skill::tool_measurement::validate(*stock, in2);
    EXPECT_FALSE(r2.passed);
    EXPECT_TRUE(hasFinding(r2, "DFM-INPUT"));

    auto in3 = goodInput();
    in3.length_meas_mm = -5.0;                        // not > 0
    auto r3 = skill::tool_measurement::validate(*stock, in3);
    EXPECT_FALSE(r3.passed);
    EXPECT_TRUE(hasFinding(r3, "DFM-INPUT"));
}

// ─── 3. Signature records kind + IDs + measured dimensions ───────────────
TEST(SkillToolMeasurement, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCylindricalStock(80.0, 20.0, "carbide");

    auto in = goodInput();
    in.tool_id         = "DR-8MM-2F-COBALT";
    in.meas_machine_id = "SPERONI-MAGIS";
    in.dia_meas_mm     = 8.013;
    in.length_meas_mm  = 102.475;

    auto out = skill::tool_measurement::apply(*stock, in);

    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("tool_measurement"));
    EXPECT_EQ(out.signature.pattern["tool_id"].get<std::string>(),
              std::string("DR-8MM-2F-COBALT"));
    EXPECT_EQ(out.signature.pattern["meas_machine_id"].get<std::string>(),
              std::string("SPERONI-MAGIS"));
    EXPECT_NEAR(out.signature.pattern["dia_meas_mm"].get<double>(),
                8.013, 1e-9);
    EXPECT_NEAR(out.signature.pattern["length_meas_mm"].get<double>(),
                102.475, 1e-9);
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
}

// ─── 4. Recognize via metadata replay ─────────────────────────────────────
TEST(SkillToolMeasurement, RecognizeMetadataReplay)
{
    auto stock = skill::createCylindricalStock(80.0, 20.0, "carbide");
    auto out = skill::tool_measurement::apply(*stock, goodInput());

    auto cands = skill::tool_measurement::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["meas_machine_id"].get<std::string>(),
              std::string("ZOLLER-P410-01"));

    skill::Workpiece raw(out.workpiece->shape());
    auto cands2 = skill::tool_measurement::recognize(raw);
    EXPECT_TRUE(cands2.empty())
        << "tool_measurement cannot be recognized geometrically";
}

// ─── 5. SPECIFIC: measured dia/length flow into ToolingMeta + inspection flag ──
TEST(SkillToolMeasurement, ToolingMetaCarriesMeasuredDimensions)
{
    auto stock = skill::createCylindricalStock(80.0, 20.0, "carbide");

    auto in = goodInput();
    in.dia_meas_mm    = 12.007;
    in.length_meas_mm = 85.330;
    auto out = skill::tool_measurement::apply(*stock, in);

    // Pre-setter measurement is an inspection-only record — flag must be true
    // and ToolingMeta must carry the measured offsets for the CAM library.
    EXPECT_TRUE(out.signature.pattern["is_inspection_only"].get<bool>());
    EXPECT_NEAR(out.signature.tooling.tool_dia_mm,    12.007, 1e-9);
    EXPECT_NEAR(out.signature.tooling.tool_length_mm, 85.330, 1e-9);
    EXPECT_EQ(out.signature.tooling.tool_type,
              std::string("tool_presetter"));
}
