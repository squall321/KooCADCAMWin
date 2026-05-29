// @lat: [[process/test-strategy#skill round-trip]]
//
// sheet_extrude skill — 5-case sweep: identity-geometry, DFM rejection of
// thickness outside [0.25, 30] mm, signature metadata, metadata-only
// recognition, derived mass_output_kg_h scaling check.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/sheet_extrude.hpp"

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

skill::sheet_extrude::Input goodInput()
{
    skill::sheet_extrude::Input in;
    in.sheet_thickness_mm = 3.0;
    in.width_mm           = 1500.0;
    in.line_speed_m_min   = 10.0;
    return in;
}

}  // namespace

// ─── 1. Apply: geometry CLONED unchanged ──────────────────────────────────
TEST(SkillSheetExtrude, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    auto out = skill::sheet_extrude::apply(*stock, goodInput());
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("sheet_extrude"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. Validate REJECTS bad input ────────────────────────────────────────
TEST(SkillSheetExtrude, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto in = goodInput();
    in.sheet_thickness_mm = 0.10;   // below 0.25 mm — use film
    auto r = skill::sheet_extrude::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-SHEET-THICKNESS"));
    EXPECT_THROW(skill::sheet_extrude::apply(*stock, in), skill::SkillError);

    auto in2 = goodInput();
    in2.sheet_thickness_mm = 50.0;  // above 30 mm
    auto r2 = skill::sheet_extrude::validate(*stock, in2);
    EXPECT_FALSE(r2.passed);
    EXPECT_TRUE(hasFinding(r2, "DFM-SHEET-THICKNESS"));

    auto in3 = goodInput();
    in3.width_mm = 0.0;
    auto r3 = skill::sheet_extrude::validate(*stock, in3);
    EXPECT_FALSE(r3.passed);
    EXPECT_TRUE(hasFinding(r3, "DFM-SHEET-WIDTH"));
}

// ─── 3. Signature records kind + thickness + width + speed ────────────────
TEST(SkillSheetExtrude, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto in = goodInput();
    in.sheet_thickness_mm = 5.0;
    in.width_mm           = 2000.0;
    in.line_speed_m_min   = 15.0;

    auto out = skill::sheet_extrude::apply(*stock, in);

    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("sheet_extrude"));
    EXPECT_NEAR(out.signature.pattern["sheet_thickness_mm"].get<double>(),
                5.0, 1e-9);
    EXPECT_NEAR(out.signature.pattern["width_mm"].get<double>(),
                2000.0, 1e-9);
    EXPECT_NEAR(out.signature.pattern["line_speed_m_min"].get<double>(),
                15.0, 1e-9);
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
}

// ─── 4. Recognize via metadata replay ─────────────────────────────────────
TEST(SkillSheetExtrude, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    auto out = skill::sheet_extrude::apply(*stock, goodInput());

    auto cands = skill::sheet_extrude::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_NEAR(cands[0].recovered_params["sheet_thickness_mm"].get<double>(),
                3.0, 1e-9);

    skill::Workpiece raw(out.workpiece->shape());
    auto cands2 = skill::sheet_extrude::recognize(raw);
    EXPECT_TRUE(cands2.empty())
        << "sheet_extrude cannot be recognized geometrically";
}

// ─── 5. SPECIFIC: mass_output_kg_h scales linearly with thickness ─────────
TEST(SkillSheetExtrude, MassOutputScalesLinearlyWithThickness)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto a = goodInput();
    auto aOut = skill::sheet_extrude::apply(*stock, a);
    const double m_a = aOut.signature.pattern["mass_output_kg_h"].get<double>();

    auto b = goodInput();
    b.sheet_thickness_mm = a.sheet_thickness_mm * 3.0;
    auto bOut = skill::sheet_extrude::apply(*stock, b);
    const double m_b = bOut.signature.pattern["mass_output_kg_h"].get<double>();

    EXPECT_NEAR(m_b, 3.0 * m_a, 1e-6)
        << "tripling sheet thickness should triple mass throughput";
    EXPECT_EQ(aOut.signature.tooling.tool_type,
              std::string("sheet_slot_die_3roll"));
}
