// @lat: [[process/test-strategy#skill round-trip]]
//
// hull_lay skill — 5-case sweep covering identity-geometry synthesis,
// DFM rejection of bad input, signature-records-kind+params,
// metadata-only recognition, and the skill-specific assertion that
// pattern.footprint_m2 == length_m × beam_m.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/hull_lay.hpp"

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

skill::hull_lay::Input goodInput()
{
    skill::hull_lay::Input in;
    in.hull_section_id = "S12P";
    in.plate_thick_mm  = 14.0;
    in.length_m        = 12.0;
    in.beam_m          = 8.0;
    return in;
}

}  // namespace

// ─── 1. Apply: geometry CLONED unchanged ──────────────────────────────────
TEST(SkillHullLay, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    auto out = skill::hull_lay::apply(*stock, goodInput());
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("hull_lay"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. Validate REJECTS bad input ───────────────────────────────────────
TEST(SkillHullLay, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto in = goodInput();
    in.plate_thick_mm = 2.0;                   // < 4 mm minimum gauge
    auto r = skill::hull_lay::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-HULL-PLATE"));
    EXPECT_THROW(skill::hull_lay::apply(*stock, in), skill::SkillError);

    auto in2 = goodInput();
    in2.hull_section_id = "";                  // empty
    auto r2 = skill::hull_lay::validate(*stock, in2);
    EXPECT_FALSE(r2.passed);
    EXPECT_TRUE(hasFinding(r2, "DFM-INPUT"));

    auto in3 = goodInput();
    in3.length_m = 0.0;                        // ≤ 0
    auto r3 = skill::hull_lay::validate(*stock, in3);
    EXPECT_FALSE(r3.passed);
    EXPECT_TRUE(hasFinding(r3, "DFM-INPUT"));
}

// ─── 3. Signature records kind + key params ──────────────────────────────
TEST(SkillHullLay, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto in = goodInput();
    in.hull_section_id = "B07S";
    in.plate_thick_mm  = 22.0;
    in.length_m        = 18.0;
    in.beam_m          = 10.0;
    auto out = skill::hull_lay::apply(*stock, in);

    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("hull_lay"));
    EXPECT_EQ(out.signature.pattern["hull_section_id"].get<std::string>(),
              std::string("B07S"));
    EXPECT_NEAR(out.signature.pattern["plate_thick_mm"].get<double>(),
                22.0, 1e-9);
    EXPECT_NEAR(out.signature.pattern["length_m"].get<double>(), 18.0, 1e-9);
    EXPECT_NEAR(out.signature.pattern["beam_m"].get<double>(),   10.0, 1e-9);
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
}

// ─── 4. Recognize via metadata replay ─────────────────────────────────────
TEST(SkillHullLay, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    auto out = skill::hull_lay::apply(*stock, goodInput());

    auto cands = skill::hull_lay::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["hull_section_id"].get<std::string>(),
              std::string("S12P"));

    skill::Workpiece raw(out.workpiece->shape());
    auto cands2 = skill::hull_lay::recognize(raw);
    EXPECT_TRUE(cands2.empty())
        << "hull_lay cannot be recognized geometrically";
}

// ─── 5. SPECIFIC: footprint_m2 == length_m × beam_m ───────────────────────
TEST(SkillHullLay, FootprintAreaIsLengthTimesBeam)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto in = goodInput();
    in.length_m = 15.0;
    in.beam_m   = 9.0;
    auto out = skill::hull_lay::apply(*stock, in);

    EXPECT_NEAR(out.signature.pattern["footprint_m2"].get<double>(),
                15.0 * 9.0, 1e-9);
    EXPECT_EQ(out.signature.tooling.tool_type, std::string("gantry_crane"));
}
