// @lat: [[process/test-strategy#skill round-trip]]
//
// cold_form_section skill — 5-case sweep covering identity-geometry synthesis,
// DFM rejection of bad section profile + thickness, metadata-only recognition.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/cold_form_section.hpp"

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

skill::cold_form_section::Input goodInput()
{
    skill::cold_form_section::Input in;
    in.section_profile = "C";
    in.thickness_mm    = 1.6;
    in.length_m        = 3.0;
    return in;
}

}  // namespace

// ─── 1. Apply: geometry CLONED unchanged ──────────────────────────────────
TEST(SkillColdFormSection, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    auto out = skill::cold_form_section::apply(*stock, goodInput());
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("cold_form_section"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. Validate REJECTS bad input (thickness 6 mm > 4 mm max) ───────────
TEST(SkillColdFormSection, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto in = goodInput();
    in.thickness_mm = 6.0;     // exceeds 4 mm cold-form max
    auto r = skill::cold_form_section::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-CFS-THK"));
    EXPECT_THROW(skill::cold_form_section::apply(*stock, in), skill::SkillError);

    auto in2 = goodInput();
    in2.section_profile = "L";     // not C/Z/sigma
    auto r2 = skill::cold_form_section::validate(*stock, in2);
    EXPECT_FALSE(r2.passed);
    EXPECT_TRUE(hasFinding(r2, "DFM-CFS-PROF"));
}

// ─── 3. Signature records kind + profile/thickness/length ────────────────
TEST(SkillColdFormSection, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto in = goodInput();
    in.section_profile = "sigma";
    in.thickness_mm    = 2.0;
    in.length_m        = 6.0;

    auto out = skill::cold_form_section::apply(*stock, in);

    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("cold_form_section"));
    EXPECT_EQ(out.signature.pattern["section_profile"].get<std::string>(),
              std::string("sigma"));
    EXPECT_NEAR(out.signature.pattern["thickness_mm"].get<double>(), 2.0, 1e-9);
    EXPECT_NEAR(out.signature.pattern["length_m"].get<double>(),     6.0, 1e-9);
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
}

// ─── 4. Recognize via metadata replay ─────────────────────────────────────
TEST(SkillColdFormSection, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    auto out = skill::cold_form_section::apply(*stock, goodInput());

    auto cands = skill::cold_form_section::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["section_profile"].get<std::string>(),
              std::string("C"));
    EXPECT_NEAR(cands[0].recovered_params["thickness_mm"].get<double>(),
                1.6, 1e-9);

    // Strip metadata → recognize empty.
    skill::Workpiece raw(out.workpiece->shape());
    auto cands2 = skill::cold_form_section::recognize(raw);
    EXPECT_TRUE(cands2.empty())
        << "cold_form_section cannot be recognized geometrically";
}

// ─── 5. SPECIFIC: longer length ⇒ longer roll-form cycle time ─────────────
TEST(SkillColdFormSection, LongerLengthIncreasesCycleTime)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto shortIn = goodInput();
    shortIn.length_m = 2.0;
    auto shortOut = skill::cold_form_section::apply(*stock, shortIn);

    auto longIn = goodInput();
    longIn.length_m = 10.0;
    auto longOut = skill::cold_form_section::apply(*stock, longIn);

    EXPECT_GT(longOut.signature.tooling.est_cycle_time_s,
              shortOut.signature.tooling.est_cycle_time_s);
    EXPECT_EQ(longOut.signature.tooling.tool_type,
              std::string("roll_former"));
}
