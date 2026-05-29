// @lat: [[process/test-strategy#skill round-trip]]
//
// profile_extrude skill — 5-case sweep: identity-geometry, DFM rejection
// of empty profile_id and out-of-range area/speed, signature metadata,
// metadata-only recognition, take-off speed cap check.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/profile_extrude.hpp"

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

skill::profile_extrude::Input goodInput()
{
    skill::profile_extrude::Input in;
    in.profile_id           = "PRF-1234";
    in.cross_section_mm2    = 300.0;
    in.take_off_speed_m_min = 8.0;
    return in;
}

}  // namespace

// ─── 1. Apply: geometry CLONED unchanged ──────────────────────────────────
TEST(SkillProfileExtrude, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    auto out = skill::profile_extrude::apply(*stock, goodInput());
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("profile_extrude"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. Validate REJECTS bad input ────────────────────────────────────────
TEST(SkillProfileExtrude, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto in = goodInput();
    in.profile_id = "";
    auto r = skill::profile_extrude::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-INPUT"));
    EXPECT_THROW(skill::profile_extrude::apply(*stock, in), skill::SkillError);

    auto in2 = goodInput();
    in2.cross_section_mm2 = 0.0;
    auto r2 = skill::profile_extrude::validate(*stock, in2);
    EXPECT_FALSE(r2.passed);
    EXPECT_TRUE(hasFinding(r2, "DFM-PROFILE-AREA"));

    auto in3 = goodInput();
    in3.take_off_speed_m_min = 100.0;  // > 60 m/min cap
    auto r3 = skill::profile_extrude::validate(*stock, in3);
    EXPECT_FALSE(r3.passed);
    EXPECT_TRUE(hasFinding(r3, "DFM-PROFILE-SPEED"));
}

// ─── 3. Signature records kind + profile_id + area + speed ────────────────
TEST(SkillProfileExtrude, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto in = goodInput();
    in.profile_id           = "WINDOW-FRAME-A";
    in.cross_section_mm2    = 1250.0;
    in.take_off_speed_m_min = 4.0;

    auto out = skill::profile_extrude::apply(*stock, in);

    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("profile_extrude"));
    EXPECT_EQ(out.signature.pattern["profile_id"].get<std::string>(),
              std::string("WINDOW-FRAME-A"));
    EXPECT_NEAR(out.signature.pattern["cross_section_mm2"].get<double>(),
                1250.0, 1e-9);
    EXPECT_NEAR(out.signature.pattern["take_off_speed_m_min"].get<double>(),
                4.0, 1e-9);
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
}

// ─── 4. Recognize via metadata replay ─────────────────────────────────────
TEST(SkillProfileExtrude, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    auto out = skill::profile_extrude::apply(*stock, goodInput());

    auto cands = skill::profile_extrude::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["profile_id"].get<std::string>(),
              std::string("PRF-1234"));

    skill::Workpiece raw(out.workpiece->shape());
    auto cands2 = skill::profile_extrude::recognize(raw);
    EXPECT_TRUE(cands2.empty())
        << "profile_extrude cannot be recognized geometrically";
}

// ─── 5. SPECIFIC: mass_output_kg_h grows with cross-section × speed ───────
TEST(SkillProfileExtrude, MassOutputScalesWithAreaTimesSpeed)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto a = goodInput();
    auto aOut = skill::profile_extrude::apply(*stock, a);
    const double m_a = aOut.signature.pattern["mass_output_kg_h"].get<double>();

    auto b = goodInput();
    b.cross_section_mm2    = a.cross_section_mm2 * 2.0;
    b.take_off_speed_m_min = a.take_off_speed_m_min * 3.0;
    auto bOut = skill::profile_extrude::apply(*stock, b);
    const double m_b = bOut.signature.pattern["mass_output_kg_h"].get<double>();

    EXPECT_NEAR(m_b, m_a * 6.0, 1e-6)
        << "mass throughput should scale as A × v";
    EXPECT_EQ(aOut.signature.tooling.tool_type,
              std::string("profile_die_calibrator"));
}
