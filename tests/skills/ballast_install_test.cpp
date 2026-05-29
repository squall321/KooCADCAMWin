// @lat: [[process/test-strategy#skill round-trip]]
//
// ballast_install skill — 5-case sweep covering identity-geometry synthesis,
// DFM rejection of bad input, signature-records-kind+params,
// metadata-only recognition, and the skill-specific assertion that
// ADDED ballast mass is recorded as a negative stock_removed_mm3 (mass-
// balance convention; pig iron ≈ 7.0 t/m³).

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/ballast_install.hpp"

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

skill::ballast_install::Input goodInput()
{
    skill::ballast_install::Input in;
    in.ballast_type    = "pig_iron";
    in.total_mass_t    = 50.0;
    in.locations_count = 4;
    return in;
}

}  // namespace

// ─── 1. Apply: geometry CLONED unchanged ──────────────────────────────────
TEST(SkillBallastInstall, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    auto out = skill::ballast_install::apply(*stock, goodInput());
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("ballast_install"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
}

// ─── 2. Validate REJECTS bad input ───────────────────────────────────────
TEST(SkillBallastInstall, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto in = goodInput();
    in.ballast_type = "";                      // empty
    auto r = skill::ballast_install::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-INPUT"));
    EXPECT_THROW(skill::ballast_install::apply(*stock, in), skill::SkillError);

    auto in2 = goodInput();
    in2.total_mass_t = 0.0;                    // ≤ 0
    auto r2 = skill::ballast_install::validate(*stock, in2);
    EXPECT_FALSE(r2.passed);
    EXPECT_TRUE(hasFinding(r2, "DFM-INPUT"));

    auto in3 = goodInput();
    in3.locations_count = -2;                  // ≤ 0
    auto r3 = skill::ballast_install::validate(*stock, in3);
    EXPECT_FALSE(r3.passed);
    EXPECT_TRUE(hasFinding(r3, "DFM-INPUT"));
}

// ─── 3. Signature records kind + key params ──────────────────────────────
TEST(SkillBallastInstall, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto in = goodInput();
    in.ballast_type    = "concrete";
    in.total_mass_t    = 120.0;
    in.locations_count = 6;
    auto out = skill::ballast_install::apply(*stock, in);

    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("ballast_install"));
    EXPECT_EQ(out.signature.pattern["ballast_type"].get<std::string>(),
              std::string("concrete"));
    EXPECT_NEAR(out.signature.pattern["total_mass_t"].get<double>(),
                120.0, 1e-9);
    EXPECT_EQ(out.signature.pattern["locations_count"].get<int>(), 6);
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
}

// ─── 4. Recognize via metadata replay ─────────────────────────────────────
TEST(SkillBallastInstall, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    auto out = skill::ballast_install::apply(*stock, goodInput());

    auto cands = skill::ballast_install::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["ballast_type"].get<std::string>(),
              std::string("pig_iron"));

    skill::Workpiece raw(out.workpiece->shape());
    auto cands2 = skill::ballast_install::recognize(raw);
    EXPECT_TRUE(cands2.empty())
        << "ballast_install cannot be recognized geometrically";
}

// ─── 5. SPECIFIC: ADDED mass recorded as negative mm3 (ρ_pig ≈ 7 t/m³) ────
TEST(SkillBallastInstall, AdditiveMassRecordedAsNegativeMm3)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto in = goodInput();
    in.total_mass_t = 70.0;                    // 70 t / 7 t/m³ = 10 m³ = 1e10 mm³
    auto out = skill::ballast_install::apply(*stock, in);

    // ADDED mass is recorded as negative stock_removed_mm3 (mass-balance
    // convention, mirrors concrete_pour + spot_weld).
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, -1.0e10, 1.0);
    EXPECT_EQ(out.signature.tooling.tool_type,
              std::string("ballast_handling_rig"));
}
