// @lat: [[process/test-strategy#skill round-trip]]
//
// catenary_install skill — 5-case sweep covering identity-geometry
// synthesis, DFM rejection of bad voltage/span/height, metadata-only
// recognition, and AC/DC current-type derivation.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/catenary_install.hpp"

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

skill::catenary_install::Input goodInput()
{
    skill::catenary_install::Input in;
    in.line_voltage_kv = 25.0;
    in.span_m          = 60.0;
    in.height_m        = 5.08;
    return in;
}

}  // namespace

// ─── 1. Apply: geometry CLONED unchanged ──────────────────────────────────
TEST(SkillCatenaryInstall, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    auto out = skill::catenary_install::apply(*stock, goodInput());
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("catenary_install"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. Validate REJECTS bad input (height + span out of range) ───────────
TEST(SkillCatenaryInstall, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto in = goodInput();
    in.height_m = 4.0;                 // < 4.7 m minimum
    auto r = skill::catenary_install::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-CT-HEIGHT"));
    EXPECT_THROW(skill::catenary_install::apply(*stock, in), skill::SkillError);

    auto in2 = goodInput();
    in2.span_m = 100.0;                // > 80 m
    auto r2 = skill::catenary_install::validate(*stock, in2);
    EXPECT_FALSE(r2.passed);
    EXPECT_TRUE(hasFinding(r2, "DFM-CT-SPAN"));

    auto in3 = goodInput();
    in3.line_voltage_kv = 0.4;         // < 0.6 kV
    auto r3 = skill::catenary_install::validate(*stock, in3);
    EXPECT_FALSE(r3.passed);
    EXPECT_TRUE(hasFinding(r3, "DFM-CT-VOLT"));
}

// ─── 3. Signature records kind + voltage/span/height ──────────────────────
TEST(SkillCatenaryInstall, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto in = goodInput();
    in.line_voltage_kv = 15.0;
    in.span_m          = 65.0;
    in.height_m        = 5.5;
    auto out = skill::catenary_install::apply(*stock, in);

    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("catenary_install"));
    EXPECT_NEAR(out.signature.pattern["line_voltage_kv"].get<double>(),
                15.0, 1e-9);
    EXPECT_NEAR(out.signature.pattern["span_m"].get<double>(),
                65.0, 1e-9);
    EXPECT_NEAR(out.signature.pattern["height_m"].get<double>(),
                5.5, 1e-9);
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
}

// ─── 4. Recognize via metadata replay ─────────────────────────────────────
TEST(SkillCatenaryInstall, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    auto out = skill::catenary_install::apply(*stock, goodInput());

    auto cands = skill::catenary_install::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_NEAR(cands[0].recovered_params["line_voltage_kv"].get<double>(),
                25.0, 1e-9);
    EXPECT_NEAR(cands[0].recovered_params["span_m"].get<double>(),
                60.0, 1e-9);

    skill::Workpiece raw(out.workpiece->shape());
    auto cands2 = skill::catenary_install::recognize(raw);
    EXPECT_TRUE(cands2.empty())
        << "catenary_install cannot be recognized geometrically";
}

// ─── 5. SPECIFIC: current_type derives from voltage (>=10 kV → AC) ────────
TEST(SkillCatenaryInstall, CurrentTypeFollowsVoltageThreshold)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto inAC = goodInput();
    inAC.line_voltage_kv = 25.0;
    auto outAC = skill::catenary_install::apply(*stock, inAC);
    EXPECT_EQ(outAC.signature.pattern["current_type"].get<std::string>(),
              std::string("AC"));

    auto inDC = goodInput();
    inDC.line_voltage_kv = 3.0;
    auto outDC = skill::catenary_install::apply(*stock, inDC);
    EXPECT_EQ(outDC.signature.pattern["current_type"].get<std::string>(),
              std::string("DC"));
}
