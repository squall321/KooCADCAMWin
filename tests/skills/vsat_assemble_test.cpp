// @lat: [[process/test-strategy#skill round-trip]]
//
// vsat_assemble — 5-case sweep: identity-geometry synthesis, DFM gates on
// dish_dia / transponder, metadata replay, polarization-info specific case.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/vsat_assemble.hpp"

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

}  // namespace

// ── 1. Apply: geometry is COMPLETELY unchanged ───────────────────────────
TEST(SkillVsatAssemble, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    skill::vsat_assemble::Input in;
    in.dish_dia_m   = 1.2;
    in.transponder  = "Ku-A1";
    in.polarization = "linear_v";

    auto out = skill::vsat_assemble::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("vsat_assemble"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ── 2. DFM: zero dish, empty transponder rejected ────────────────────────
TEST(SkillVsatAssemble, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::vsat_assemble::Input bad;
    bad.dish_dia_m   = 0.0;                // invalid
    bad.transponder  = "Ku-A1";
    bad.polarization = "linear_v";
    EXPECT_FALSE(skill::vsat_assemble::validate(*stock, bad).passed);
    EXPECT_THROW(skill::vsat_assemble::apply(*stock, bad), skill::SkillError);

    bad.dish_dia_m   = 1.2;
    bad.transponder  = "";                 // invalid
    EXPECT_FALSE(skill::vsat_assemble::validate(*stock, bad).passed);
    EXPECT_THROW(skill::vsat_assemble::apply(*stock, bad), skill::SkillError);

    // Out-of-range dish dia is WARNING (not error) — still passes.
    skill::vsat_assemble::Input warn;
    warn.dish_dia_m   = 0.3;               // below 0.6 m typical range
    warn.transponder  = "Ku-A1";
    warn.polarization = "linear_v";
    auto r = skill::vsat_assemble::validate(*stock, warn);
    EXPECT_TRUE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-VSAT-DIA"));
}

// ── 3. Signature records kind + dia/transponder/polarization ─────────────
TEST(SkillVsatAssemble, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::vsat_assemble::Input in;
    in.dish_dia_m   = 2.4;
    in.transponder  = "Ka-B3";
    in.polarization = "rhcp";

    auto out = skill::vsat_assemble::apply(*stock, in);
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("vsat_assemble"));
    EXPECT_NEAR(out.signature.pattern["dish_dia_m"].get<double>(), 2.4, 1e-9);
    EXPECT_EQ(out.signature.pattern["transponder"].get<std::string>(),
              std::string("Ka-B3"));
    EXPECT_EQ(out.signature.pattern["polarization"].get<std::string>(),
              std::string("rhcp"));
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
    EXPECT_EQ(out.signature.tooling.tool_type, std::string("vsat_assembly_kit"));
    // tool_dia_mm mirrors dish dia in mm.
    EXPECT_NEAR(out.signature.tooling.tool_dia_mm, 2400.0, 1e-6);
}

// ── 4. Recognize via metadata replay ─────────────────────────────────────
TEST(SkillVsatAssemble, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::vsat_assemble::Input in;
    in.dish_dia_m   = 1.8;
    in.transponder  = "C-7";
    in.polarization = "linear_h";

    auto out = skill::vsat_assemble::apply(*stock, in);
    auto cands = skill::vsat_assemble::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_NEAR(cands[0].recovered_params["dish_dia_m"].get<double>(), 1.8, 1e-9);
    EXPECT_EQ(cands[0].recovered_params["transponder"].get<std::string>(),
              std::string("C-7"));

    skill::Workpiece raw(out.workpiece->shape());
    EXPECT_TRUE(skill::vsat_assemble::recognize(raw).empty());
}

// ── 5. SPECIFIC: unknown polarization emits VSAT-POL info, passes ────────
TEST(SkillVsatAssemble, UnknownPolarizationEmitsInfo)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::vsat_assemble::Input in;
    in.dish_dia_m   = 1.2;
    in.transponder  = "Ku-A1";
    in.polarization = "elliptical_X";       // not in the known-4 set

    auto r = skill::vsat_assemble::validate(*stock, in);
    EXPECT_TRUE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-VSAT-POL"));
    EXPECT_NO_THROW(skill::vsat_assemble::apply(*stock, in));

    // Known polarization → no info finding emitted.
    skill::vsat_assemble::Input ok;
    ok.dish_dia_m   = 1.2;
    ok.transponder  = "Ku-A1";
    ok.polarization = "linear_h";
    auto r2 = skill::vsat_assemble::validate(*stock, ok);
    EXPECT_TRUE(r2.passed);
    EXPECT_FALSE(hasFinding(r2, "DFM-VSAT-POL"));
}
